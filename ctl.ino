#include <string.h>
#include <SoftwareSerial.h>

// timing constants
#define SECONDS(X) ((X) * 1000)

#define IDENTIFY_TIMEOUT SECONDS(10)
#define STAY_OPEN        SECONDS(10)

// pin numbers
#define PIN_RFID_TX 			2
#define PIN_RFID_RX 			3
#define PIN_ENABLE_OPEN_DETECTION	4
#define PIN_DOOR_OPEN_DETECT            5
#define PIN_ENABLE_SHUT_DETECTION       6
#define PIN_DOOR_SHUT_DETECT            11
#define PIN_IR_EMIT			12  // operate @ 38Khz, ~30% duty
#define PIN_IR_DETECT                   13 
#define PIN_MOTION_DETECT               14  // n.b. active low
#define PIN_MOTOR_FORWARD		15
#define PIN_MOTOR_REVERSE		16
#define PIN_ENABLE_MOTOR                17

// sensor state signalling values
#define TIMED_OUT  1
#define IDENTIFIED 0
#define OBSTRUCTED 1
#define CLOSED     0

// system interface

SoftwareSerial rfid(PIN_RFID_RX, PIN_RFID_TX);

void check_for_notag(void);
void rfid_halt(void);
void parse(void);
void print_serial(void);
void read_serial(void);
void rfid_seek(void);
void set_flag(void);

uint8_t tag[] = { 0x87, 0x9D, 0x64, 0x02 };

enum RfidCmd
{
  RESET    = 0x80,
  FIRMWARE = 0x81,
  SEEK     = 0x82,
  POWER    = 0x90,
  HALT     = 0x93
};

void rfid_send_frame(uint8_t cmd, uint8_t *data, uint8_t length)
{
  uint8_t payload_length = length + 1;
  uint8_t checksum = 0x00 + payload_length + cmd;
  rfid.write((uint8_t)0xFF);			// header
  rfid.write((uint8_t)0x00);			// reserved, unimplimented
  rfid.write((uint8_t)payload_length);		// payload byte count including cmd
  rfid.write((uint8_t)cmd);              	// command
  for (uint8_t i = 0; i < length; ++i)	// data
  {
    rfid.write(data[i]);
    checksum += data[i];
  }
  rfid.write((uint8_t)checksum);			// checksum ends packet
}

void rfid_halt(void)
{
  rfid_send_frame(HALT, 0, 0);
}

void rfid_seek()
{
  rfid_send_frame(SEEK, 0, 0);
  delay(10);
}

bool rfid_timeout_read(unsigned long expire_time, uint8_t *out)
{
  while (millis() < expire_time)
  {
    if (rfid.available() > 0)
    {
      *out = rfid.read();
      return true;
    }

    delay(20);
  }

  return false;
}

bool rfid_read_seek_response(unsigned long expire_time)
{
  uint8_t b;

  // ignore everything up to packet frame header
  while (1)
  {
    if (!rfid_timeout_read(expire_time, &b))
      return false;
    if (b == 0xFF)
      break;
   }

  // require rest of packet header
  if (!rfid_timeout_read(expire_time, &b) || b != 0x00)
    return false;
  
  if (!rfid_timeout_read(expire_time, &b) || b != 0x02)
    return false;

  // require seek command reply
  if (!rfid_timeout_read(expire_time, &b) || b != SEEK)
    return false;
  
  // require success
  if (!rfid_timeout_read(expire_time, &b) || b != 0x4C)
    return false;

  // require checksum
  if (!rfid_timeout_read(expire_time, &b) || b != 0xD0)
    return false;

  return true;  
}

bool rfid_cmp_tag_packet(unsigned long expire_time)
{
  uint8_t l, b;

  // ignore everything up to packet frame header
  while (1)
  {
    if (!rfid_timeout_read(expire_time, &b))
      return false;
    if (b == 0xFF)
      break;
   }

  // require reserved byte
  if (!rfid_timeout_read(expire_time, &b) || b != 0x00)
    return false;                             // bad packet frame

  // read payload length. needs to be a sensible tag length
  if (!rfid_timeout_read(expire_time, &b) || (b != 6 && b != 9))
    return false;

  // ought to be a reply to a seek command
  if (!rfid_timeout_read(expire_time, &b) || b != SEEK)
    return false;

  // ignore "tag type" byte
  if (!rfid_timeout_read(expire_time, &b))
    return false;

  for (int i = l - 3; i >= 0; --i)
  {
    if (!rfid_timeout_read(expire_time, &b))
      return false;
    if (tag[i] != b)
    {
      Serial.print(tag[i], HEX);
      Serial.print(" != ");
      Serial.println(b, HEX);
      return false;
    }
  }
  return true;
}

bool rfid_do_id(unsigned long expire_time)
{
  rfid.begin(19200);
  delay(10);

  while (millis() < expire_time)
  {
    rfid_seek();
    delay(10);

    if (!rfid_read_seek_response(expire_time))
    {
      rfid.end();
      return false;
    }

    if (rfid_cmp_tag_packet(expire_time))
    {
      rfid.end();
      return true;
    }
  }
  rfid.end();
  return false;
}

void motion_wait()
{
  while (digitalRead(PIN_MOTION_DETECT) != LOW) delay(100);
}

int wait_id_or_timeout(unsigned long timeout)
{
  if (rfid_do_id(millis() + timeout))
    return IDENTIFIED;
  return TIMED_OUT;
}

void door_open()
{
  digitalWrite(PIN_ENABLE_MOTOR, HIGH);
  digitalWrite(PIN_MOTOR_FORWARD, HIGH);
  digitalWrite(PIN_MOTOR_REVERSE, LOW);
}

void door_close()
{
  digitalWrite(PIN_ENABLE_MOTOR, HIGH);
  digitalWrite(PIN_MOTOR_REVERSE, HIGH);
  digitalWrite(PIN_MOTOR_FORWARD, LOW);
}

void door_stop()
{
  digitalWrite(PIN_ENABLE_MOTOR, LOW);
}

void wait_door_open()
{
  digitalWrite(PIN_ENABLE_OPEN_DETECTION, HIGH);
  while (digitalRead(PIN_DOOR_OPEN_DETECT) != HIGH) delay(100);
  digitalWrite(PIN_ENABLE_OPEN_DETECTION, LOW);
}

// block timeout milliseconds
void wait_timer(unsigned long timeout)
{
  delay(timeout);
}

int wait_closed_or_obstructed()
{
  const unsigned int period = 19; // 38KHz found by trial and error 
  const float duty = 0.3;         // 30% duty cycle
  const unsigned int delay1 = period * duty;
  const unsigned int delay2 = period - delay1;
  
  digitalWrite(PIN_ENABLE_SHUT_DETECTION, HIGH);

  // run ir emitter for 10000 periods so detector can find signal
  for (i = 0; i < 10000; ++i)
  {
    digitalWrite(PIN_IR_EMIT, HIGH);
    delayMicroseconds(delay1);
    digitalWrite(PIN_IR_EMIT, LOW);
    delayMicroseconds(delay2);

    // calls to make timing identical to the following loop
    if (digitalRead(PIN_DOOR_SHUT_DETECT) == HIGH);
    if (digitalRead(PIN_IR_DETECT) == HIGH);
  }

  while (1)
  {
    digitalWrite(PIN_IR_EMIT, HIGH);
    delayMicroseconds(delay1);
    digitalWrite(PIN_IR_EMIT, LOW);
    delayMicroseconds(delay2);

    if (digitalRead(PIN_DOOR_SHUT_DETECT) == HIGH)
    {
      digitalWrite(PIN_ENABLE_SHUT_DETECTION, LOW);
      return CLOSED;
    }
    if (digitalRead(PIN_IR_DETECT) == HIGH)
    {
      digitalWrite(PIN_ENABLE_SHUT_DETECTION, LOW);
      return OBSTRUCTED;
    }
  }
}

// system control state machine

enum State
{
  WAIT,				// 000b
  IDENTIFYING,			// 001b
  OPENING,			// 010b
  ENTERING,			// 011b
  CLOSING			// 100b
};

// global state
State state;

// trampolined mainloop
void loop()
{
  switch (state)
  {
    case WAIT:
      motion_wait();
      state = IDENTIFYING;
      return;
    case IDENTIFYING:
      if (wait_id_or_timeout(IDENTIFY_TIMEOUT) == TIMED_OUT)
        state = WAIT;
      else
        state = OPENING;
      return;
    case OPENING:
      door_open();
      wait_door_open();
      door_stop();
      state = ENTERING;
      return;
    case ENTERING:
       wait_timer(STAY_OPEN);
       state = CLOSING;
    case CLOSING:
    default:
       door_close();
       if (wait_closed_or_obstructed() == OBSTRUCTED)
       {
         door_stop();
         state = ENTERING;
       }
       else
       {
         door_stop();
         state = WAIT;
       }
       return;
  }
}

// main
void setup()
{
  Serial.begin(9600);

  pinMode(PIN_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_REVERSE, OUTPUT);
  pinMode(PIN_ENABLE_MOTOR, OUTPUT);

  pinMode(PIN_ENABLE_OPEN_DETECTION, OUTPUT);
  pinMode(PIN_DOOR_OPEN_DETECT, INPUT);

  pinMode(PIN_MOTION_DETECT, INPUT);

  pinMode(PIN_ENABLE_SHUT_DETECTION, OUTPUT);
  pinMode(PIN_DOOR_SHUT_DETECT, INPUT);

  pinMode(PIN_IR_EMIT, OUTPUT);
  pinMode(PIN_IR_DETECT, INPUT);

  pinMode(PIN_RFID_RX, INPUT);
  pinMode(PIN_RFID_TX, OUTPUT);

  // make sure door moves to closed position on system start
  state = CLOSING;
}

