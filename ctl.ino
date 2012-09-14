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

SoftwareSerial RFID = SoftwareSerial(PIN_RFID_RX, PIN_RFID_TX);
char tag[13] = "AAAAAAAAAAAA";

void motion_wait()
{
  while (digitalRead(PIN_MOTION_DETECT) != LOW) delay(100);
}

int wait_id_or_timeout(unsigned long timeout)
{
  int val = 0;
  int bytesread = 0;
  int i;
  int expire_time = millis() + timeout;
  char code[13];

  RFID.begin(9600);

  while (millis() < expire_time)
  {
    i = 0;
    while (RFID.available > 0 && bytesread < 12 && i < 24)
    {
      val = RFID.read();

      if (val == 3)
        break;

      if (val == 2)
        continue;

      code[bytesread++] = val;
      code[bytesread] = '\0';

      if (bytesread == 12)
      {
        if (!strcmp(code, tag))
        {
          RFID.end();
          return IDENTIFIED;
        }
        else
          lolwut;
      }

      ++i;        
    }

    delay(100);
    
  }
  RFID.end();
  return TIMED_OUT;
}

void door_open()
{
  digitalWrite(PIN_ENABLE_MOTOR, HIGH);
  digitalWrite(PIN_MOTOR_FORWARD, HIGH);
}

void door_close()
{
  digitalWrite(PIN_ENABLE_MOTOR, HIGH);
  digitalWrite(PIN_MOTOR_REVERSE, HIGH);
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

