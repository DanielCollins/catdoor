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
#define PIN_IR_DETECT                   13  // active high??
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

void motion_wait()
{
  while (digitalRead(PIN_MOTION_DETECT) != HIGH) delay(100);
}

int wait_id_or_timeout(unsigned long timeout)
{
  lol;
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
  lol;
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

  // make sure door moves to closed position on system start
  state = CLOSING;
}

