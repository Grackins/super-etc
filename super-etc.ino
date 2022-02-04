/*
  FH 2014 Throttle and Shift.ino

Bosch electronic throttle body control
for MSOE SAE Formula Hybrid 2014

Author: Austin R. Bartz
Last Revision Date: 4/6/2014

Hardware Connections:
-TPS 0:                   Pin A0 (Blue Wire)
-TPS 1:                   Pin A1 (Thin White Wire)
-Throttle Input 0:        Pin A2 (White Wire)
-Throttle Input 1:        Pin A3 (White Marked Wire)
-Error LED                Pin A5 (Blue Wire)
-UpShift Relay:           Pin 2  (Yellow Wire)
-DownShift Relay:         Pin 3  (Pink Wire)
-UpShift Button:          Pin 4  (Green Wire)
-DownShift Button:        Pin 5  (Orange Wire)
-Ignition Cut             Pin 6  (White with Green Stripe Wire)
-Neutral Switch           Pin 7  (Brown Wire)
-L298N H-Bridge Enable A: Pin 9  (N/A)
-L298N H-Bridge Input 1:  Pin 8  (Other Speaker Wire)
-L298N H-Bridge Input 2:  Pin 11 (Gray Line Wire)
*/

//PID Library
#include <PID_v1.h>

//Pins assignments
#define pinI1 6
#define pinI2 7
#define speedPin 11

#define PEDAL_MODE_SERIAL   0
#define PEDAL_MODE_PHYSICAL     1
#define PEDAL_MODE_DEFAULT     PEDAL_MODE_SERIAL

#define TPS_MODE_BARE       0
#define TPS_MODE_MED3       1
#define TPS_MODE_MED5       2
#define TPS_MODE      TPS_MODE_MED3

#define TPS_CLOSED_POSITION 100
#define TPS_IDLE_POSITION   200
#define TPS_OPEN_POSITION   900

struct Settings {
  bool capture = 0;
  char pedalMode = PEDAL_MODE_DEFAULT;
  unsigned long int counter = 0;
} settings;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,1,0,0, DIRECT);
String msg;
unsigned int value = 0;
unsigned int last = -1;
static unsigned int prevValue = 255;
static unsigned int prevLast = -1;

#if PEDAL_MODE == PEDAL_MODE_SERIAL
unsigned int lastReadPedal = TPS_IDLE_POSITION;
#endif

int getTps1() {
  int currentValue = analogRead(A1);
#if TPS_MODE == TPS_MODE_BARE
  return currentValue;
#elif TPS_MODE == TPS_MODE_MED3
  static int buf[3] = {TPS_IDLE_POSITION, TPS_IDLE_POSITION, TPS_IDLE_POSITION};
  static uint8_t idx = 0;
  buf[idx] = currentValue;
  idx = (idx == 2) ? 0 : idx + 1;
  if (buf[0] <= buf[1])
    if (buf[1] <= buf[2])
      return buf[1];
    else if (buf[2] <= buf[0])
      return buf[0];
    else
      return buf[2];
  else // if(buf[1] < buf[0])
    if (buf[0] <= buf[2])
      return buf[0];
    else if (buf[2] <= buf[1])
      return buf[1];
    else
      return buf[2];
#elif TPS_MODE == TPS_MODE_MED5
  static int buf[5] = {TPS_IDLE_POSITION, TPS_IDLE_POSITION, TPS_IDLE_POSITION, TPS_IDLE_POSITION, TPS_IDLE_POSITION};
  static int sorted[5] = {TPS_IDLE_POSITION, TPS_IDLE_POSITION, TPS_IDLE_POSITION, TPS_IDLE_POSITION, TPS_IDLE_POSITION};
  static uint8_t idx = 0;
  buf[idx] = currentValue;
  idx = (idx == 4) ? 0 : idx + 1;
  for (int i = 0; i < 5; i++)
    sorted[i] = buf[i];
  for (int i = 0; i < 5; i++)
    for (int j = 0; j + 1 <= i; j++)
      if (sorted[j] > sorted[j + 1]) {
        int tmp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = tmp;
      }
  return sorted[2];
#endif
}

int getPedalPosition() {
  int value;
  if (settings.pedalMode == PEDAL_MODE_SERIAL)
    value = lastReadPedal;
  else if (settings.pedalMode == PEDAL_MODE_PHYSICAL)
    value = analogRead(A0);
  return value;
}

ISR(TIMER3_COMPA_vect) {
  settings.counter++;
//  return;
  int tpsValue = getTps1();
  Input = (double) tpsValue;
 
  // Read Setpoint from throttle pedal
  Setpoint = (double) getPedalPosition();

  if(Setpoint < TPS_IDLE_POSITION)
  {
    Serial.println("WARN: Setpoint under TPS_IDLE_POSITION");
    analogWrite(speedPin,0);
  }
  else
  {
    myPID.Compute();
    analogWrite(speedPin, Output);
  }
  if (settings.capture) {
//    Serial.println((int) Output);
    Serial.println(tpsValue);
  }
}

void setup()
{
  cli();
  // Change PWM frequency of pin 11
  TCCR1B = TCCR1B & 0b11111000 | 0x00000001;

  // Config pid timer interrupt
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  // Interrupts each 0.96 millis
  OCR3A = 14;
  TCCR3B |= 1 << WGM32;
  TCCR3B |= (1 << CS30) | (1 << CS32);
  TIMSK3 |= 1 << OCIE3A;
  sei();

  pinMode(pinI1,OUTPUT); 
  pinMode(pinI2,OUTPUT);
  pinMode(speedPin,OUTPUT);
  digitalWrite(pinI1,LOW);
  digitalWrite(pinI2,HIGH);
  Serial.begin(57600);

  Input = TPS_IDLE_POSITION;
  Setpoint = TPS_IDLE_POSITION;
  myPID.SetMode(AUTOMATIC);
}

void setSetpoint(double value) {
  lastReadPedal = value;
  // TODO log
}

void loop()
{
//  static int last = 0;
//  if (millis() > last + 1000) {
//    last = millis();
//    Serial.println(counter);  
//  }
//  return;
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command[0] == '?')
      Serial.println(settings.counter);
    else if (command[0] == 'R') {
      setSetpoint((double) TPS_IDLE_POSITION);
      // TODO
    }
    else if (command[0] == 'L') {
      settings.capture = command.length() == 2 && command[1] == '1';
    }
    else if (command[0] == 'S') {
      int value = command.substring(1).toInt();
      if (value != 0) {
        setSetpoint((double) value);
      }
      else {
        // TODO
        return;
      }
    }
    else if (command[0] == 'G') {
      settings.pedalMode = command.length() == 2 && command[1] == '1';
    }
    else if (command[0] == 'C') {
      int pos[2] = {0};
      pos[0] = command.indexOf('|');
      if (pos[0] == -1) {
        return;
      }
      pos[1] = command.indexOf('|', pos[0] + 1);
      if (pos[1] == -1) {
        return;
      }
      if (command.indexOf('|', pos[1] + 1) != -1) {
        return;
      }
      double Kp, Ki, Kd;
      Kp = command.substring(1, pos[0]).toFloat();
      Ki = command.substring(pos[0] + 1, pos[1]).toFloat();
      Kd = command.substring(pos[1] + 1).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
    }
    else if (settings.capture == false) {
      if (command[0] == 'P'){
        Serial.print(myPID.GetKp());
        Serial.print('|');
        Serial.print(myPID.GetKi());
        Serial.print('|');
        Serial.print(myPID.GetKd());
        Serial.print('\n');
      }
    }
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
