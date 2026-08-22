#include <Arduino.h>
#include <Servo.h>

enum PinAssignments {
  encoderPinA = 2,   // encoder feedback from the worm gear motor
  encoderPinB = 3,

  motorPinA = 4,     // PPM signal to Motor Drive Board channel A
  motorPinB = 5,     // PPM signal to Motor Drive Board channel B (if a 2nd motor is added later)
};

const int STOP_US = 1500;

Servo motorA;
Servo motorB;

volatile long encoderPos = 0;
long lastReportedPos = 1;

boolean A_set = false;
boolean B_set = false;

void doEncoderA();

void setup()
{
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);

  Serial.begin(9600);

  motorA.attach(motorPinA);
  motorB.attach(motorPinB);
  motorA.writeMicroseconds(STOP_US);   // start stopped, always
  motorB.writeMicroseconds(STOP_US);
  delay(1000);
}

void loop()
{
  if (lastReportedPos != encoderPos)
  {
    Serial.print("Index:");
    Serial.println(encoderPos);
    lastReportedPos = encoderPos;
  }

  motorA.writeMicroseconds(1700);   // gentle forward test on channel A
}

void doEncoderA(){
  A_set = digitalRead(encoderPinA) == HIGH;
  encoderPos += (A_set != B_set) ? +1 : -1;
  B_set = digitalRead(encoderPinB) == HIGH;
  encoderPos += (A_set == B_set) ? +1 : -1;
}