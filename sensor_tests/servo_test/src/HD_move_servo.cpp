#include <Arduino.h>
#include <Servo.h>

Servo myServo;

const int servoPin = 25;
const int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);

  myServo.attach(servoPin);
  myServo.write(90);
  delay(1000);
}

void loop() {
  myServo.write(40);
  delay(1000);

  myServo.write(220);
  delay(1000);
}
