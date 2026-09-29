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

  // Slowly move from 95° to 120°
  for (int angle = 40; angle <= 190; angle = angle+5) {
    myServo.write(angle);
    delay(50);
  }

  delay(500);

  // Slowly move back from 120° to 95°
  for (int angle = 190; angle >= 40; angle = angle- 5) {
    myServo.write(angle);
    delay(50);
  }

  delay(1000);
}