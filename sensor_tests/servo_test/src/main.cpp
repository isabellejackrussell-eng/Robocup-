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
  digitalWrite(ledPin, HIGH);
  myServo.write(60);
  delay(1000);
  digitalWrite(ledPin, LOW);

  digitalWrite(ledPin, HIGH);
  myServo.write(120);
  delay(1000);
  digitalWrite(ledPin, LOW);
}
