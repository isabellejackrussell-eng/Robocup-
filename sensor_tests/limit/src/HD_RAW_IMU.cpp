#include <Arduino.h>

const int trigPin = 3;
const int echoPin = 2;

void setup()
{
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, LOW);

  Serial.println("Testing Ultrasound A on DIGITAL RAW2");
}

void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  Serial.print("Pulse = ");
  Serial.print(duration);
  Serial.print(" us");

  if (duration > 0) {
    Serial.print("    Distance = ");
    Serial.print(duration / 58.0);
    Serial.println(" cm");
  } else {
    Serial.println("    NO ECHO");
  }

  delay(500);
}