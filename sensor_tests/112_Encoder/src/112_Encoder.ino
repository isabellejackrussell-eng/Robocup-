#include <Arduino.h>
#include <Servo.h>

Servo myservoA, myservoB;

enum PinAssignments {
  encoder1PinA = 2,
  encoder1PinB = 3,

  encoder2PinA = 4,
  encoder2PinB = 5,
};

volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;

boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;

void setup()
{
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);

  myservoA.attach(0);
  myservoB.attach(1);

  Serial.begin(115200);
}

// Prints both encoder values every 100 ms for the given time
void waitAndPrint(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms) {
    noInterrupts();
    long p1 = encoderPos1;
    long p2 = encoderPos2;
    interrupts();

    Serial.print("Encoder1: ");
    Serial.print(p1);
    Serial.print("\tEncoder2: ");
    Serial.println(p2);
    delay(100);
  }
}

void loop()
{
  Serial.println("--- Backward ---");
  myservoA.writeMicroseconds(1050);
  myservoB.writeMicroseconds(1050);
  waitAndPrint(1500);

  Serial.println("--- Forward ---");
  myservoA.writeMicroseconds(1950);
  myservoB.writeMicroseconds(1950);
  waitAndPrint(1500);

  Serial.println("--- Stop ---");
  myservoA.writeMicroseconds(1500);
  myservoB.writeMicroseconds(1500);
  waitAndPrint(1000);
}

// Interrupt on A changing state
void doEncoder1A() {
  A_set1 = digitalRead(encoder1PinA) == HIGH;
  encoderPos1 += (A_set1 != B_set1) ? +1 : -1;

  B_set1 = digitalRead(encoder1PinB) == HIGH;
  encoderPos1 += (A_set1 == B_set1) ? +1 : -1;
}

void doEncoder2A() {
  A_set2 = digitalRead(encoder2PinA) == HIGH;
  encoderPos2 += (A_set2 != B_set2) ? +1 : -1;

  B_set2 = digitalRead(encoder2PinB) == HIGH;
  encoderPos2 += (A_set2 == B_set2) ? +1 : -1;
}