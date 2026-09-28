#include "hd_raw_encoder.h"

namespace hd_raw_encoder {
namespace {

volatile int32_t leftTicks = 0;
volatile int32_t rightTicks = 0;
volatile bool leftAState = false;
volatile bool leftBState = false;
volatile bool rightAState = false;
volatile bool rightBState = false;
bool initialised = false;

void handleLeftA() {
  leftAState = digitalRead(kLeftPinA) == HIGH;
  leftTicks += leftAState != leftBState ? 1 : -1;

  leftBState = digitalRead(kLeftPinB) == HIGH;
  leftTicks += leftAState == leftBState ? 1 : -1;
}

void handleRightA() {
  rightAState = digitalRead(kRightPinA) == HIGH;
  rightTicks += rightAState != rightBState ? 1 : -1;

  rightBState = digitalRead(kRightPinB) == HIGH;
  rightTicks += rightAState == rightBState ? 1 : -1;
}

}  // namespace

bool initialise() {
  pinMode(kLeftPinA, INPUT);
  pinMode(kLeftPinB, INPUT);
  pinMode(kRightPinA, INPUT);
  pinMode(kRightPinB, INPUT);

  leftAState = digitalRead(kLeftPinA) == HIGH;
  leftBState = digitalRead(kLeftPinB) == HIGH;
  rightAState = digitalRead(kRightPinA) == HIGH;
  rightBState = digitalRead(kRightPinB) == HIGH;

  reset();
  attachInterrupt(digitalPinToInterrupt(kLeftPinA), handleLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightPinA), handleRightA, CHANGE);
  initialised = true;
  return true;
}

bool read(Reading& reading) {
  reading = {0, 0, millis(), false};
  if (!initialised) {
    return false;
  }

  noInterrupts();
  reading.leftTicks = leftTicks;
  reading.rightTicks = rightTicks;
  interrupts();

  reading.timestampMs = millis();
  reading.valid = true;
  return true;
}

void reset() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

bool isInitialised() {
  return initialised;
}

}  // namespace hd_raw_encoder
