#ifndef LOGIC_WEIGHT_COLLECTION_H
#define LOGIC_WEIGHT_COLLECTION_H

#include <Arduino.h>

namespace logic_weightCollection {

enum class State : uint8_t {
  notInitialised,
  movingDown,
  down,
  movingUp,
  up,
  fault,
};

constexpr uint8_t kRequiredInitialAngleDegrees = 90;
constexpr uint8_t kDownAngleDegrees = 40;
constexpr uint8_t kUpAngleDegrees = 160;
constexpr uint32_t kArmMovementTimeMs = 1500;

// Requires the servo arm to have been initialised at exactly 90 degrees and
// the limit switch to be available. The arm is then put in its down position.
bool initialise();

// Reads the weight-hunting collection-zone state and the limit switch, then
// advances the capture sequence without blocking the main loop.
void update(uint32_t nowMs = millis());

State getState();
bool isDown();
bool isUp();
bool isBusy();
const char* stateName(State state);
void printStatus();

}  // namespace logic_weightCollection

#endif
