#pragma once

#include <Arduino.h>

namespace logic_weightHandling {

// Change this if the sorting mechanism is connected to smart-servo ID 4.
constexpr uint8_t kSortingServoId = 1;

constexpr float kClockwiseMovementDegrees = 90.0f;
constexpr float kAnticlockwiseMovementDegrees = -90.0f;
constexpr uint32_t kMeasurementDelayMs = 3000;
constexpr uint32_t kMovementDurationMs = 3000;

// Call after the limit switch and smart servos have been initialised.
// Returns false if either required hardware module is unavailable.
bool initialise();

// Call regularly from loop(). This reads the limit switch and inductive sensor
// through their hardware-driver APIs and advances the sorting sequence.
void update();

// True while a measurement is waiting, the sorter is moving, or the logic is
// waiting for the current switch press to be released.
bool isBusy();

}  // namespace logic_weightHandling
