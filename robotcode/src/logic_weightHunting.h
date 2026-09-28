#ifndef LOGIC_WEIGHT_HUNTING_H
#define LOGIC_WEIGHT_HUNTING_H

#include <Arduino.h>

#include "filter_weightDetect.h"

namespace logic_weightHunting {

enum class State : uint8_t {
  notInitialised,
  stopped,
  huntingForWeight,
  turningRight,
  turningLeft,
  centringWeight,
  drivingForward,
  weightInCollectionZone,
};

struct Status {
  State state;
  float columnError;
  float pidOutput;
  int leftMotorPower;
  int rightMotorPower;
};

// Movement and PID tuning. If the camera image turns out to be mirrored,
// change kColumnToTurnSign from 1.0f to -1.0f.
constexpr float kTargetColumn = 3.5f;
constexpr float kCentredToleranceColumns = 0.5f;
constexpr uint16_t kCollectionZoneDistanceMm = 60;
constexpr float kColumnToTurnSign = 1.0f;

constexpr float kProportionalGain = 48.0f;
constexpr float kIntegralGain = 2.0f;
constexpr float kDerivativeGain = 7.0f;

constexpr int kSideSensorTurnPower = 120;
constexpr int kMinimumPidTurnPower = 65;
constexpr int kForwardPower = 140;
constexpr int kMaximumForwardCorrection = 70;
constexpr int kMaximumMotorPower = 250;

// With no detected weight, both motors remain forward while this correction
// smoothly moves from side to side. One complete sweep takes four seconds.
constexpr int kHuntForwardPower = 105;
constexpr int kHuntSteeringCorrection = 40;
constexpr uint32_t kHuntSweepPeriodMs = 4000;

// Requires the drive motors to have already been initialised.
bool initialise();

// Consumes one fused weight result, drives the two motors, and returns the
// current hunting state. The collection-zone state latches until reset().
State update(
    const filter_weightDetect::WeightIdentification& weight,
    uint32_t nowMs = millis());

// Stops movement and re-arms the logic for the next weight.
void reset();
void stop();

State getState();
bool isWeightInCollectionZone();
const Status& getStatus();
const char* stateName(State state);
void printStatus();

}  // namespace logic_weightHunting

#endif
