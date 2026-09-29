#include "logic_weightHunting.h"

#include <math.h>

#include "hd_move_motors.h"

namespace logic_weightHunting {
namespace {

constexpr float kMaximumIntegral = 2.0f;
constexpr float kMinimumPidPeriodSeconds = 0.02f;
constexpr float kMaximumPidPeriodSeconds = 0.5f;

bool initialised = false;
bool havePreviousError = false;
float integral = 0.0f;
float previousError = 0.0f;
uint32_t previousPidMs = 0;

Status status = {
    State::notInitialised,
    0.0f,
    0.0f,
    0,
    0,
};

void clearPid() {
  havePreviousError = false;
  integral = 0.0f;
  previousError = 0.0f;
  previousPidMs = 0;
  status.columnError = 0.0f;
  status.pidOutput = 0.0f;
}

void commandMotors(int leftPower, int rightPower) {
  leftPower = constrain(
      leftPower, -kMaximumMotorPower, kMaximumMotorPower);
  rightPower = constrain(
      rightPower, -kMaximumMotorPower, kMaximumMotorPower);

  status.leftMotorPower = leftPower;
  status.rightMotorPower = rightPower;
  motors_write(leftPower, rightPower);
}

void setState(State nextState) {
  if (status.state == nextState) {
    return;
  }

  status.state = nextState;
  Serial.print("[WEIGHT HUNTING] ");
  Serial.println(stateName(nextState));
}

float calculatePid(float error, uint32_t nowMs) {
  float periodSeconds = 0.1f;
  float derivative = 0.0f;

  if (havePreviousError) {
    periodSeconds = (nowMs - previousPidMs) / 1000.0f;
    periodSeconds = constrain(
        periodSeconds,
        kMinimumPidPeriodSeconds,
        kMaximumPidPeriodSeconds);
    derivative = (error - previousError) / periodSeconds;
  }

  integral += error * periodSeconds;
  integral = constrain(integral, -kMaximumIntegral, kMaximumIntegral);

  previousError = error;
  previousPidMs = nowMs;
  havePreviousError = true;

  const float output =
      kProportionalGain * error +
      kIntegralGain * integral +
      kDerivativeGain * derivative;
  return constrain(
      output,
      -static_cast<float>(kMaximumMotorPower),
      static_cast<float>(kMaximumMotorPower));
}

int centringTurnPower(float pidOutput) {
  int power = static_cast<int>(roundf(pidOutput * kColumnToTurnSign));
  if (power > 0 && power < kMinimumPidTurnPower) {
    power = kMinimumPidTurnPower;
  } else if (power < 0 && power > -kMinimumPidTurnPower) {
    power = -kMinimumPidTurnPower;
  }
  return power;
}

int huntSteeringCorrection(uint32_t nowMs) {
  const uint32_t halfPeriodMs = kHuntSweepPeriodMs / 2;
  const uint32_t phaseMs = nowMs % kHuntSweepPeriodMs;

  // Triangle wave: -maximum -> +maximum -> -maximum. Unlike a sudden timed
  // left/right switch, this produces a gentle sweeping path.
  float sweep = 0.0f;
  if (phaseMs < halfPeriodMs) {
    sweep = -1.0f + 2.0f * phaseMs / halfPeriodMs;
  } else {
    sweep = 3.0f - 2.0f * phaseMs / halfPeriodMs;
  }

  return static_cast<int>(roundf(sweep * kHuntSteeringCorrection));
}

void driveHuntingPattern(uint32_t nowMs) {
  const int correction = huntSteeringCorrection(nowMs);
  setState(State::huntingForWeight);
  commandMotors(
      kHuntForwardPower + correction,
      kHuntForwardPower - correction);
}

void turnTowardsSideSensor(
    const filter_weightDetect::WeightIdentification& weight) {
  bool turnRight = weight.seenByRightTof;

  if (weight.seenByRightTof && weight.seenByLeftTof) {
    // When both side sensors see a weight, face the closer reading first.
    turnRight = weight.rightDistanceMm <= weight.leftDistanceMm;
  }

  if (turnRight) {
    setState(State::turningRight);
    commandMotors(kSideSensorTurnPower, -kSideSensorTurnPower);
  } else {
    setState(State::turningLeft);
    commandMotors(-kSideSensorTurnPower, kSideSensorTurnPower);
  }
}

}  // namespace

bool initialise() {
  initialised = motors_is_initialised();
  clearPid();
  commandMotors(0, 0);
  status.state = initialised
      ? State::huntingForWeight
      : State::notInitialised;
  return initialised;
}

State update(
    const filter_weightDetect::WeightIdentification& weight,
    uint32_t nowMs) {
  if (!initialised) {
    status.state = State::notInitialised;
    return status.state;
  }

  // This is deliberately latched. The collection mechanism can query the
  // state and call reset() after it has dealt with the weight.
  if (status.state == State::weightInCollectionZone) {
    commandMotors(0, 0);
    return status.state;
  }

  if (!filter_weightDetect::weightFound(weight)) {
    clearPid();
    driveHuntingPattern(nowMs);
    return status.state;
  }

  if (!weight.seenBy8x8) {
    clearPid();
    turnTowardsSideSensor(weight);
    return status.state;
  }

  status.columnError = weight.column - kTargetColumn;
  status.pidOutput = calculatePid(status.columnError, nowMs);

  const bool centred =
      fabsf(status.columnError) <= kCentredToleranceColumns;
  if (centred &&
      weight.centreDistanceMm > 0 &&
      weight.centreDistanceMm <= kCollectionZoneDistanceMm) {
    setState(State::weightInCollectionZone);
    commandMotors(0, 0);
    return status.state;
  }

  if (!centred) {
    const int turnPower = centringTurnPower(status.pidOutput);
    setState(State::centringWeight);
    commandMotors(turnPower, -turnPower);
    return status.state;
  }

  const int steeringCorrection = constrain(
      static_cast<int>(roundf(status.pidOutput * kColumnToTurnSign)),
      -kMaximumForwardCorrection,
      kMaximumForwardCorrection);
  setState(State::drivingForward);
  commandMotors(
      kForwardPower + steeringCorrection,
      kForwardPower - steeringCorrection);
  return status.state;
}

void reset() {
  clearPid();
  if (initialised) {
    setState(State::huntingForWeight);
  }
  commandMotors(0, 0);
}

void stop() {
  clearPid();
  setState(State::stopped);
  commandMotors(0, 0);
}

State getState() {
  return status.state;
}

bool isWeightInCollectionZone() {
  return status.state == State::weightInCollectionZone;
}

const Status& getStatus() {
  return status;
}

const char* stateName(State state) {
  switch (state) {
    case State::notInitialised:
      return "NOT INITIALISED";
    case State::stopped:
      return "STOPPED";
    case State::huntingForWeight:
      return "HUNTING FOR WEIGHT";
    case State::turningRight:
      return "TURNING RIGHT";
    case State::turningLeft:
      return "TURNING LEFT";
    case State::centringWeight:
      return "CENTRING WEIGHT";
    case State::drivingForward:
      return "DRIVING FORWARD";
    case State::weightInCollectionZone:
      return "WEIGHT IN COLLECTION ZONE";
    default:
      return "UNKNOWN";
  }
}

void printStatus() {
  Serial.print("Weight hunting: ");
  Serial.print(stateName(status.state));
  Serial.print(" | error=");
  Serial.print(status.columnError, 2);
  Serial.print(" PID=");
  Serial.print(status.pidOutput, 1);
  Serial.print(" motors=");
  Serial.print(status.leftMotorPower);
  Serial.print(',');
  Serial.println(status.rightMotorPower);
}

void printTestDecision(
    const filter_weightDetect::WeightIdentification& weight) {
  Serial.print("[WEIGHT LOGIC TEST] ");

  if (!filter_weightDetect::weightFound(weight)) {
    Serial.println("no weight -> hunt forward in snake pattern");
    return;
  }

  if (!weight.seenBy8x8) {
    if (weight.seenByRightTof && weight.seenByLeftTof) {
      Serial.println(
          weight.rightDistanceMm <= weight.leftDistanceMm
              ? "weight on both sides -> turn right"
              : "weight on both sides -> turn left");
    } else {
      Serial.println(
          weight.seenByRightTof
              ? "weight on right -> turn right"
              : "weight on left -> turn left");
    }
    return;
  }

  const float error = weight.column - kTargetColumn;
  if (fabsf(error) > kCentredToleranceColumns) {
    Serial.println(error > 0.0f
        ? "8x8 weight right of centre -> turn right"
        : "8x8 weight left of centre -> turn left");
  } else if (weight.centreDistanceMm <= kCollectionZoneDistanceMm) {
    Serial.println("weight in collection zone");
  } else {
    Serial.println("weight centred -> drive forward");
  }
}

}  // namespace logic_weightHunting
