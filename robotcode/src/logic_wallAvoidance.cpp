#include "logic_wallAvoidance.h"

#include <math.h>

#include "hd_move_motors.h"
#include "hd_raw_imu.h"

namespace logic_wallAvoidance {
namespace {

bool initialised = false;
uint32_t turnStartedAtMs = 0;
uint32_t lastTurnUpdateMs = 0;

Status status = {
    false,
    false,
    false,
    Turn::none,
    0.0f,
    0.0f,
    0.0f,
};

bool sensorInsideThreshold(
    const hd_raw_tof::Readings& readings,
    uint8_t sensorIndex) {
  if (sensorIndex >= hd_raw_tof::kSensorCount) {
    return false;
  }

  const hd_raw_tof::Reading& reading = readings[sensorIndex];
  return reading.valid &&
      reading.distanceMm > 0 &&
      reading.distanceMm < kWallThresholdMm;
}

float angularSeparationDegrees(float first, float second) {
  float difference = fmodf(fabsf(first - second), 360.0f);
  if (difference > 180.0f) {
    difference = 360.0f - difference;
  }
  return difference;
}

bool readHeading(float& headingDegrees) {
  hd_raw_imu::Reading imu;
  if (!hd_raw_imu::read(imu) || !imu.valid) {
    return false;
  }
  headingDegrees = imu.headingDegrees;
  return true;
}

void commandTurn(Turn turn) {
  switch (turn) {
    case Turn::right20:
    case Turn::around180:
      motors_write(kTurnMotorPower, -kTurnMotorPower);
      return;

    case Turn::none:
    default:
      motors_stop();
      return;
  }
}

bool beginTurn(Turn turn, float targetDegrees, uint32_t nowMs) {
  float headingDegrees = 0.0f;
  if (!readHeading(headingDegrees)) {
    Serial.println("[WALL AVOID] IMU heading unavailable; stopping");
    motors_stop();
    return false;
  }

  status.turning = true;
  status.turn = turn;
  status.startHeadingDegrees = headingDegrees;
  status.turnedDegrees = 0.0f;
  status.targetDegrees = targetDegrees;
  turnStartedAtMs = nowMs;
  lastTurnUpdateMs = nowMs;

  Serial.print("[WALL AVOID] ");
  Serial.print(turnName(turn));
  Serial.print(" from heading ");
  Serial.print(headingDegrees, 1);
  Serial.print(" deg; target turn=");
  Serial.print(targetDegrees, 0);
  Serial.println(" deg");

  commandTurn(turn);
  return true;
}

bool continueTurn(uint32_t nowMs) {
  if (!status.turning) {
    return false;
  }

  if (nowMs - turnStartedAtMs >= kTurnTimeoutMs) {
    Serial.println("[WALL AVOID] Turn timeout; stopping motors");
    motors_stop();
    status.turning = false;
    status.turn = Turn::none;
    return true;
  }

  if (nowMs - lastTurnUpdateMs < kTurnUpdatePeriodMs) {
    commandTurn(status.turn);
    return true;
  }
  lastTurnUpdateMs = nowMs;

  float headingDegrees = 0.0f;
  if (!readHeading(headingDegrees)) {
    Serial.println("[WALL AVOID] Lost IMU heading during turn; stopping");
    motors_stop();
    return true;
  }

  status.turnedDegrees = angularSeparationDegrees(
      headingDegrees,
      status.startHeadingDegrees);

  if (status.turnedDegrees + kTurnToleranceDegrees >= status.targetDegrees) {
    motors_stop();
    Serial.print("[WALL AVOID] Turn complete: ");
    Serial.print(status.turnedDegrees, 1);
    Serial.print(" / ");
    Serial.print(status.targetDegrees, 0);
    Serial.println(" deg");
    status.turning = false;
    status.turn = Turn::none;
    // Keep ownership of the motors for this cycle so the hunting layer cannot
    // immediately overwrite the stop command on the exact turn-completion tick.
    return true;
  }

  commandTurn(status.turn);
  return true;
}

}  // namespace

bool initialise() {
  status = {};
  status.turn = Turn::none;

  initialised =
      motors_is_initialised() &&
      hd_raw_imu::isInitialised() &&
      hd_raw_tof::isSensorInitialised(kTopRightSensorIndex) &&
      hd_raw_tof::isSensorInitialised(kTopLeftSensorIndex);

  if (!initialised) {
    Serial.println(
        "[WALL AVOID] Not ready: requires motors, IMU, top-right ToF 1 and top-left ToF 3");
    return false;
  }

  Serial.println(
      "[WALL AVOID] Ready: top-right=ToF1, top-left=ToF3, threshold=200 mm");
  return true;
}

bool update(const hd_raw_tof::Readings& readings, uint32_t nowMs) {
  status.topRightClose = sensorInsideThreshold(
      readings,
      kTopRightSensorIndex);
  status.topLeftClose = sensorInsideThreshold(
      readings,
      kTopLeftSensorIndex);

  if (!initialised) {
    return false;
  }

  if (status.turning) {
    return continueTurn(nowMs);
  }

  if (!status.topLeftClose && !status.topRightClose) {
    return false;
  }

  if (status.topLeftClose && status.topRightClose) {
    // The requested behaviour does not specify a direction for the 180 turn;
    // use a deterministic right-hand point turn.
    return beginTurn(Turn::around180, kBothSensorsTurnDegrees, nowMs);
  }

  if (status.topLeftClose) {
    return beginTurn(Turn::right20, kSingleSensorTurnDegrees, nowMs);
  }

  return beginTurn(Turn::right20, kSingleSensorTurnDegrees, nowMs);
}

bool clawInhibited() {
  return status.topLeftClose || status.topRightClose || status.turning;
}

bool wallDetected() {
  return status.topLeftClose || status.topRightClose;
}

bool isTurning() {
  return status.turning;
}

const Status& getStatus() {
  return status;
}

const char* turnName(Turn turn) {
  switch (turn) {
    case Turn::right20:
      return "ONE TOP SENSOR <200 mm -> RIGHT 20";
    case Turn::around180:
      return "BOTH TOP <200 mm -> RIGHT 180";
    case Turn::none:
    default:
      return "NONE";
  }
}

}  // namespace logic_wallAvoidance
