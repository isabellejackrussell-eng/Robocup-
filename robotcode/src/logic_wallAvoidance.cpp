#include "logic_wallAvoidance.h"

#include <math.h>

#include "hd_move_motors.h"
#include "hd_raw_imu.h"

namespace logic_wallAvoidance {
namespace {

bool initialised = false;
uint32_t turnStartedAtMs = 0;
uint32_t lastTurnUpdateMs = 0;
uint32_t reverseStartedAtMs = 0;
uint16_t previousWallDistanceMm = 0;
bool havePreviousWallDistance = false;
Turn turnAfterReverse = Turn::none;

Status status = {};

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

bool nearestWallDistance(
    const hd_raw_tof::Readings& readings,
    uint16_t& distanceMm) {
  bool found = false;
  const uint8_t sensorIndices[] = {
      kTopRightSensorIndex,
      kTopLeftSensorIndex,
  };
  for (const uint8_t sensorIndex : sensorIndices) {
    const hd_raw_tof::Reading& reading = readings[sensorIndex];
    if (!reading.valid || reading.distanceMm == 0) {
      continue;
    }
    if (!found || reading.distanceMm < distanceMm) {
      distanceMm = reading.distanceMm;
      found = true;
    }
  }
  return found;
}

Turn oppositeTurn(Turn turn) {
  return turn == Turn::left ? Turn::right : Turn::left;
}

Turn chooseInitialTurn(const hd_raw_tof::Readings& readings) {
  if (status.topLeftClose && !status.topRightClose) {
    return Turn::right;
  }
  if (status.topRightClose && !status.topLeftClose) {
    return Turn::left;
  }

  // When both sensors see the wall, turn toward the side with more clearance.
  return readings[kTopLeftSensorIndex].distanceMm >
          readings[kTopRightSensorIndex].distanceMm
      ? Turn::left
      : Turn::right;
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
    case Turn::right:
      motors_write(kTurnMotorPower, -kTurnMotorPower);
      return;

    case Turn::left:
      motors_write(-kTurnMotorPower, kTurnMotorPower);
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
  status.reversing = false;
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

void beginReverse(Turn failedTurn, uint32_t nowMs) {
  status.turning = false;
  status.reversing = true;
  status.turn = Turn::none;
  turnAfterReverse = oppositeTurn(failedTurn);
  reverseStartedAtMs = nowMs;
  havePreviousWallDistance = false;

  Serial.print("[WALL AVOID] Wall getting closer; reversing before turning ");
  Serial.println(turnAfterReverse == Turn::left ? "LEFT" : "RIGHT");
  motors_write(-kReverseMotorPower, -kReverseMotorPower);
}

bool continueReverse(uint32_t nowMs) {
  if (nowMs - reverseStartedAtMs < kReverseDurationMs) {
    motors_write(-kReverseMotorPower, -kReverseMotorPower);
    return true;
  }

  status.reversing = false;
  const Turn recoveryTurn = turnAfterReverse;
  turnAfterReverse = Turn::none;
  if (beginTurn(recoveryTurn, kSingleSensorTurnDegrees, nowMs)) {
    return true;
  }

  motors_stop();
  return true;
}

enum class TurnResult : uint8_t {
  running,
  complete,
  failed,
};

TurnResult continueTurn(uint32_t nowMs) {
  if (!status.turning) {
    return TurnResult::failed;
  }

  if (nowMs - turnStartedAtMs >= kTurnTimeoutMs) {
    Serial.println("[WALL AVOID] Turn timeout; stopping motors");
    motors_stop();
    status.turning = false;
    status.turn = Turn::none;
    return TurnResult::failed;
  }

  if (nowMs - lastTurnUpdateMs < kTurnUpdatePeriodMs) {
    commandTurn(status.turn);
    return TurnResult::running;
  }
  lastTurnUpdateMs = nowMs;

  float headingDegrees = 0.0f;
  if (!readHeading(headingDegrees)) {
    Serial.println("[WALL AVOID] Lost IMU heading during turn; stopping");
    motors_stop();
    status.turning = false;
    status.turn = Turn::none;
    return TurnResult::failed;
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
    return TurnResult::complete;
  }

  commandTurn(status.turn);
  return TurnResult::running;
}

}  // namespace

bool initialise() {
  status = {};
  status.turn = Turn::none;
  turnAfterReverse = Turn::none;
  havePreviousWallDistance = false;

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

bool update(
    const hd_raw_tof::Readings& readings,
    uint32_t nowMs,
    bool readingsFresh) {
  status.topRightClose = sensorInsideThreshold(
      readings,
      kTopRightSensorIndex);
  status.topLeftClose = sensorInsideThreshold(
      readings,
      kTopLeftSensorIndex);

  if (!initialised) {
    return false;
  }

  if (status.reversing) {
    return continueReverse(nowMs);
  }

  if (status.turning) {
    const Turn currentTurn = status.turn;

    if (readingsFresh) {
      if (!status.topLeftClose && !status.topRightClose) {
        Serial.println("[WALL AVOID] Wall clear; resuming normal motion");
        motors_stop();
        status.turning = false;
        status.turn = Turn::none;
        havePreviousWallDistance = false;
        return true;
      }

      uint16_t wallDistanceMm = 0;
      if (nearestWallDistance(readings, wallDistanceMm)) {
        if (havePreviousWallDistance &&
            wallDistanceMm + kGettingCloserMarginMm <
                previousWallDistanceMm) {
          beginReverse(currentTurn, nowMs);
          return true;
        }
        previousWallDistanceMm = wallDistanceMm;
        havePreviousWallDistance = true;
      }
    }

    const TurnResult turnResult = continueTurn(nowMs);
    if (turnResult == TurnResult::complete &&
        (status.topLeftClose || status.topRightClose)) {
      // A 20-degree segment completed but the wall is still present. Continue
      // in the same direction instead of handing control to the stop layer.
      beginTurn(currentTurn, kSingleSensorTurnDegrees, nowMs);
    }
    return true;
  }

  if (!status.topLeftClose && !status.topRightClose) {
    return false;
  }

  uint16_t wallDistanceMm = 0;
  havePreviousWallDistance = nearestWallDistance(readings, wallDistanceMm);
  previousWallDistanceMm = wallDistanceMm;

  const Turn turn = chooseInitialTurn(readings);
  const float targetDegrees = status.topLeftClose && status.topRightClose
      ? kBothSensorsTurnDegrees
      : kSingleSensorTurnDegrees;
  return beginTurn(turn, targetDegrees, nowMs);
}

bool clawInhibited() {
  return status.topLeftClose || status.topRightClose || isActive();
}

bool wallDetected() {
  return status.topLeftClose || status.topRightClose;
}

bool isTurning() {
  return status.turning;
}

bool isActive() {
  return status.turning || status.reversing;
}

const Status& getStatus() {
  return status;
}

const char* turnName(Turn turn) {
  switch (turn) {
    case Turn::left:
      return "LEFT";
    case Turn::right:
      return "RIGHT";
    case Turn::none:
    default:
      return "NONE";
  }
}

}  // namespace logic_wallAvoidance
