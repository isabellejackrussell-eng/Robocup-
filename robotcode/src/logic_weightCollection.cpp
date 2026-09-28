#include "logic_weightCollection.h"

#include "hd_move_servoArm.h"
#include "hd_raw_limitSwitch.h"
#include "logic_weightHunting.h"

namespace logic_weightCollection {
namespace {

State state = State::notInitialised;
bool initialised = false;
bool collectionCompleted = false;
uint32_t movementStartedAtMs = 0;

void setState(State nextState) {
  if (state == nextState) {
    return;
  }

  state = nextState;
  Serial.print("[WEIGHT COLLECTION] Arm state: ");
  Serial.println(stateName(state));
}

bool commandArm(uint8_t angleDegrees, State movementState, uint32_t nowMs) {
  if (!hd_move_servoArm::setAngle(angleDegrees)) {
    Serial.println("[WEIGHT COLLECTION] Servo-arm command failed");
    setState(State::fault);
    return false;
  }

  movementStartedAtMs = nowMs;
  setState(movementState);
  return true;
}

bool readLimitSwitch(hd_raw_limitSwitch::Reading& reading) {
  if (!hd_raw_limitSwitch::read(reading) || !reading.valid) {
    Serial.println("[WEIGHT COLLECTION] Limit-switch read failed");
    return false;
  }
  return true;
}

bool weightIsInCollectionZone() {
  return logic_weightHunting::isWeightInCollectionZone();
}

void startReturnToDown(uint32_t nowMs) {
  Serial.println(
      "[WEIGHT COLLECTION] Limit switch pressed; returning arm down");
  collectionCompleted = true;
  commandArm(kDownAngleDegrees, State::movingDown, nowMs);
}

}  // namespace

bool initialise() {
  initialised = false;
  collectionCompleted = false;
  state = State::notInitialised;

  if (!hd_move_servoArm::isInitialised()) {
    Serial.println("[WEIGHT COLLECTION] Servo arm is not initialised");
    return false;
  }
  if (!hd_raw_limitSwitch::isInitialised()) {
    Serial.println("[WEIGHT COLLECTION] Limit switch is not initialised");
    return false;
  }

  const uint8_t initialAngle = hd_move_servoArm::getCommandedAngle();
  if (initialAngle != kRequiredInitialAngleDegrees) {
    Serial.print("[WEIGHT COLLECTION] Expected arm startup angle 90, got ");
    Serial.println(initialAngle);
    setState(State::fault);
    return false;
  }

  Serial.println("[WEIGHT COLLECTION] Confirmed arm startup angle: 90 degrees");
  initialised = true;
  return commandArm(kDownAngleDegrees, State::movingDown, millis());
}

void update(uint32_t nowMs) {
  if (!initialised || state == State::fault) {
    return;
  }

  hd_raw_limitSwitch::Reading limitReading;

  switch (state) {
    case State::movingDown:
      if (nowMs - movementStartedAtMs < kArmMovementTimeMs) {
        return;
      }

      setState(State::down);
      if (!collectionCompleted) {
        return;
      }

      // Keep the robot stopped until the captured weight has moved clear of
      // the switch. Then re-arm weight hunting for the next weight.
      if (readLimitSwitch(limitReading) && !limitReading.pressed) {
        collectionCompleted = false;
        logic_weightHunting::reset();
        Serial.println("[WEIGHT COLLECTION] Capture complete; hunting re-armed");
      }
      return;

    case State::down:
      if (collectionCompleted) {
        if (readLimitSwitch(limitReading) && !limitReading.pressed) {
          collectionCompleted = false;
          logic_weightHunting::reset();
          Serial.println(
              "[WEIGHT COLLECTION] Capture complete; hunting re-armed");
        }
        return;
      }

      if (!weightIsInCollectionZone()) {
        return;
      }

      // Never begin a capture against an already-pressed switch.
      if (!readLimitSwitch(limitReading) || limitReading.pressed) {
        return;
      }

      Serial.println("[WEIGHT COLLECTION] Weight ready; raising arm");
      commandArm(kUpAngleDegrees, State::movingUp, nowMs);
      return;

    case State::movingUp:
      if (readLimitSwitch(limitReading) && limitReading.pressed) {
        startReturnToDown(nowMs);
        return;
      }

      if (nowMs - movementStartedAtMs >= kArmMovementTimeMs) {
        setState(State::up);
      }
      return;

    case State::up:
      if (readLimitSwitch(limitReading) && limitReading.pressed) {
        startReturnToDown(nowMs);
      }
      return;

    case State::notInitialised:
    case State::fault:
      return;
  }
}

State getState() {
  return state;
}

bool isDown() {
  return state == State::down;
}

bool isUp() {
  return state == State::up;
}

bool isBusy() {
  return initialised &&
      (state == State::movingDown ||
       state == State::movingUp ||
       state == State::up ||
       collectionCompleted);
}

const char* stateName(State value) {
  switch (value) {
    case State::notInitialised:
      return "NOT INITIALISED";
    case State::movingDown:
      return "MOVING DOWN";
    case State::down:
      return "DOWN";
    case State::movingUp:
      return "MOVING UP";
    case State::up:
      return "UP";
    case State::fault:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

void printStatus() {
  Serial.print("Weight collection: arm=");
  Serial.print(stateName(state));
  Serial.print(" commanded=");
  Serial.print(hd_move_servoArm::getCommandedAngle());
  Serial.print(" degrees, busy=");
  Serial.println(isBusy() ? "YES" : "NO");
}

}  // namespace logic_weightCollection
