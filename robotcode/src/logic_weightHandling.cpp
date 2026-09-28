#include "logic_weightHandling.h"

#include <math.h>

#include "hd_move_smartServos.h"
#include "hd_raw_inductiveProximity.h"
#include "hd_raw_limitSwitch.h"

namespace logic_weightHandling {
namespace {

enum class State {
  waitingForPress,
  waitingBeforeMove,
  waitingBeforeReturn,
  waitingForRelease,
};

State state = State::waitingForPress;
bool initialised = false;
bool measuredMetal = false;
float returnAngleDegrees = 0.0f;
uint32_t stateStartedAtMs = 0;

bool readLimitSwitch(hd_raw_limitSwitch::Reading& reading) {
  if (!hd_raw_limitSwitch::read(reading) || !reading.valid) {
    Serial.println("[WEIGHT HANDLING] Limit-switch read failed");
    return false;
  }

  return true;
}

void waitForRelease() {
  state = State::waitingForRelease;
  stateStartedAtMs = millis();
}

}  // namespace

bool initialise() {
  state = State::waitingForPress;
  stateStartedAtMs = millis();

  const bool limitSwitchReady = hd_raw_limitSwitch::isInitialised();
  const bool sortingServoReady = smartServoIsReady(kSortingServoId);
  initialised = limitSwitchReady && sortingServoReady;

  if (!limitSwitchReady) {
    Serial.println("[WEIGHT HANDLING] Limit switch is not initialised");
  }
  if (!sortingServoReady) {
    Serial.print("[WEIGHT HANDLING] Smart servo is not ready: ID ");
    Serial.println(kSortingServoId);
  }

  return initialised;
}

void update() {
  if (!initialised) {
    return;
  }

  const uint32_t now = millis();
  hd_raw_limitSwitch::Reading limitReading;

  switch (state) {
    case State::waitingForPress:
      if (!readLimitSwitch(limitReading) || !limitReading.pressed) {
        return;
      }

      // Classify once at the start of the press. A detection means metal;
      // otherwise the pressed object is treated as plastic.
      measuredMetal = inductiveSensorDetected();
      returnAngleDegrees = getServoAngle(kSortingServoId);

      if (isnan(returnAngleDegrees)) {
        Serial.println("[WEIGHT HANDLING] Could not read sorter angle");
        waitForRelease();
        return;
      }

      Serial.print("[WEIGHT HANDLING] Measured ");
      Serial.print(measuredMetal ? "METAL" : "PLASTIC");
      Serial.print(" (inductive raw=");
      Serial.print(inductiveSensorRaw());
      Serial.print(')');
      Serial.print("; waiting ");
      Serial.print(kMeasurementDelayMs);
      Serial.println(" ms before sorting");
      state = State::waitingBeforeMove;
      stateStartedAtMs = now;
      return;

    case State::waitingBeforeMove: {
      // The object must continue holding the switch throughout the delay.
      if (!readLimitSwitch(limitReading) || !limitReading.pressed) {
        Serial.println("[WEIGHT HANDLING] Sort cancelled: switch released");
        state = State::waitingForPress;
        return;
      }

      if (now - stateStartedAtMs < kMeasurementDelayMs) {
        return;
      }

      // Keep the two signed commands explicit so metal and plastic can never
      // accidentally be sent the same directional offset.
      const float movementDegrees =
          measuredMetal ? kClockwiseMovementDegrees
                        : kAnticlockwiseMovementDegrees;
      const float targetAngle = returnAngleDegrees + movementDegrees;

      Serial.print("[WEIGHT HANDLING] Sort command: start=");
      Serial.print(returnAngleDegrees);
      Serial.print(" degrees, offset=");
      Serial.print(movementDegrees);
      Serial.print(" degrees, target=");
      Serial.print(targetAngle);
      Serial.println(" degrees");

      if (!setServoAngle(kSortingServoId, targetAngle)) {
        Serial.println("[WEIGHT HANDLING] Sorter movement failed");
        waitForRelease();
        return;
      }

      state = State::waitingBeforeReturn;
      stateStartedAtMs = now;
      return;
    }

    case State::waitingBeforeReturn:
      if (now - stateStartedAtMs < kMovementDurationMs) {
        return;
      }

      Serial.println("[WEIGHT HANDLING] Returning sorter to start angle");
      if (!setServoAngle(kSortingServoId, returnAngleDegrees)) {
        Serial.println("[WEIGHT HANDLING] Sorter return movement failed");
      }
      waitForRelease();
      return;

    case State::waitingForRelease:
      if (readLimitSwitch(limitReading) && !limitReading.pressed) {
        state = State::waitingForPress;
      }
      return;
  }
}

bool isBusy() {
  return initialised && state != State::waitingForPress;
}

}  // namespace logic_weightHandling
