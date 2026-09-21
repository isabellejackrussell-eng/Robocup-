#include "perception/wall_detection.h"

#include <math.h>

#include "config/robot_config.h"

namespace perception::wall {
namespace {
WallState state;

bool usable(const DistanceReading& reading) {
  return reading.isFresh(millis(), config::SENSOR_STALE_TIMEOUT_MS) &&
         reading.distanceMm >= config::WALL_MIN_VALID_MM &&
         reading.distanceMm <= config::WALL_MAX_VALID_MM;
}

float filtered(float previous, float current, bool previousValid) {
  if (!previousValid || isnan(previous)) return current;
  return previous + config::WALL_FILTER_ALPHA * (current - previous);
}
}  // namespace

void filterWallReadings(const sensors::wall::WallSensorRaw& raw) {
  const bool left = usable(raw.ultrasonicLeft);
  const bool front = usable(raw.ultrasonicFront);
  const bool right = usable(raw.ultrasonicRight);
  if (left) state.wallLeftDistanceMm = filtered(state.wallLeftDistanceMm,
                                                raw.ultrasonicLeft.distanceMm, state.leftValid);
  if (front) state.wallFrontDistanceMm = filtered(state.wallFrontDistanceMm,
                                                   raw.ultrasonicFront.distanceMm, state.frontValid);
  if (right) state.wallRightDistanceMm = filtered(state.wallRightDistanceMm,
                                                   raw.ultrasonicRight.distanceMm, state.rightValid);
  state.leftValid = left;
  state.frontValid = front;
  state.rightValid = right;
  state.crossToFLeft = raw.crossToFLeft;
  state.crossToFRight = raw.crossToFRight;
}

void updateWallState(const sensors::wall::WallSensorRaw& raw) {
  filterWallReadings(raw);
  const bool crossLeftValid = usable(state.crossToFLeft);
  const bool crossRightValid = usable(state.crossToFRight);
  state.possibleLeftCorner = state.leftValid && crossLeftValid &&
      fabsf(state.wallLeftDistanceMm - state.crossToFLeft.distanceMm) >=
          config::WALL_CORNER_DIFFERENCE_MM;
  state.possibleRightCorner = state.rightValid && crossRightValid &&
      fabsf(state.wallRightDistanceMm - state.crossToFRight.distanceMm) >=
          config::WALL_CORNER_DIFFERENCE_MM;
  state.timestampMs = millis();
}

const WallState& getWallState() { return state; }

float getWallDistanceAtBearing(float bearingDeg) {
  if (bearingDeg < -20.0f) return state.leftValid ? state.wallLeftDistanceMm : NAN;
  if (bearingDeg > 20.0f) return state.rightValid ? state.wallRightDistanceMm : NAN;
  if (state.frontValid) return state.wallFrontDistanceMm;
  if (bearingDeg < 0.0f && state.crossToFLeft.isFresh(millis(), config::SENSOR_STALE_TIMEOUT_MS)) {
    return state.crossToFLeft.distanceMm;
  }
  if (bearingDeg >= 0.0f &&
      state.crossToFRight.isFresh(millis(), config::SENSOR_STALE_TIMEOUT_MS)) {
    return state.crossToFRight.distanceMm;
  }
  return NAN;
}

bool detectPossibleCorner() { return state.possibleLeftCorner || state.possibleRightCorner; }

}  // namespace perception::wall
