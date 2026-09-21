#pragma once

#include "sensors/wall_sensors.h"

namespace perception::wall {

struct WallState {
  bool leftValid = false;
  bool frontValid = false;
  bool rightValid = false;
  float wallLeftDistanceMm = NAN;
  float wallFrontDistanceMm = NAN;
  float wallRightDistanceMm = NAN;
  bool possibleLeftCorner = false;
  bool possibleRightCorner = false;
  DistanceReading crossToFLeft;
  DistanceReading crossToFRight;
  uint32_t timestampMs = 0;
};

void filterWallReadings(const sensors::wall::WallSensorRaw& raw);
void updateWallState(const sensors::wall::WallSensorRaw& raw);
const WallState& getWallState();
float getWallDistanceAtBearing(float bearingDeg);
bool detectPossibleCorner();

}  // namespace perception::wall

