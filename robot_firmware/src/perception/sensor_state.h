#pragma once

#include <Arduino.h>

namespace perception {

struct SensorState {
  bool weightCandidate = false;
  bool confirmedWeight = false;
  float weightDistanceMm = NAN;
  float weightBearingDeg = NAN;
  float wallLeftDistanceMm = NAN;
  float wallFrontDistanceMm = NAN;
  float wallRightDistanceMm = NAN;
  bool wallLeftValid = false;
  bool wallFrontValid = false;
  bool wallRightValid = false;
  bool possibleLeftCorner = false;
  bool possibleRightCorner = false;
  bool weightInCaptureZone = false;
  bool sensorsHealthy = false;
  uint32_t timestampMs = 0;
};

void updateSensorState();
const SensorState& getSensorState();

}  // namespace perception

