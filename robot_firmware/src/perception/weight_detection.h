#pragma once

#include <Arduino.h>

#include "sensors/tof8x8.h"

namespace perception::weight {

struct WeightData {
  bool found = false;
  float bearingDeg = 0.0f;
  float distanceMm = 0.0f;
  float centroidColumn = 0.0f;
  float centroidRow = 0.0f;
  uint8_t clusterSize = 0;
  float confidence = 0.0f;
  uint32_t timestampMs = 0;
};

void calibrateBackground(const sensors::tof8x8::Frame& frame);
bool backgroundCalibrated();
WeightData process8x8Frame(const sensors::tof8x8::Frame& frame);
bool weightCandidateDetected();
float getWeightDistance();
float getWeightBearing();
float getWeightConfidence();
const WeightData& getWeightData();

}  // namespace perception::weight

