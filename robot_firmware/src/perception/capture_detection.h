#pragma once

#include "sensors/capture_sensors.h"

namespace perception::capture {

struct CaptureState {
  bool leftTriggered = false;
  bool rightTriggered = false;
  bool weightPresent = false;
  float confidence = 0.0f;
  float filteredLeftMm = NAN;
  float filteredRightMm = NAN;
  uint32_t timestampMs = 0;
};

void filterCaptureReadings(const sensors::capture::CaptureSensorRaw& raw);
bool weightInCaptureZone();
float captureConfidence();
const CaptureState& getCaptureState();

}  // namespace perception::capture

