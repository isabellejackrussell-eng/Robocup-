#pragma once

#include "common/reading.h"

namespace sensors::capture {

struct CaptureSensorRaw {
  DistanceReading left;
  DistanceReading right;
  uint32_t timestampMs = 0;
};

bool initCaptureSensors();
void readCaptureLeft();
void readCaptureRight();
void readCaptureSensors();
const CaptureSensorRaw& getCaptureSensorRaw();

}  // namespace sensors::capture

