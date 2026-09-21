#pragma once

#include <Arduino.h>
#include <math.h>

struct DistanceReading {
  float distanceMm = NAN;
  bool valid = false;
  uint32_t timestampMs = 0;

  bool isFresh(uint32_t nowMs, uint32_t timeoutMs) const {
    return valid && static_cast<uint32_t>(nowMs - timestampMs) <= timeoutMs;
  }
};

