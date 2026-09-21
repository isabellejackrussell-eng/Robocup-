#pragma once

#include <Arduino.h>

#include "config/robot_config.h"

namespace sensors::tof8x8 {

struct Frame {
  uint16_t distanceMm[config::TOF8X8_POINT_COUNT] = {};
  bool valid = false;
  uint32_t timestampMs = 0;
};

bool init8x8();
bool read8x8();
const Frame& get8x8Frame();
bool calibrate8x8();
bool hasCalibrationFrame();
const Frame& getCalibrationFrame();
bool healthy();

}  // namespace sensors::tof8x8

