#pragma once

#include <Arduino.h>

#include "common/reading.h"
#include "config/robot_config.h"

namespace sensors::range_tof {

bool initRangeToFSensors();
bool read(uint8_t index);
void readAll();
DistanceReading get(uint8_t index);
bool configured(uint8_t index);
bool healthy(uint8_t index);

}  // namespace sensors::range_tof

