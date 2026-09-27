#ifndef HD_RAW_ULTRASONIC_H
#define HD_RAW_ULTRASONIC_H

#include <Arduino.h>

namespace hd_raw_ultrasonic {

constexpr uint8_t kSensorCount = 2;
constexpr uint32_t kEchoTimeoutUs = 30000;

enum class Sensor : uint8_t {
  a = 0,
  b = 1,
};

struct Reading {
  uint32_t echoDurationUs;
  uint16_t distanceCm;
  uint32_t timestampMs;
  bool valid;
  bool timedOut;
};

using Readings = Reading[kSensorCount];

void initialise();

bool readSensor(Sensor sensor, Reading& reading);
void readAll(Readings& readings);

uint16_t microsecondsToCentimetres(uint32_t microseconds);

}  // namespace hd_raw_ultrasonic

#endif
