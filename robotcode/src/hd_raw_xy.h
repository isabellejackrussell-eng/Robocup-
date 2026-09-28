#ifndef HD_RAW_XY_H
#define HD_RAW_XY_H

#include <Arduino.h>

namespace hd_raw_xy {

constexpr uint8_t kChipSelectPin = 10;

struct MotionReading {
  int16_t deltaXCounts;
  int16_t deltaYCounts;
  uint32_t timestampMs;
  bool valid;
};

// Initialises the PMW3901 optical-flow sensor.
bool initialise();

// Reads the incremental X/Y motion counts since the previous sensor read.
// These are raw sensor counts, not millimetres or a global position.
bool readMotion(MotionReading& reading);

bool isInitialised();

}  // namespace hd_raw_xy

#endif
