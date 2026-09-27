#ifndef HD_RAW_TOF8X8_H
#define HD_RAW_TOF8X8_H

#include <Arduino.h>

namespace hd_raw_tof8x8 {

constexpr uint8_t kGridSize = 8;
constexpr size_t kPointCount = kGridSize * kGridSize;
constexpr uint8_t kDefaultI2cAddress = 0x33;

using Frame = uint16_t[kPointCount];

// Initialises the SEN0628 matrix LiDAR and selects its 8x8 ranging mode.
// Returns false when either operation fails so main.cpp can decide when to retry.
bool initialise();

// Reads one complete 8x8 distance frame in millimetres.
// Returns false if the sensor is not initialised or the read fails.
bool readFrame(Frame& frame);

bool isInitialised();

}  // namespace hd_raw_tof8x8

#endif
