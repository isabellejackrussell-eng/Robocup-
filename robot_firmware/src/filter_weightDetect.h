#ifndef FILTER_WEIGHT_DETECT_H
#define FILTER_WEIGHT_DETECT_H

#include <Arduino.h>

#include "hd_raw_tof8x8.h"

namespace filter_weightDetect {

struct WeightResult {
  bool found;
  float averageColumn;
  float averageRow;
  uint16_t averageDistanceMm;
  uint8_t clusterSize;
};

// Capture the empty scene at the sensor's final height and angle.
bool calibrateBackground(const hd_raw_tof8x8::Frame& distances);

// Average several empty-scene frames to reduce calibration jitter.
bool calibrateBackgroundAveraged(
    const hd_raw_tof8x8::Frame* frames,
    size_t frameCount);

bool isBackgroundCalibrated();
void clearBackgroundCalibration();

// Compare a frame with the calibrated background and find the best
// weight-shaped foreground cluster.
WeightResult detectWeight(
    const hd_raw_tof8x8::Frame& distances,
    bool verbose = true);

void printWeightResult(const WeightResult& result);

}  // namespace filter_weightDetect

#endif
