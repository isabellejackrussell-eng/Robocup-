#ifndef FILTER_WEIGHT_DETECT_H
#define FILTER_WEIGHT_DETECT_H

#include <Arduino.h>

#include "hd_raw_tof.h"
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

// -----------------------------------------------------------------------------
// Six single-point ToF sensors
// -----------------------------------------------------------------------------

constexpr uint8_t kRangeDetectionCount = 4;

enum class RangeObjectType : uint8_t {
  none,
  weight,
  wall,
  unknown,
};

struct RangeObjectDetection {
  RangeObjectType type;
  uint16_t distanceMm;
};

// Measurements within this distance are treated as the same object. This is
// used to distinguish a tall wall seen by both sensor heights from a low
// weight seen only by a bottom sensor or the 8x8 sensor.
constexpr uint16_t kWallDistanceToleranceMm = 100;

// Clears the three-sample debounce/filter state.
void resetRangeDetections();

// Updates two bottom/top weight-detection pairs (0/1 and 2/3), followed by
// the two wall-only sensors (4 and 5).
void updateRangeDetections(const hd_raw_tof::Readings& readings);

RangeObjectDetection getRangeDetection(uint8_t detectionIndex);
const char* rangeObjectTypeName(RangeObjectType type);
void printRangeDetections();

// -----------------------------------------------------------------------------
// Combined right / centre / left weight identification
// -----------------------------------------------------------------------------

enum class WeightState : uint8_t {
  notFound,
  found,
};

struct WeightIdentification {
  WeightState state;

  // Average distance across every sensor region that identified the weight.
  uint16_t distanceMm;

  // Individual distances are retained for movement logic. A value of zero
  // means that sensor region did not identify a weight.
  uint16_t rightDistanceMm;
  uint16_t centreDistanceMm;
  uint16_t leftDistanceMm;

  // 8x8 average column, from 0 (left side of its image) to 7 (right side).
  // Set to -1 when the 8x8 sensor did not identify the weight.
  float column;

  // More than one of these can be true for the same wide/overlapping target.
  bool seenByRightTof;
  bool seenBy8x8;
  bool seenByLeftTof;
};

// Fuses the current single-point ToF readings with the result from the existing
// 8x8 weight detector. Set 0 is the right pair, set 1 is the left pair, and the
// 8x8 sensor is the centre. A centre candidate matching a top/wall ToF distance
// is rejected as a wall.
WeightIdentification identifyWeight(
    const hd_raw_tof::Readings& readings,
    const WeightResult& matrixWeight);

bool weightFound(const WeightIdentification& identification);
void printWeightIdentification(const WeightIdentification& identification);

}  // namespace filter_weightDetect

#endif
