#ifndef WEIGHT_DETECT_H
#define WEIGHT_DETECT_H

#include <Arduino.h>

const int GRID_SIZE = 8;
const int NEAR_DELTA_MM = 40;          // how much closer than THIS pixel's own background counts as "object"
const int MIN_CLUSTER_SIZE = 2;        // absolute floor, still checked before the distance-aware window
const int FLATNESS_THRESHOLD_MM = 80;  // max depth spread allowed within one cluster
const float WEIGHT_DIAMETER_MM = 50.0f;
const float HFOV_DEG = 60.0f;          // SEN0628 horizontal FOV

struct WeightResult {
  bool found;
  float avgCol;      // 0-7, column centroid (use for left/right direction)
  float avgRow;
  int   avgDistMM;
  int   clusterSize;
};

// Call once at setup(), with the sensor pointed at empty floor/wall exactly
// as it'll sit on the robot. Re-calibrate any time height or tilt angle changes.
void calibrateBackground(uint16_t distances[GRID_SIZE * GRID_SIZE]);

// Optional: average a few frames first to smooth out ToF jitter before storing.
void calibrateBackgroundAveraged(uint16_t frames[][GRID_SIZE * GRID_SIZE], int numFrames);

// Main detection call - requires calibrateBackground() to have been called first.
WeightResult detectWeight(uint16_t distances[GRID_SIZE * GRID_SIZE], bool verbose = true);

void printResult(WeightResult r);

#endif