#ifndef WEIGHT_DETECT_H
#define WEIGHT_DETECT_H

#include <Arduino.h>

constexpr int GRID_SIZE = 8;
constexpr int NEAR_DELTA_MM = 40;
constexpr int MIN_CLUSTER_SIZE = 2;
constexpr int FLATNESS_THRESHOLD_MM = 40;
constexpr float WEIGHT_DIAMETER_MM = 50.0f;
constexpr float HFOV_DEG = 60.0f;

struct WeightResult
{
  bool found;
  float avgCol;
  float avgRow;
  int avgDistMM;
  int clusterSize;
};

void calibrateBackground(uint16_t distances[GRID_SIZE * GRID_SIZE]);
void calibrateBackgroundAveraged(
    uint16_t frames[][GRID_SIZE * GRID_SIZE], int numFrames);
bool isBackgroundCalibrated();

WeightResult detectWeight(
    uint16_t distances[GRID_SIZE * GRID_SIZE], bool verbose, Print &output);
void printResult(const WeightResult &result, Print &output);

#endif
