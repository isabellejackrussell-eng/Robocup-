#ifndef WEIGHT_DETECT_H
#define WEIGHT_DETECT_H

#include <Arduino.h>

const int GRID_SIZE = 8;
const int NEAR_THRESHOLD_MM = 80;
const int MIN_CLUSTER_SIZE = 2;

struct WeightResult {
  bool found;
  float avgCol;
  float avgRow;
  int   avgDistMM;
  int   clusterSize;
};

int backgroundMM(uint16_t distances[GRID_SIZE * GRID_SIZE]);
WeightResult detectWeight(uint16_t distances[GRID_SIZE * GRID_SIZE]);
void printResult(WeightResult r);

#endif