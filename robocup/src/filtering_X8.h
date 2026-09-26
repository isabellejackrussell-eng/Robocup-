#ifndef FILTERING_8X8_H
#define FILTERING_8X8_H

#include <Arduino.h>

struct WeightResult {
    bool found;
    float avgCol;
    float avgRow;
    int avgDistMM;
    int clusterSize;
};

void calibrateBackground(uint16_t distances[64]);

void calibrateBackgroundAveraged(
    uint16_t frames[][64],
    int numFrames
);

WeightResult detectWeight(
    uint16_t distances[64],
    bool verbose
);

void printResult(WeightResult r);

#endif