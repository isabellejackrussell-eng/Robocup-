#pragma once

#include "common/reading.h"

namespace sensors::wall {

struct WallSensorRaw {
  DistanceReading ultrasonicLeft;
  DistanceReading ultrasonicFront;
  DistanceReading ultrasonicRight;
  DistanceReading crossToFLeft;
  DistanceReading crossToFRight;
  uint32_t timestampMs = 0;
};

bool initWallSensors();
void serviceUltrasonics();
void readUltrasonicLeft();
void readUltrasonicFront();
void readUltrasonicRight();
void readCrossToFLeft();
void readCrossToFRight();
void readAllWallSensors();
const WallSensorRaw& getWallSensorRaw();

}  // namespace sensors::wall

