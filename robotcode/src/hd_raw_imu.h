#pragma once

#include <Arduino.h>

namespace hd_raw_imu {

constexpr uint8_t kI2cAddress = 0x28;
constexpr uint32_t kRecommendedSampleIntervalMs = 20;  // 50 Hz

struct Vector3 {
  float x;
  float y;
  float z;
};

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct Calibration {
  uint8_t system;
  uint8_t gyroscope;
  uint8_t accelerometer;
  uint8_t magnetometer;
};

struct Reading {
  Vector3 linearAccelerationMps2;
  Vector3 gyroscopeRadiansPerSecond;

  float headingDegrees;
  float rollDegrees;
  float pitchDegrees;
  Quaternion orientation;

  Calibration calibration;
  int8_t temperatureCelsius;
  uint32_t timestampMs;
  bool valid;
};

// Initialises the BNO055 at address 0x28 and enables its external crystal.
bool initialise();

// Reads one complete raw sample from the BNO055.
bool read(Reading& reading);

bool isInitialised();

}  // namespace hd_raw_imu
