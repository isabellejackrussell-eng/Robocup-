#pragma once

#include <Arduino.h>

namespace hd_raw_encoder {

// Pin assignments copied from the working encoder sensor test.
constexpr uint8_t kLeftPinA = 2;
constexpr uint8_t kLeftPinB = 3;
constexpr uint8_t kRightPinA = 4;
constexpr uint8_t kRightPinB = 5;

struct Reading {
  int32_t leftTicks;
  int32_t rightTicks;
  uint32_t timestampMs;
  bool valid;
};

// Configures both quadrature encoders and attaches their interrupts.
bool initialise();

// Takes an atomic snapshot of both cumulative tick counts.
bool read(Reading& reading);

// Resets both cumulative counts to zero.
void reset();

bool isInitialised();

}  // namespace hd_raw_encoder
