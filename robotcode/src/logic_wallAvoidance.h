#ifndef LOGIC_WALL_AVOIDANCE_H
#define LOGIC_WALL_AVOIDANCE_H

#include <Arduino.h>

#include "hd_raw_tof.h"

namespace logic_wallAvoidance {

// The four single-point ToFs are arranged as two bottom/top pairs:
//   sensor 0 = bottom right
//   sensor 1 = top right
//   sensor 2 = bottom left
//   sensor 3 = top left
// Only the two TOP sensors are used by wall avoidance.
constexpr uint8_t kTopRightSensorIndex = 1;
constexpr uint8_t kTopLeftSensorIndex = 3;
constexpr uint16_t kWallThresholdMm = 200;

constexpr float kSingleSensorTurnDegrees = 20.0f;
constexpr float kBothSensorsTurnDegrees = 180.0f;
constexpr float kTurnToleranceDegrees = 3.0f;
constexpr int kTurnMotorPower = 325;
constexpr uint32_t kTurnTimeoutMs = 6000;
constexpr uint32_t kTurnUpdatePeriodMs = 20;

enum class Turn : uint8_t {
  none,
  right20,
  around180,
};

struct Status {
  bool topLeftClose;
  bool topRightClose;
  bool turning;
  Turn turn;
  float startHeadingDegrees;
  float turnedDegrees;
  float targetDegrees;
};

// Requires motors, the BNO055, and the four single-point ToFs to have already
// been initialised.
bool initialise();

// Call once per control cycle after hd_raw_tof::readAll(). If a wall is within
// 200 mm of either top sensor this function takes control of the drive motors:
//   top-left only  -> point-turn right 20 degrees
//   top-right only -> point-turn right 20 degrees
//   both           -> point-turn right 180 degrees
// Returns true while wall avoidance owns the motors for this cycle.
bool update(
    const hd_raw_tof::Readings& readings,
    uint32_t nowMs = millis());

// True whenever either top sensor currently sees something inside 200 mm, or
// while a commanded avoidance turn is still being completed. Use this to
// inhibit claw capture.
bool clawInhibited();
bool wallDetected();
bool isTurning();
const Status& getStatus();
const char* turnName(Turn turn);

}  // namespace logic_wallAvoidance

#endif
