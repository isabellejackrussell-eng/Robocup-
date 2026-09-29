#include <Arduino.h>
#include <math.h>

#include "filter_weightDetect.h"
#include "hd_move_motors.h"
#include "hd_move_servoArm.h"
#include "hd_raw_imu.h"
#include "hd_raw_tof.h"
#include "hd_raw_tof8x8.h"
#include "logic_wallAvoidance.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 3000;
constexpr uint32_t kRetryPeriodMs = 1000;
constexpr uint32_t kSamplePeriodMs = 100;
constexpr uint32_t kRangeSamplePeriodMs = 50;
constexpr uint32_t kWeightDetectionHoldMs = 750;
constexpr uint16_t kCaptureZoneDistanceMm = 60;
constexpr float kCaptureZoneMinimumColumn = 5.5f;
constexpr float kCaptureZoneMaximumColumn = 7.0f;
constexpr float kTargetColumn =
    (kCaptureZoneMinimumColumn + kCaptureZoneMaximumColumn) / 2.0f;
constexpr int kApproachLeftPower = 275;
constexpr int kApproachRightPower = 325;
constexpr int kMinimumMovingPower = 250;
constexpr int kMaximumMotorPower = 450;
constexpr float kSteeringGain = 35.0f;
constexpr int kMaximumSteeringCorrection = 70;
constexpr int kCloseAlignmentTurnPower = 300;
constexpr int kArmUpAngleDegrees = 40;
constexpr int kArmDownAngleDegrees = 200;
constexpr int kArmStepDegrees = 2;
constexpr uint32_t kArmStepPeriodMs = 20;
constexpr uint32_t kArmDownHoldMs = 750;

// Per-pixel median of the 20 no-weight frames in the newest recorded dataset.
// This lets the diagnostic detect immediately without requiring calibration
// every time it powers on. Send 'c' with an empty view to replace it when the
// sensor height, angle, or floor changes.
constexpr hd_raw_tof8x8::Frame kRecordedEmptyScene = {
    4000, 1553, 4000, 4000, 4000, 1632, 1655, 4000,
    1533, 1518, 1613, 4000, 4000, 4000, 1663, 4000,
    897, 903, 918, 927, 935, 941, 944, 952,
    5, 5, 910, 922, 928, 932, 935, 941,
    5, 6, 893, 912, 918, 919, 922, 925,
    9, 8, 16, 893, 900, 249, 283, 291,
    23, 42, 47, 88, 117, 152, 167, 169,
    11, 89, 86, 98, 103, 88, 97, 68,
};

filter_weightDetect::WeightResult heldWeight = {};
bool haveHeldWeight = false;
bool motionEnabled = true;
bool captureCycleLatched = false;
bool haveRangeReadings = false;
uint32_t lastWeightDetectionMs = 0;
uint32_t lastMatrixSampleMs = 0;
uint32_t lastRangeSampleMs = 0;
hd_raw_tof::Readings rangeReadings = {};

void printFrame(const hd_raw_tof8x8::Frame& frame) {
  for (uint8_t row = 0; row < hd_raw_tof8x8::kGridSize; ++row) {
    Serial.print('Y');
    Serial.print(row);
    Serial.print(": ");
    for (uint8_t column = 0;
         column < hd_raw_tof8x8::kGridSize;
         ++column) {
      Serial.print(frame[row * hd_raw_tof8x8::kGridSize + column]);
      Serial.print(',');
    }
    Serial.println();
  }
}

bool readFrame(hd_raw_tof8x8::Frame& frame) {
  if (hd_raw_tof8x8::readFrame(frame)) {
    return true;
  }

  Serial.println("8x8 frame read failed");
  return false;
}

void clearHeldWeight() {
  heldWeight = {};
  haveHeldWeight = false;
  lastWeightDetectionMs = 0;
}

filter_weightDetect::WeightResult stabiliseWeightDetection(
    const filter_weightDetect::WeightResult& current,
    uint32_t nowMs) {
  if (current.found) {
    heldWeight = current;
    haveHeldWeight = true;
    lastWeightDetectionMs = nowMs;
    return current;
  }

  if (haveHeldWeight &&
      nowMs - lastWeightDetectionMs <= kWeightDetectionHoldMs) {
    Serial.print("[8x8] Holding confirmed weight through frame dropout (age=");
    Serial.print(nowMs - lastWeightDetectionMs);
    Serial.println("ms)");
    return heldWeight;
  }

  clearHeldWeight();
  return current;
}

bool isInCaptureZone(const filter_weightDetect::WeightResult& weight) {
  return weight.found &&
      weight.averageDistanceMm <= kCaptureZoneDistanceMm &&
      weight.averageColumn >= kCaptureZoneMinimumColumn &&
      weight.averageColumn <= kCaptureZoneMaximumColumn;
}

void printCaptureZone(const filter_weightDetect::WeightResult& weight) {
  const bool inCaptureZone = isInCaptureZone(weight);
  Serial.print("CAPTURE ZONE: ");
  Serial.print(inCaptureZone ? "YES" : "NO");
  if (weight.found) {
    Serial.print(" (distance=");
    Serial.print(weight.averageDistanceMm);
    Serial.print("mm, threshold=");
    Serial.print(kCaptureZoneDistanceMm);
    Serial.print("mm, column=");
    Serial.print(weight.averageColumn, 2);
    Serial.print(", allowed=");
    Serial.print(kCaptureZoneMinimumColumn, 2);
    Serial.print("..");
    Serial.print(kCaptureZoneMaximumColumn, 2);
    Serial.print(')');
  }
  Serial.println();
}

void driveForward() {
  motors_write(kApproachLeftPower, kApproachRightPower);
}

void runCaptureArmCycle() {
  motors_stop();
  Serial.println("[CAPTURE] Weight in zone; sweeping arm 40 -> 200 degrees");

  for (int angle = kArmUpAngleDegrees;
       angle <= kArmDownAngleDegrees;
       angle += kArmStepDegrees) {
    hd_move_servoArm::setAngle(angle);
    delay(kArmStepPeriodMs);
  }
  hd_move_servoArm::setAngle(kArmDownAngleDegrees);
  delay(kArmDownHoldMs);

  Serial.println("[CAPTURE] Returning arm to 40 degrees");
  for (int angle = kArmDownAngleDegrees;
       angle >= kArmUpAngleDegrees;
       angle -= kArmStepDegrees) {
    hd_move_servoArm::setAngle(angle);
    delay(kArmStepPeriodMs);
  }
  hd_move_servoArm::setAngle(kArmUpAngleDegrees);
  Serial.println("[CAPTURE] Arm cycle complete");
}

void updateMotion(const filter_weightDetect::WeightResult& weight) {
  if (!motionEnabled) {
    motors_stop();
    Serial.println("[MOTION] STOPPED BY COMMAND");
    return;
  }

  if (logic_wallAvoidance::clawInhibited()) {
    motors_stop();
    Serial.println("[MOTION] STOPPED - WALL TOO CLOSE FOR CLAW");
    return;
  }

  if (captureCycleLatched) {
    if (!weight.found) {
      captureCycleLatched = false;
      Serial.println("[CAPTURE] Collected weight cleared; next capture armed");
    }
    driveForward();
    Serial.println("[MOTION] FORWARD AFTER CAPTURE");
    return;
  }

  if (!weight.found) {
    driveForward();
    Serial.println("[MOTION] FORWARD - SEARCHING FOR WEIGHT");
    return;
  }

  if (isInCaptureZone(weight)) {
    motors_stop();
    Serial.println("[MOTION] STOPPED - WEIGHT IN CAPTURE ZONE");
    if (!captureCycleLatched) {
      captureCycleLatched = true;
      runCaptureArmCycle();
    }
    driveForward();
    Serial.println("[MOTION] CAPTURE COMPLETE - CONTINUING FORWARD");
    return;
  }

  const float columnError = weight.averageColumn - kTargetColumn;
  if (weight.averageDistanceMm <= kCaptureZoneDistanceMm) {
    const int turnPower = columnError > 0.0f
        ? kCloseAlignmentTurnPower
        : -kCloseAlignmentTurnPower;
    motors_write(turnPower, -turnPower);
    Serial.println(columnError > 0.0f
        ? "[MOTION] CLOSE WEIGHT - ALIGNING RIGHT"
        : "[MOTION] CLOSE WEIGHT - ALIGNING LEFT");
    return;
  }

  const int correction = constrain(
      static_cast<int>(roundf(columnError * kSteeringGain)),
      -kMaximumSteeringCorrection,
      kMaximumSteeringCorrection);
  const int leftPower = constrain(
      kApproachLeftPower + correction,
      kMinimumMovingPower,
      kMaximumMotorPower);
  const int rightPower = constrain(
      kApproachRightPower - correction,
      kMinimumMovingPower,
      kMaximumMotorPower);
  motors_write(leftPower, rightPower);
  Serial.print("[MOTION] APPROACHING weight; motors=");
  Serial.print(leftPower);
  Serial.print(',');
  Serial.println(rightPower);
}

void readCommands() {
  if (Serial.available() <= 0) {
    return;
  }

  const char command = static_cast<char>(Serial.read());
  if (command == 'x' || command == 'X') {
    motionEnabled = false;
    motors_stop();
    Serial.println("[COMMAND] Motion stopped; send 'g' to resume");
    return;
  }
  if (command == 'g' || command == 'G') {
    motionEnabled = true;
    Serial.println("[COMMAND] Weight approach enabled");
    return;
  }
  if (command != 'c' && command != 'C') {
    return;
  }

  motors_stop();

  hd_raw_tof8x8::Frame calibrationFrame;
  if (!readFrame(calibrationFrame)) {
    Serial.println("Background calibration failed");
    return;
  }

  filter_weightDetect::calibrateBackground(calibrationFrame);
  clearHeldWeight();
  Serial.println("Background calibrated");
}

bool updateWallAvoidance(uint32_t nowMs) {
  const bool rangeSampleDue =
      nowMs - lastRangeSampleMs >= kRangeSamplePeriodMs;
  if (rangeSampleDue) {
    lastRangeSampleMs = nowMs;
    hd_raw_tof::readAll(rangeReadings);
    haveRangeReadings = true;
  }

  if (!haveRangeReadings) {
    return false;
  }

  // Continue an active turn at IMU rate. Start a new bang-bang decision only
  // after a fresh ToF sample so stale readings cannot immediately retrigger.
  if (logic_wallAvoidance::isTurning() || rangeSampleDue) {
    return logic_wallAvoidance::update(rangeReadings, nowMs);
  }
  return false;
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  const uint32_t serialWaitStartedMs = millis();
  while (!Serial && millis() - serialWaitStartedMs < kSerialWaitMs) {
  }

  pinMode(kIoPowerPin, OUTPUT);
  digitalWrite(kIoPowerPin, HIGH);
  delay(500);

  while (!hd_raw_tof8x8::initialise()) {
    Serial.println("begin error, retrying...");
    delay(kRetryPeriodMs);
  }

  motors_init();
  motors_stop();

  if (!hd_move_servoArm::initialise(kArmUpAngleDegrees)) {
    motionEnabled = false;
    Serial.println("[INIT] Servo arm failed; motion disabled");
  } else {
    Serial.println("[INIT] Servo arm raised to 40 degrees");
  }

  const bool allRangeTofsReady = hd_raw_tof::initialise();
  Serial.println(allRangeTofsReady
      ? "[INIT] All four range ToFs ready"
      : "[INIT] One or more range ToFs unavailable");

  const bool imuReady = hd_raw_imu::initialise();
  Serial.println(imuReady
      ? "[INIT] IMU ready"
      : "[INIT] IMU unavailable");
  logic_wallAvoidance::initialise();

  filter_weightDetect::calibrateBackground(kRecordedEmptyScene);
  Serial.println("begin success");
  Serial.println("init success, starting readings...");
  Serial.println(
      "Recorded empty-scene background loaded; 'c' optionally recalibrates.");
  Serial.println("Send 'x' to stop motion and 'g' to resume weight approach.");
  if (motionEnabled) {
    driveForward();
    Serial.println("[MOTION] Startup complete - moving forward");
  }
}

void loop() {
  readCommands();

  // The serial emergency stop has priority over every autonomous layer,
  // including a wall turn that was already in progress.
  if (!motionEnabled) {
    motors_stop();
    return;
  }

  const uint32_t nowMs = millis();
  if (updateWallAvoidance(nowMs)) {
    return;
  }

  if (logic_wallAvoidance::clawInhibited()) {
    motors_stop();
    return;
  }

  if (nowMs - lastMatrixSampleMs < kSamplePeriodMs) {
    return;
  }
  lastMatrixSampleMs = nowMs;

  hd_raw_tof8x8::Frame frame;
  if (!readFrame(frame)) {
    motors_stop();
    return;
  }

  printFrame(frame);
  Serial.println("------------------------------");

  const filter_weightDetect::WeightResult rawResult =
      filter_weightDetect::detectWeight(frame, true);
  const filter_weightDetect::WeightResult result =
      stabiliseWeightDetection(rawResult, millis());
  filter_weightDetect::printWeightResult(result);
  printCaptureZone(result);
  updateMotion(result);
  Serial.println("------------------------------");
}
