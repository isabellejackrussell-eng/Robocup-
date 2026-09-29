#include <Arduino.h>
#include <math.h>

#include "filter_weightDetect.h"
#include "hd_move_motors.h"
#include "hd_move_servoArm.h"
#include "hd_raw_tof.h"
#include "hd_raw_tof8x8.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kMatrixConnectTimeoutMs = 3000;
constexpr uint32_t kMatrixRetryPeriodMs = 100;
constexpr size_t kCalibrationFrameCount = 10;
constexpr uint32_t kSensorPeriodMs = 100;
constexpr uint32_t kStatusPeriodMs = 500;

// Motor tests showed that commands around 250 could stall, while 325 moved
// both tracks. The right track was slower in forward motion, so it receives a
// small base-power offset. Steering keeps both tracks moving forward rather
// than using the unreliable low-power point turns.
constexpr int kLeftDrivePower = 325;
constexpr int kRightDrivePower = 375;
constexpr int kMinimumMovingPower = 250;
constexpr int kMaximumMotorPower = 450;
constexpr int kWeaveCorrection = 65;
constexpr uint32_t kWeavePeriodMs = 5000;
constexpr float kTargetColumn = 3.5f;
constexpr float kColumnSteeringGain = 45.0f;
constexpr int kMaximumApproachCorrection = 100;
constexpr uint16_t kWallTurnDistanceMm = 200;
constexpr int kWallTurnLeftPower = 425;
constexpr int kWallTurnRightPower = 250;

constexpr uint8_t kDetectionConfirmFrames = 3;
constexpr uint8_t kLostWeightFrames = 5;
constexpr uint8_t kClearConfirmFrames = 5;

// The physical touching/front test returned 40-43 mm. A 45 mm sensor
// threshold therefore represents the requested approximately 40 mm physical
// collection distance without risking a failure to trigger at 41-43 mm.
constexpr uint16_t kCollectionDistanceMm = 45;

constexpr int kArmDownAngleDegrees = 40;
constexpr int kArmUpAngleDegrees = 190;
constexpr int kArmStepDegrees = 2;
constexpr uint32_t kArmStepPeriodMs = 20;
constexpr uint32_t kArmHoldTimeMs = 750;

enum class RobotState : uint8_t {
  searching,
  approaching,
  sweepingArmUp,
  holdingArmUp,
  sweepingArmDown,
  waitingForWeightToClear,
  emergencyStopped,
};

enum class SensorSource : uint8_t {
  matrix8x8,
  rangeTofs,
};

RobotState state = RobotState::searching;
SensorSource sensorSource = SensorSource::matrix8x8;
filter_weightDetect::WeightResult latestWeight = {};
bool latestFrameValid = false;
bool rangeTofsAvailable = false;
bool closeWallAhead = false;
uint8_t detectionFrames = 0;
uint8_t lostFrames = 0;
uint8_t clearFrames = 0;
int armAngleDegrees = kArmDownAngleDegrees;
int commandedLeftPower = 0;
int commandedRightPower = 0;
uint32_t lastSensorMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastArmStepMs = 0;
uint32_t stateStartedMs = 0;

const char* sensorSourceName() {
  return sensorSource == SensorSource::matrix8x8 ? "8x8" : "TOFS";
}

const char* stateName(RobotState value) {
  switch (value) {
    case RobotState::searching:
      return "SEARCHING";
    case RobotState::approaching:
      return "APPROACHING";
    case RobotState::sweepingArmUp:
      return "SWEEPING ARM UP";
    case RobotState::holdingArmUp:
      return "HOLDING ARM UP";
    case RobotState::sweepingArmDown:
      return "SWEEPING ARM DOWN";
    case RobotState::waitingForWeightToClear:
      return "WAITING FOR WEIGHT TO CLEAR";
    case RobotState::emergencyStopped:
      return "EMERGENCY STOPPED";
    default:
      return "UNKNOWN";
  }
}

void setState(RobotState nextState) {
  if (state == nextState) {
    return;
  }

  state = nextState;
  stateStartedMs = millis();
  Serial.print("[ROBOT] ");
  Serial.println(stateName(state));
}

void commandMotors(int leftPower, int rightPower) {
  commandedLeftPower = constrain(
      leftPower, -kMaximumMotorPower, kMaximumMotorPower);
  commandedRightPower = constrain(
      rightPower, -kMaximumMotorPower, kMaximumMotorPower);
  motors_write(commandedLeftPower, commandedRightPower);
}

void stopMotors() {
  commandedLeftPower = 0;
  commandedRightPower = 0;
  motors_stop();
}

int weaveCorrection(uint32_t nowMs) {
  const uint32_t halfPeriodMs = kWeavePeriodMs / 2;
  const uint32_t phaseMs = nowMs % kWeavePeriodMs;
  float position = 0.0f;

  if (phaseMs < halfPeriodMs) {
    position = -1.0f + 2.0f * phaseMs / halfPeriodMs;
  } else {
    position = 3.0f - 2.0f * phaseMs / halfPeriodMs;
  }

  return static_cast<int>(roundf(position * kWeaveCorrection));
}

void driveSearchPattern(uint32_t nowMs) {
  const int correction = weaveCorrection(nowMs);
  commandMotors(
      kLeftDrivePower + correction,
      kRightDrivePower - correction);
}

void driveTowardsWeight(const filter_weightDetect::WeightResult& weight) {
  const float columnError = weight.averageColumn - kTargetColumn;
  const int correction = constrain(
      static_cast<int>(roundf(columnError * kColumnSteeringGain)),
      -kMaximumApproachCorrection,
      kMaximumApproachCorrection);

  const int leftPower = constrain(
      kLeftDrivePower + correction,
      kMinimumMovingPower,
      kMaximumMotorPower);
  const int rightPower = constrain(
      kRightDrivePower - correction,
      kMinimumMovingPower,
      kMaximumMotorPower);
  commandMotors(leftPower, rightPower);
}

bool calibrateEmptyScene() {
  Serial.println("[CALIBRATION] Keep the 8x8 view empty for three seconds");
  delay(3000);

  hd_raw_tof8x8::Frame frames[kCalibrationFrameCount];
  for (size_t frame = 0; frame < kCalibrationFrameCount; ++frame) {
    if (!hd_raw_tof8x8::readFrame(frames[frame])) {
      Serial.println("[CALIBRATION] Frame read failed");
      return false;
    }
    delay(50);
  }

  const bool calibrated = filter_weightDetect::calibrateBackgroundAveraged(
      frames, kCalibrationFrameCount);
  Serial.println(calibrated
      ? "[CALIBRATION] Background ready"
      : "[CALIBRATION] Background failed");
  return calibrated;
}

bool readMatrixWeight() {
  hd_raw_tof8x8::Frame frame;
  if (!hd_raw_tof8x8::readFrame(frame)) {
    latestFrameValid = false;
    latestWeight = {};
    Serial.println("[8x8] Frame read failed");
    return false;
  }

  latestFrameValid = true;
  latestWeight = filter_weightDetect::detectWeight(frame, false);
  return true;
}

void updateRangeSensorState(const hd_raw_tof::Readings& readings) {
  filter_weightDetect::updateRangeDetections(readings);
  closeWallAhead = false;

  for (uint8_t detection = 0;
       detection < filter_weightDetect::kRangeDetectionCount;
       ++detection) {
    const filter_weightDetect::RangeObjectDetection object =
        filter_weightDetect::getRangeDetection(detection);
    if (object.type == filter_weightDetect::RangeObjectType::wall &&
        object.distanceMm > 0 &&
        object.distanceMm <= kWallTurnDistanceMm) {
      closeWallAhead = true;
      break;
    }
  }
}

void readAuxiliaryRangeTofs() {
  if (!rangeTofsAvailable) {
    closeWallAhead = false;
    return;
  }

  hd_raw_tof::Readings readings;
  hd_raw_tof::readAll(readings);
  updateRangeSensorState(readings);
}

bool readRangeTofWeight() {
  hd_raw_tof::Readings readings;
  hd_raw_tof::readAll(readings);
  updateRangeSensorState(readings);

  const filter_weightDetect::WeightResult noMatrixWeight = {};
  const filter_weightDetect::WeightIdentification identification =
      filter_weightDetect::identifyWeight(readings, noMatrixWeight);

  latestFrameValid = false;
  for (uint8_t sensor = 0; sensor < hd_raw_tof::kSensorCount; ++sensor) {
    if (hd_raw_tof::isSensorInitialised(sensor)) {
      latestFrameValid = true;
      break;
    }
  }

  latestWeight = {};
  if (!filter_weightDetect::weightFound(identification)) {
    return latestFrameValid;
  }

  latestWeight.found = true;
  latestWeight.averageDistanceMm = identification.distanceMm;

  // The individual ToFs are arranged as right and left bottom/top pairs.
  // Convert their coarse direction into the same 0..7 steering coordinate
  // used by the 8x8 detector.
  if (identification.seenByRightTof && identification.seenByLeftTof) {
    latestWeight.averageColumn = kTargetColumn;
  } else if (identification.seenByRightTof) {
    latestWeight.averageColumn = 6.0f;
  } else {
    latestWeight.averageColumn = 1.0f;
  }
  return true;
}

bool readWeight() {
  if (sensorSource == SensorSource::rangeTofs) {
    return readRangeTofWeight();
  }

  const bool matrixRead = readMatrixWeight();
  readAuxiliaryRangeTofs();
  return matrixRead;
}

bool connectMatrixWithinTimeout() {
  const uint32_t startedMs = millis();
  do {
    if (hd_raw_tof8x8::initialise()) {
      return true;
    }
    Serial.println("[INIT] 8x8 sensor not connected; retrying...");
    delay(kMatrixRetryPeriodMs);
  } while (millis() - startedMs < kMatrixConnectTimeoutMs);

  return false;
}

void initialiseRangeTofs() {
  filter_weightDetect::resetRangeDetections();

  const bool allInitialised = hd_raw_tof::initialise();
  uint8_t initialisedCount = 0;
  for (uint8_t sensor = 0; sensor < hd_raw_tof::kSensorCount; ++sensor) {
    if (hd_raw_tof::isSensorInitialised(sensor)) {
      ++initialisedCount;
    }
  }
  rangeTofsAvailable = initialisedCount > 0;

  Serial.print("[INIT] ToFs: ");
  Serial.print(initialisedCount);
  Serial.print('/');
  Serial.print(hd_raw_tof::kSensorCount);
  Serial.println(" sensors connected");
  if (!allInitialised) {
    Serial.println("[INIT] Some ToFs failed; forward movement will continue");
  }
}

void useRangeTofFallback() {
  sensorSource = SensorSource::rangeTofs;
  initialiseRangeTofs();
  Serial.println("[INIT] ToF fallback active");
}

void startArmCollection() {
  stopMotors();
  detectionFrames = 0;
  lostFrames = 0;
  clearFrames = 0;
  lastArmStepMs = millis();
  setState(RobotState::sweepingArmUp);
}

void updateArm(uint32_t nowMs) {
  if (state == RobotState::holdingArmUp) {
    if (nowMs - stateStartedMs >= kArmHoldTimeMs) {
      lastArmStepMs = nowMs;
      setState(RobotState::sweepingArmDown);
    }
    return;
  }

  if (nowMs - lastArmStepMs < kArmStepPeriodMs) {
    return;
  }
  lastArmStepMs = nowMs;

  if (state == RobotState::sweepingArmUp) {
    armAngleDegrees = min(
        armAngleDegrees + kArmStepDegrees, kArmUpAngleDegrees);
    hd_move_servoArm::setAngle(armAngleDegrees);
    if (armAngleDegrees >= kArmUpAngleDegrees) {
      setState(RobotState::holdingArmUp);
    }
    return;
  }

  if (state == RobotState::sweepingArmDown) {
    armAngleDegrees = max(
        armAngleDegrees - kArmStepDegrees, kArmDownAngleDegrees);
    hd_move_servoArm::setAngle(armAngleDegrees);
    if (armAngleDegrees <= kArmDownAngleDegrees) {
      clearFrames = 0;
      setState(RobotState::waitingForWeightToClear);
    }
  }
}

void updateSearching(uint32_t nowMs) {
  if (latestFrameValid && latestWeight.found) {
    if (detectionFrames < kDetectionConfirmFrames) {
      ++detectionFrames;
    }
  } else {
    detectionFrames = 0;
  }

  if (detectionFrames >= kDetectionConfirmFrames) {
    lostFrames = 0;
    setState(RobotState::approaching);
    driveTowardsWeight(latestWeight);
    return;
  }

  if ((!latestFrameValid || !latestWeight.found) && closeWallAhead) {
    commandMotors(kWallTurnLeftPower, kWallTurnRightPower);
    return;
  }

  driveSearchPattern(nowMs);
}

void updateApproaching() {
  if ((!latestFrameValid || !latestWeight.found) && closeWallAhead) {
    detectionFrames = 0;
    lostFrames = 0;
    setState(RobotState::searching);
    commandMotors(kWallTurnLeftPower, kWallTurnRightPower);
    return;
  }

  if (!latestFrameValid || !latestWeight.found) {
    if (lostFrames < kLostWeightFrames) {
      ++lostFrames;
    }

    if (lostFrames >= kLostWeightFrames) {
      detectionFrames = 0;
      setState(RobotState::searching);
    } else {
      // Continue gently forward through brief one-frame dropouts.
      commandMotors(kLeftDrivePower, kRightDrivePower);
    }
    return;
  }

  lostFrames = 0;
  if (latestWeight.averageDistanceMm <= kCollectionDistanceMm) {
    Serial.print("[COLLECTION] Weight at ");
    Serial.print(latestWeight.averageDistanceMm);
    Serial.println(" mm; stopping and sweeping arm");
    startArmCollection();
    return;
  }

  driveTowardsWeight(latestWeight);
}

void updateWaitingForClear() {
  stopMotors();
  if (!latestFrameValid || latestWeight.found) {
    clearFrames = 0;
    return;
  }

  if (clearFrames < kClearConfirmFrames) {
    ++clearFrames;
  }
  if (clearFrames >= kClearConfirmFrames) {
    detectionFrames = 0;
    lostFrames = 0;
    setState(RobotState::searching);
  }
}

void printStatus(uint32_t nowMs) {
  if (nowMs - lastStatusMs < kStatusPeriodMs) {
    return;
  }
  lastStatusMs = nowMs;

  Serial.print("[STATUS] state=");
  Serial.print(stateName(state));
  Serial.print(" sensor=");
  Serial.print(sensorSourceName());
  Serial.print(" motors=");
  Serial.print(commandedLeftPower);
  Serial.print(',');
  Serial.print(commandedRightPower);
  Serial.print(" arm=");
  Serial.print(armAngleDegrees);
  Serial.print(" weight=");
  Serial.print(latestFrameValid && latestWeight.found ? "YES" : "NO");
  Serial.print(" wall<=200mm=");
  Serial.print(closeWallAhead ? "YES" : "NO");
  if (latestFrameValid && latestWeight.found) {
    Serial.print(" distance=");
    Serial.print(latestWeight.averageDistanceMm);
    Serial.print("mm column=");
    Serial.print(latestWeight.averageColumn, 2);
  }
  Serial.println();
}

void readCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'x' || command == 'X') {
      stopMotors();
      setState(RobotState::emergencyStopped);
      Serial.println("[COMMAND] Emergency stop; send 'g' to resume");
    } else if ((command == 'g' || command == 'G') &&
               state == RobotState::emergencyStopped) {
      stopMotors();
      armAngleDegrees = kArmDownAngleDegrees;
      hd_move_servoArm::setAngle(armAngleDegrees);
      detectionFrames = 0;
      lostFrames = 0;
      clearFrames = 0;
      setState(RobotState::searching);
      Serial.println("[COMMAND] Autonomous hunting resumed");
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);

  pinMode(kIoPowerPin, OUTPUT);
  digitalWrite(kIoPowerPin, HIGH);

  // Arm the motor controllers first. As soon as their required stop-pulse
  // interval has elapsed, drive forward while the sensors finish starting.
  motors_init();
  commandMotors(kLeftDrivePower, kRightDrivePower);

  Serial.println();
  Serial.println("Autonomous weight hunting and collection");
  Serial.println("Send 'x' at any time to stop; send 'g' to resume.");

  if (connectMatrixWithinTimeout()) {
    sensorSource = SensorSource::matrix8x8;
    Serial.println("[INIT] 8x8 sensor OK");
  } else {
    Serial.println("[INIT] 8x8 unavailable after 3 seconds");
    useRangeTofFallback();
  }

  if (!hd_move_servoArm::initialise(kArmDownAngleDegrees)) {
    Serial.println("[INIT] Servo arm failed; robot stopped");
    setState(RobotState::emergencyStopped);
    return;
  }
  armAngleDegrees = kArmDownAngleDegrees;
  Serial.println("[INIT] Servo arm at 40 degrees");

  if (sensorSource == SensorSource::matrix8x8) {
    filter_weightDetect::clearBackgroundCalibration();
    if (!calibrateEmptyScene()) {
      Serial.println("[CALIBRATION] 8x8 failed; changing to ToF fallback");
      useRangeTofFallback();
    }
  }

  if (sensorSource == SensorSource::matrix8x8) {
    // These sensors provide wall avoidance while the 8x8 remains the primary
    // weight detector.
    initialiseRangeTofs();
  }

  lastSensorMs = millis() - kSensorPeriodMs;
  setState(RobotState::searching);
  commandMotors(kLeftDrivePower, kRightDrivePower);
  Serial.println("[ROBOT] Autonomous hunting started; moving forward");
}

void loop() {
  readCommands();
  const uint32_t nowMs = millis();

  if (state == RobotState::emergencyStopped) {
    stopMotors();
    printStatus(nowMs);
    return;
  }

  if (state == RobotState::sweepingArmUp ||
      state == RobotState::holdingArmUp ||
      state == RobotState::sweepingArmDown) {
    stopMotors();
    updateArm(nowMs);
    printStatus(nowMs);
    return;
  }

  if (nowMs - lastSensorMs < kSensorPeriodMs) {
    printStatus(nowMs);
    return;
  }
  lastSensorMs = nowMs;
  readWeight();

  switch (state) {
    case RobotState::searching:
      updateSearching(nowMs);
      break;
    case RobotState::approaching:
      updateApproaching();
      break;
    case RobotState::waitingForWeightToClear:
      updateWaitingForClear();
      break;
    case RobotState::sweepingArmUp:
    case RobotState::holdingArmUp:
    case RobotState::sweepingArmDown:
    case RobotState::emergencyStopped:
      break;
  }

  printStatus(nowMs);
}
