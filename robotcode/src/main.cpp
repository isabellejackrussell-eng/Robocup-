#include <Arduino.h>
#include <math.h>

#include "filter_weightDetect.h"
#include "hd_move_motors.h"
#include "hd_move_servoArm.h"
#include "hd_raw_imu.h"
#include "hd_raw_tof.h"
#include "hd_raw_tof8x8.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kMatrixConnectTimeoutMs = 3000;
constexpr uint32_t kMatrixRetryPeriodMs = 100;
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
// Pair 0 is on the right (bottom 0, top 1); pair 1 is on the left
// (bottom 2, top 3). Wall avoidance deliberately uses only the top sensors.
constexpr uint8_t kTopRightTofIndex = 1;
constexpr uint8_t kTopLeftTofIndex = 3;
constexpr uint16_t kTopWallThresholdMm = 200;
constexpr float kSingleWallTurnDegrees = 20.0f;
constexpr float kBothWallsTurnDegrees = 180.0f;
constexpr int kWallPointTurnPower = 325;
constexpr uint32_t kWallStopTimeMs = 250;
constexpr uint32_t kFallbackTurnMsPerDegree = 12;
constexpr uint32_t kWallTurnTimeoutExtraMs = 1500;

constexpr uint8_t kDetectionConfirmFrames = 3;
constexpr uint8_t kLostWeightFrames = 5;
constexpr uint8_t kClearConfirmFrames = 5;

// The physical touching/front test returned 40-43 mm. A 45 mm sensor
// threshold therefore represents the requested approximately 40 mm physical
// collection distance without risking a failure to trigger at 41-43 mm.
constexpr uint16_t kCollectionDistanceMm = 45;

constexpr int kArmUpAngleDegrees = 40;
constexpr int kArmDownAngleDegrees = 200;
constexpr int kArmStepDegrees = 2;
constexpr uint32_t kArmStepPeriodMs = 20;
constexpr uint32_t kArmCatchHoldTimeMs = 750;
constexpr uint32_t kArmGuardPeriodMs = 500;

enum class RobotState : uint8_t {
  searching,
  approaching,
  sweepingArmUp,
  holdingArmDown,
  sweepingArmDown,
  waitingForWeightToClear,
  stoppingForWall,
  turningFromWall,
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
bool topLeftWallClose = false;
bool topRightWallClose = false;
uint16_t topLeftDistanceMm = 0;
uint16_t topRightDistanceMm = 0;
bool imuAvailable = false;
bool wallTurnUsesImu = false;
bool haveWallTurnHeading = false;
float wallTurnTargetDegrees = 0.0f;
float wallTurnAccumulatedDegrees = 0.0f;
float previousWallTurnHeadingDegrees = 0.0f;
uint32_t wallTurnStartedMs = 0;
uint32_t lastWallImuMs = 0;
uint8_t detectionFrames = 0;
uint8_t lostFrames = 0;
uint8_t clearFrames = 0;
int armAngleDegrees = kArmUpAngleDegrees;
int commandedLeftPower = 0;
int commandedRightPower = 0;
uint32_t lastSensorMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastArmStepMs = 0;
uint32_t lastArmGuardMs = 0;
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
    case RobotState::holdingArmDown:
      return "HOLDING ARM DOWN";
    case RobotState::sweepingArmDown:
      return "SWEEPING ARM DOWN";
    case RobotState::waitingForWeightToClear:
      return "WAITING FOR WEIGHT TO CLEAR";
    case RobotState::stoppingForWall:
      return "STOPPING FOR WALL";
    case RobotState::turningFromWall:
      return "TURNING RIGHT FROM WALL";
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

  topRightDistanceMm = readings[kTopRightTofIndex].valid
      ? readings[kTopRightTofIndex].distanceMm
      : 0;
  topLeftDistanceMm = readings[kTopLeftTofIndex].valid
      ? readings[kTopLeftTofIndex].distanceMm
      : 0;
  topRightWallClose =
      topRightDistanceMm > 0 && topRightDistanceMm < kTopWallThresholdMm;
  topLeftWallClose =
      topLeftDistanceMm > 0 && topLeftDistanceMm < kTopWallThresholdMm;
  closeWallAhead = topLeftWallClose || topRightWallClose;
}

void applyWeightIdentification(
    const filter_weightDetect::WeightIdentification& identification) {
  latestWeight = {};
  if (!filter_weightDetect::weightFound(identification)) {
    return;
  }

  latestWeight.found = true;
  latestWeight.averageDistanceMm = identification.seenBy8x8
      ? identification.centreDistanceMm
      : identification.distanceMm;

  if (identification.seenBy8x8) {
    latestWeight.averageColumn = identification.column;
  } else if (identification.seenByRightTof &&
             identification.seenByLeftTof) {
    latestWeight.averageColumn = kTargetColumn;
  } else if (identification.seenByRightTof) {
    latestWeight.averageColumn = 6.0f;
  } else {
    latestWeight.averageColumn = 1.0f;
  }
}

void readAuxiliaryRangeTofs() {
  if (!rangeTofsAvailable) {
    closeWallAhead = false;
    topLeftWallClose = false;
    topRightWallClose = false;
    topLeftDistanceMm = 0;
    topRightDistanceMm = 0;
    return;
  }

  hd_raw_tof::Readings readings;
  hd_raw_tof::readAll(readings);
  updateRangeSensorState(readings);
  const filter_weightDetect::WeightIdentification identification =
      filter_weightDetect::identifyWeight(readings, latestWeight);
  applyWeightIdentification(identification);
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

  applyWeightIdentification(identification);
  return latestFrameValid;
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
  setState(RobotState::sweepingArmDown);
}

void updateArm(uint32_t nowMs) {
  if (state == RobotState::holdingArmDown) {
    if (nowMs - stateStartedMs >= kArmCatchHoldTimeMs) {
      lastArmStepMs = nowMs;
      setState(RobotState::sweepingArmUp);
    }
    return;
  }

  if (nowMs - lastArmStepMs < kArmStepPeriodMs) {
    return;
  }
  lastArmStepMs = nowMs;

  if (state == RobotState::sweepingArmUp) {
    armAngleDegrees = max(
        armAngleDegrees - kArmStepDegrees, kArmUpAngleDegrees);
    hd_move_servoArm::setAngle(armAngleDegrees);
    if (armAngleDegrees <= kArmUpAngleDegrees) {
      clearFrames = 0;
      setState(RobotState::waitingForWeightToClear);
    }
    return;
  }

  if (state == RobotState::sweepingArmDown) {
    armAngleDegrees = min(
        armAngleDegrees + kArmStepDegrees, kArmDownAngleDegrees);
    hd_move_servoArm::setAngle(armAngleDegrees);
    if (armAngleDegrees >= kArmDownAngleDegrees) {
      setState(RobotState::holdingArmDown);
    }
  }
}

void ensureArmRaised(uint32_t nowMs) {
  const bool collecting =
      state == RobotState::sweepingArmDown ||
      state == RobotState::holdingArmDown ||
      state == RobotState::sweepingArmUp;
  if (collecting ||
      nowMs - lastArmGuardMs < kArmGuardPeriodMs ||
      !hd_move_servoArm::isInitialised()) {
    return;
  }

  lastArmGuardMs = nowMs;
  armAngleDegrees = kArmUpAngleDegrees;
  hd_move_servoArm::setAngle(armAngleDegrees);
}

float headingChangeDegrees(float currentDegrees, float previousDegrees) {
  float change = currentDegrees - previousDegrees;
  while (change > 180.0f) {
    change -= 360.0f;
  }
  while (change < -180.0f) {
    change += 360.0f;
  }
  return fabsf(change);
}

void beginWallAvoidance() {
  wallTurnTargetDegrees = topLeftWallClose && topRightWallClose
      ? kBothWallsTurnDegrees
      : kSingleWallTurnDegrees;
  wallTurnAccumulatedDegrees = 0.0f;
  haveWallTurnHeading = false;
  wallTurnUsesImu = false;
  wallTurnStartedMs = 0;
  lastWallImuMs = 0;
  stopMotors();
  setState(RobotState::stoppingForWall);

  Serial.print("[WALL] Top ");
  if (topLeftWallClose && topRightWallClose) {
    Serial.print("left and right");
  } else if (topLeftWallClose) {
    Serial.print("left");
  } else {
    Serial.print("right");
  }
  Serial.print(" ToF below 200mm; stopping, then turning right ");
  Serial.print(wallTurnTargetDegrees, 0);
  Serial.println(" degrees");

}

void finishWallAvoidance(uint32_t nowMs) {
  topLeftWallClose = false;
  topRightWallClose = false;
  closeWallAhead = false;
  detectionFrames = 0;
  lostFrames = 0;
  lastSensorMs = nowMs;
  setState(RobotState::searching);
  commandMotors(kLeftDrivePower, kRightDrivePower);
  Serial.println("[WALL] Turn complete; continuing forward");
}

void updateWallAvoidance(uint32_t nowMs) {
  if (state == RobotState::stoppingForWall) {
    stopMotors();
    if (nowMs - stateStartedMs < kWallStopTimeMs) {
      return;
    }

    hd_raw_imu::Reading reading;
    wallTurnUsesImu = imuAvailable && hd_raw_imu::read(reading) && reading.valid;
    if (wallTurnUsesImu) {
      previousWallTurnHeadingDegrees = reading.headingDegrees;
      haveWallTurnHeading = true;
      lastWallImuMs = nowMs;
    }
    wallTurnStartedMs = nowMs;
    setState(RobotState::turningFromWall);
    commandMotors(kWallPointTurnPower, -kWallPointTurnPower);
    return;
  }

  commandMotors(kWallPointTurnPower, -kWallPointTurnPower);

  if (wallTurnUsesImu &&
      nowMs - lastWallImuMs >= hd_raw_imu::kRecommendedSampleIntervalMs) {
    lastWallImuMs = nowMs;
    hd_raw_imu::Reading reading;
    if (hd_raw_imu::read(reading) && reading.valid) {
      if (haveWallTurnHeading) {
        wallTurnAccumulatedDegrees += headingChangeDegrees(
            reading.headingDegrees, previousWallTurnHeadingDegrees);
      }
      previousWallTurnHeadingDegrees = reading.headingDegrees;
      haveWallTurnHeading = true;
    }
  }

  const uint32_t timedTurnMs = static_cast<uint32_t>(
      wallTurnTargetDegrees * kFallbackTurnMsPerDegree);
  const uint32_t turnElapsedMs = nowMs - wallTurnStartedMs;
  const bool angleReached =
      wallTurnUsesImu &&
      wallTurnAccumulatedDegrees >= wallTurnTargetDegrees;
  const bool timedFallbackReached = !wallTurnUsesImu &&
      turnElapsedMs >= timedTurnMs;
  const bool safetyTimeoutReached =
      turnElapsedMs >= timedTurnMs + kWallTurnTimeoutExtraMs;

  if (angleReached || timedFallbackReached || safetyTimeoutReached) {
    finishWallAvoidance(nowMs);
  }
}

void updateSearching(uint32_t nowMs) {
  // A close top-sensor reading always wins over weight hunting so the claw
  // and front of the robot remain clear of a wall.
  if (closeWallAhead) {
    detectionFrames = 0;
    lostFrames = 0;
    beginWallAvoidance();
    return;
  }

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

  driveSearchPattern(nowMs);
}

void updateApproaching() {
  if (closeWallAhead) {
    detectionFrames = 0;
    lostFrames = 0;
    beginWallAvoidance();
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
  Serial.print(" wallTurn=");
  Serial.print(closeWallAhead ? "YES" : "NO");
  Serial.print(" topLeft=");
  if (topLeftDistanceMm > 0) {
    Serial.print(topLeftDistanceMm);
    Serial.print("mm");
  } else {
    Serial.print("INVALID");
  }
  Serial.print(" topRight=");
  if (topRightDistanceMm > 0) {
    Serial.print(topRightDistanceMm);
    Serial.print("mm");
  } else {
    Serial.print("INVALID");
  }
  if (state == RobotState::turningFromWall) {
    Serial.print(" wallTurnProgress=");
    Serial.print(wallTurnAccumulatedDegrees, 1);
    Serial.print('/');
    Serial.print(wallTurnTargetDegrees, 0);
    Serial.print("deg");
  }
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
      armAngleDegrees = kArmUpAngleDegrees;
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

  if (!hd_move_servoArm::initialise(kArmUpAngleDegrees)) {
    Serial.println("[INIT] Servo arm failed; robot stopped");
    setState(RobotState::emergencyStopped);
    return;
  }
  armAngleDegrees = kArmUpAngleDegrees;
  Serial.println("[INIT] Servo arm raised to 40 degrees");

  if (sensorSource == SensorSource::matrix8x8) {
    filter_weightDetect::clearBackgroundCalibration();
    Serial.println(
        "[INIT] 8x8 using calibration-free moving-scene weight detection");
  }

  if (sensorSource == SensorSource::matrix8x8) {
    // These sensors provide wall avoidance while the 8x8 remains the primary
    // weight detector.
    initialiseRangeTofs();
  }

  imuAvailable = hd_raw_imu::initialise();
  Serial.println(imuAvailable
      ? "[INIT] IMU angle-controlled wall turns ready"
      : "[INIT] IMU unavailable; wall turns will use timed fallback");

  lastSensorMs = millis() - kSensorPeriodMs;
  setState(RobotState::searching);
  commandMotors(kLeftDrivePower, kRightDrivePower);
  Serial.println("[ROBOT] Autonomous hunting started; moving forward");
}

void loop() {
  readCommands();
  const uint32_t nowMs = millis();
  ensureArmRaised(nowMs);

  if (state == RobotState::emergencyStopped) {
    stopMotors();
    printStatus(nowMs);
    return;
  }

  if (state == RobotState::stoppingForWall ||
      state == RobotState::turningFromWall) {
    updateWallAvoidance(nowMs);
    printStatus(nowMs);
    return;
  }

  if (state == RobotState::sweepingArmUp ||
      state == RobotState::holdingArmDown ||
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
    case RobotState::holdingArmDown:
    case RobotState::sweepingArmDown:
    case RobotState::stoppingForWall:
    case RobotState::turningFromWall:
    case RobotState::emergencyStopped:
      break;
  }

  printStatus(nowMs);
}
