#include <Arduino.h>
#include <Wire.h>

#include "filter_weightDetect.h"
#include "hd_move_motors.h"
#include "hd_move_servoArm.h"
#include "hd_move_smartServos.h"
#include "hd_raw_bluetooth.h"
#include "hd_raw_imu.h"
#include "hd_raw_inductiveProximity.h"
#include "hd_raw_limitSwitch.h"
#include "hd_raw_tof.h"
#include "hd_raw_tof8x8.h"
#include "hd_raw_ultrasonic.h"
#include "hd_raw_xy.h"
#include "logic_weightCollection.h"
#include "logic_weightHunting.h"
#include "logic_weightHandling.h"

// Encoder collection is not started here because its test pins 2/3 and 4/5
// currently conflict with the integrated ultrasonic sensor pins.

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 3000;
constexpr uint32_t kReportPeriodMs = 500;
constexpr uint32_t kWeightControlPeriodMs = 100;
constexpr size_t kWeightCalibrationFrameCount = 5;

bool tof8x8Ready = false;
bool xyReady = false;
bool imuReady = false;
bool limitSwitchReady = false;
bool servoArmReady = false;
bool weightDetectorReady = false;
bool weightHuntingReady = false;
uint32_t lastReportMs = 0;
uint32_t lastWeightControlMs = 0;
filter_weightDetect::WeightIdentification latestWeight = {
    filter_weightDetect::WeightState::notFound,
    0,
    0,
    0,
    0,
    -1.0f,
    false,
    false,
    false,
};

void printInitialisationResult(const char* device, bool success) {
  Serial.print("[INIT] ");
  Serial.print(device);
  Serial.print(": ");
  Serial.println(success ? "OK" : "FAILED");
}

void initialiseIoPower() {
  pinMode(kIoPowerPin, OUTPUT);
  digitalWrite(kIoPowerPin, HIGH);
  delay(500);
  printInitialisationResult("CPU-board I/O power", true);
}

void initialiseBluetooth() {
  const bool bluetoothReady = hd_raw_bluetooth::initialise();
  printInitialisationResult("Bluetooth UART", bluetoothReady);

  if (bluetoothReady) {
    hd_raw_bluetooth::sendLine("Robot Bluetooth ready");
  }
}

bool calibrateWeightDetector() {
  if (!tof8x8Ready) {
    return false;
  }

  Serial.println(
      "[CALIBRATION] Keep weights clear of the 8x8 sensor for 2 seconds");
  delay(2000);

  hd_raw_tof8x8::Frame frames[kWeightCalibrationFrameCount];
  for (size_t frameIndex = 0;
       frameIndex < kWeightCalibrationFrameCount;
       ++frameIndex) {
    if (!hd_raw_tof8x8::readFrame(frames[frameIndex])) {
      Serial.println("[CALIBRATION] 8x8 frame read failed");
      return false;
    }
    delay(50);
  }

  return filter_weightDetect::calibrateBackgroundAveraged(
      frames, kWeightCalibrationFrameCount);
}

void initialiseSensors() {
  imuReady = hd_raw_imu::initialise();
  printInitialisationResult("BNO055 IMU", imuReady);

  const bool tofReady = hd_raw_tof::initialise();
  printInitialisationResult("six ToF sensors", tofReady);

  for (uint8_t sensor = 0; sensor < hd_raw_tof::kSensorCount; ++sensor) {
    Serial.print("  ToF ");
    Serial.print(sensor);
    Serial.print(" (");
    Serial.print(hd_raw_tof::sensorModelName(sensor));
    Serial.print("): ");
    Serial.println(
        hd_raw_tof::isSensorInitialised(sensor) ? "OK" : "FAILED");
  }

  tof8x8Ready = hd_raw_tof8x8::initialise();
  printInitialisationResult("8x8 ToF", tof8x8Ready);

  hd_raw_ultrasonic::initialise();
  printInitialisationResult("ultrasonic sensors", true);

  xyReady = hd_raw_xy::initialise();
  printInitialisationResult("XY optical-flow sensor", xyReady);

  limitSwitchReady = hd_raw_limitSwitch::initialise();
  printInitialisationResult("limit switch", limitSwitchReady);

  inductiveSensorInitialise();
  printInitialisationResult("inductive proximity sensor", true);

  filter_weightDetect::resetRangeDetections();
  printInitialisationResult("ToF weight/wall filter", true);

  weightDetectorReady = calibrateWeightDetector();
  printInitialisationResult(
      "8x8 empty-scene weight calibration", weightDetectorReady);

  // filter_positionData::collect() is ready to use once the encoder pin
  // conflict is resolved and hd_raw_encoder::initialise() can be called.
}

void testServoArm() {
  servoArmReady = hd_move_servoArm::initialise();
  printInitialisationResult("servo arm", servoArmReady);

  if (!servoArmReady) {
    return;
  }

  Serial.println("[TEST] Sweeping servo arm from 40 to 170 degrees");
  hd_move_servoArm::setAngle(40);
  delay(500);

  for (int angle = 40; angle <= 170; angle += 5) {
    hd_move_servoArm::setAngle(angle);
    delay(50);
  }

  for (int angle = 170; angle >= 40; angle -= 5) {
    hd_move_servoArm::setAngle(angle);
    delay(50);
  }

  hd_move_servoArm::setAngle(hd_move_servoArm::kInitialAngleDegrees);
  Serial.println("[TEST] Servo arm returned to 90 degrees");
}

void initialiseSmartServos() {
  smartServoInitialise();

  const bool servo1Ready = smartServoIsReady(1);
  const bool servo4Ready = smartServoIsReady(4);
  printInitialisationResult("smart servo 1", servo1Ready);
  printInitialisationResult("smart servo 4", servo4Ready);

  if (servo1Ready) {
    setServoAngle(1, 0.0f);
    smartServoPrintStatus(1);
  }

  if (servo4Ready) {
    setServoAngle(4, 0.0f);
    smartServoPrintStatus(4);
  }
}

void testMotors() {
  motors_init();
  printInitialisationResult("drive motors", motors_is_initialised());

  Serial.println("[TEST] Running motor forward/reverse test");
  motors_test_forward_back();
  Serial.println("[TEST] Motor test complete; motors stopped");
}

void updateWeightHunting(uint32_t nowMs) {
  if (!weightHuntingReady || !weightDetectorReady ||
      nowMs - lastWeightControlMs < kWeightControlPeriodMs) {
    return;
  }
  lastWeightControlMs = nowMs;

  hd_raw_tof::Readings readings;
  hd_raw_tof::readAll(readings);
  filter_weightDetect::updateRangeDetections(readings);

  hd_raw_tof8x8::Frame frame;
  if (!hd_raw_tof8x8::readFrame(frame)) {
    latestWeight = {
        filter_weightDetect::WeightState::notFound,
        0,
        0,
        0,
        0,
        -1.0f,
        false,
        false,
        false,
    };
    logic_weightHunting::stop();
    return;
  }

  const filter_weightDetect::WeightResult matrixWeight =
      filter_weightDetect::detectWeight(frame, false);
  latestWeight =
      filter_weightDetect::identifyWeight(readings, matrixWeight);
  logic_weightHunting::update(latestWeight, nowMs);
}

void printTofReadings() {
  hd_raw_tof::Readings readings;
  hd_raw_tof::readAll(readings);
  filter_weightDetect::updateRangeDetections(readings);

  Serial.print("ToF: ");
  for (uint8_t sensor = 0; sensor < hd_raw_tof::kSensorCount; ++sensor) {
    Serial.print(sensor);
    Serial.print('=');

    if (readings[sensor].valid) {
      Serial.print(readings[sensor].distanceMm);
      Serial.print("mm");
    } else if (readings[sensor].timedOut) {
      Serial.print("TIMEOUT");
    } else {
      Serial.print("INVALID");
    }

    if (sensor + 1 < hd_raw_tof::kSensorCount) {
      Serial.print("  ");
    }
  }
  Serial.println();

  filter_weightDetect::printRangeDetections();
}

void printTof8x8Reading() {
  if (!tof8x8Ready) {
    Serial.println("8x8 ToF: NOT INITIALISED");
    return;
  }

  hd_raw_tof8x8::Frame frame;
  if (!hd_raw_tof8x8::readFrame(frame)) {
    Serial.println("8x8 ToF: READ FAILED");
    return;
  }

  constexpr size_t kCentrePoint =
      (hd_raw_tof8x8::kGridSize / 2) * hd_raw_tof8x8::kGridSize +
      (hd_raw_tof8x8::kGridSize / 2);
  Serial.print("8x8 ToF centre: ");
  Serial.print(frame[kCentrePoint]);
  Serial.println("mm");
}

void printUltrasonicReadings() {
  hd_raw_ultrasonic::Readings readings;
  hd_raw_ultrasonic::readAll(readings);

  Serial.print("Ultrasonic: ");
  for (uint8_t sensor = 0;
       sensor < hd_raw_ultrasonic::kSensorCount;
       ++sensor) {
    Serial.print(sensor == 0 ? "A=" : "B=");
    if (readings[sensor].valid) {
      Serial.print(readings[sensor].distanceCm);
      Serial.print("cm");
    } else {
      Serial.print("TIMEOUT");
    }

    if (sensor + 1 < hd_raw_ultrasonic::kSensorCount) {
      Serial.print("  ");
    }
  }
  Serial.println();
}

void printDigitalSensorReadings() {
  hd_raw_limitSwitch::Reading limitReading;
  const bool limitRead =
      limitSwitchReady && hd_raw_limitSwitch::read(limitReading);
  const bool metalDetected = inductiveSensorDetected();

  Serial.print("Limit switch: ");
  Serial.print(limitRead ? (limitReading.pressed ? "PRESSED" : "OPEN")
                         : "READ FAILED");
  Serial.print("  Metal detected: ");
  Serial.print(metalDetected ? "YES" : "NO");
  Serial.print(" (raw=");
  Serial.print(inductiveSensorRaw());
  Serial.println(')');
}

void printXyReading() {
  hd_raw_xy::MotionReading reading;
  if (!xyReady || !hd_raw_xy::readMotion(reading)) {
    Serial.println("XY optical flow: READ FAILED");
    return;
  }

  Serial.print("XY optical flow: dx=");
  Serial.print(reading.deltaXCounts);
  Serial.print(" dy=");
  Serial.println(reading.deltaYCounts);
}

void printImuReading() {
  hd_raw_imu::Reading reading;
  if (!imuReady || !hd_raw_imu::read(reading)) {
    Serial.println("IMU: READ FAILED");
    return;
  }

  Serial.print("IMU: heading=");
  Serial.print(reading.headingDegrees, 1);
  Serial.print(" roll=");
  Serial.print(reading.rollDegrees, 1);
  Serial.print(" pitch=");
  Serial.print(reading.pitchDegrees, 1);
  Serial.print(" deg  linear acceleration=");
  Serial.print(reading.linearAccelerationMps2.x, 2);
  Serial.print(',');
  Serial.print(reading.linearAccelerationMps2.y, 2);
  Serial.print(',');
  Serial.print(reading.linearAccelerationMps2.z, 2);
  Serial.print(" m/s^2  calibration=");
  Serial.print(reading.calibration.system);
  Serial.print('/');
  Serial.print(reading.calibration.gyroscope);
  Serial.print('/');
  Serial.print(reading.calibration.accelerometer);
  Serial.print('/');
  Serial.println(reading.calibration.magnetometer);
}

void reportImplementedHardware() {
  Serial.println("----------------------------------------");
  printTofReadings();
  printTof8x8Reading();
  printUltrasonicReadings();
  printDigitalSensorReadings();
  printXyReading();
  printImuReading();
  filter_weightDetect::printWeightIdentification(latestWeight);
  logic_weightHunting::printStatus();
  logic_weightCollection::printStatus();
  Serial.print("Servo arm commanded angle: ");
  Serial.print(hd_move_servoArm::getCommandedAngle());
  Serial.println(" degrees");
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);

  const uint32_t serialWaitStartedAt = millis();
  while (!Serial && millis() - serialWaitStartedAt < kSerialWaitMs) {
  }

  Serial.println();
  Serial.println("Robot firmware integration check");
  Serial.println("========================================");

  initialiseIoPower();
  initialiseBluetooth();
  Wire.begin();
  initialiseSensors();
  testServoArm();
  initialiseSmartServos();
  printInitialisationResult(
      "weight handling logic", logic_weightHandling::initialise());
  testMotors();
  weightHuntingReady = logic_weightHunting::initialise();
  printInitialisationResult(
      "weight hunting logic", weightHuntingReady);
  printInitialisationResult(
      "weight collection logic", logic_weightCollection::initialise());

  // The task scheduler remains disabled until its sensor, navigation,
  // collection, unloading, watchdog, and motor callbacks are implemented.
  // task_init();

  Serial.println("========================================");
  Serial.println("Initialisation complete; starting live readings");
}

void loop() {
  smartServoUpdate();
  logic_weightHandling::update();

  const uint32_t now = millis();
  updateWeightHunting(now);
  logic_weightCollection::update(now);
  if (now - lastReportMs < kReportPeriodMs) {
    return;
  }

  lastReportMs = now;
  reportImplementedHardware();

  // taskManager.execute();  // Enable when all scheduled callbacks exist.
}
