#if 0
// Temporarily disabled while testing only the ToF sensors on the expander.
// Keep this code here so the robot test program can be restored later.
#include <Arduino.h>
#include <Wire.h>

#include "filter_weightDetect.h"
#include "hd_move_motors.h"
#include "hd_move_servoArm.h"
#include "hd_raw_limitSwitch.h"
#include "hd_raw_tof.h"
#include "hd_raw_tof8x8.h"
#include "logic_weightCollection.h"
#include "logic_weightHunting.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kReportPeriodMs = 500;
constexpr size_t kCalibrationFrameCount = 5;

bool weightSensorsReady = false;
uint32_t lastReportMs = 0;

void printInitialisationResult(const char* name, bool success) {
  Serial.print("[INIT] ");
  Serial.print(name);
  Serial.print(": ");
  Serial.println(success ? "OK" : "FAILED");
}

bool calibrateWeightSensor() {
  Serial.println(
      "[CALIBRATION] Keep the area in front of the 8x8 clear for 2 seconds");
  delay(2000);

  hd_raw_tof8x8::Frame frames[kCalibrationFrameCount];
  for (size_t frame = 0; frame < kCalibrationFrameCount; ++frame) {
    if (!hd_raw_tof8x8::readFrame(frames[frame])) {
      return false;
    }
    delay(50);
  }

  return filter_weightDetect::calibrateBackgroundAveraged(
      frames, kCalibrationFrameCount);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char command = Serial.read();
    if (command == 'w' || command == 'W') {
      logic_weightCollection::triggerCaptureTest();
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  while (!Serial && millis() < 3000) {
  }

  pinMode(kIoPowerPin, OUTPUT);
  digitalWrite(kIoPowerPin, HIGH);
  delay(500);
  Wire.begin();

  Serial.println("Robot small test program");
  Serial.println("Type w to run the weight-capture test");

  const bool tofReady = hd_raw_tof::initialise();
  const bool matrixReady = hd_raw_tof8x8::initialise();
  printInitialisationResult("six ToF sensors", tofReady);
  printInitialisationResult("8x8 ToF", matrixReady);

  filter_weightDetect::resetRangeDetections();
  weightSensorsReady =
      tofReady && matrixReady && calibrateWeightSensor();
  printInitialisationResult("weight sensor filter", weightSensorsReady);

  printInitialisationResult(
      "limit switch", hd_raw_limitSwitch::initialise());
  printInitialisationResult(
      "servo arm at 90 degrees", hd_move_servoArm::initialise(90));

  motors_init();
  printInitialisationResult("drive motors", motors_is_initialised());
  motors_test_movements();

  printInitialisationResult(
      "weight hunting state", logic_weightHunting::initialise());
  printInitialisationResult(
      "weight collection", logic_weightCollection::initialise());

  Serial.println("Tests ready. Type w to capture a weight.");
}

void loop() {
  readSerialCommands();
  logic_weightCollection::update();

  const uint32_t now = millis();
  if (!weightSensorsReady || now - lastReportMs < kReportPeriodMs) {
    return;
  }

  lastReportMs = now;
  Serial.println("----------------------------------------");
  const filter_weightDetect::WeightIdentification weight =
      filter_weightDetect::runWeightDetectionTest();
  logic_weightHunting::printTestDecision(weight);
  logic_weightCollection::printStatus();
}
#endif

#include <Arduino.h>

#include "hd_move_motors.h"
#include "hd_move_servoArm.h"
#include "hd_move_smartServos.h"
#include "hd_raw_imu.h"
#include "hd_raw_inductiveProximity.h"
#include "hd_raw_tof.h"
#include "hd_raw_tof8x8.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kPrintPeriodMs = 200;
constexpr uint32_t kSmartServoMovePeriodMs = 2000;
constexpr uint8_t kSmartServoIds[] = {1, 4};
constexpr float kSmartServoTestAngles[] = {-20.0f, 0.0f, 20.0f, 0.0f};

bool tofInitialised = false;
bool tof8x8Initialised = false;
bool imuInitialised = false;
uint32_t lastPrintMs = 0;
uint32_t lastSmartServoMoveMs = 0;
size_t nextSmartServoAngle = 0;

void testServoArm() {
  const bool servoReady = hd_move_servoArm::initialise(90);
  Serial.print("[SERVO ARM] Initialisation: ");
  Serial.println(servoReady ? "OK" : "FAILED");

  if (!servoReady) {
    return;
  }

  Serial.println("[SERVO ARM TEST] Sweeping from 40 to 170 degrees");
  for (int angle = 40; angle <= 170; angle += 5) {
    hd_move_servoArm::setAngle(angle);
    delay(50);
  }

  Serial.println("[SERVO ARM TEST] Sweeping from 170 to 40 degrees");
  for (int angle = 170; angle >= 40; angle -= 5) {
    hd_move_servoArm::setAngle(angle);
    delay(50);
  }

  hd_move_servoArm::setAngle(90);
  Serial.println("[SERVO ARM TEST] Complete; returned to 90 degrees");
}

void testDriveMotors() {
  motors_init();
  Serial.print("[MOTORS] Initialisation: ");
  Serial.println(motors_is_initialised() ? "OK" : "FAILED");

  if (motors_is_initialised()) {
    motors_test_movements();
    Serial.println("[MOTORS TEST] Complete; motors stopped");
  }
}

void printInitialisationResults() {
  Serial.print("[TOF] Overall initialisation: ");
  Serial.println(tofInitialised ? "OK" : "ONE OR MORE SENSORS FAILED");

  for (uint8_t sensor = 0; sensor < hd_raw_tof::kSensorCount; ++sensor) {
    Serial.print("[TOF] Sensor ");
    Serial.print(sensor);
    Serial.print(" (");
    Serial.print(hd_raw_tof::sensorModelName(sensor));
    Serial.print("): ");
    Serial.println(
        hd_raw_tof::isSensorInitialised(sensor) ? "OK" : "FAILED");
  }

  Serial.print("[TOF] 8x8 matrix sensor: ");
  Serial.println(tof8x8Initialised ? "OK" : "FAILED");

  Serial.print("[IMU] BNO055: ");
  Serial.println(imuInitialised ? "OK" : "FAILED");

  Serial.print("[SMART SERVO] ID 1: ");
  Serial.println(smartServoIsReady(1) ? "OK" : "FAILED");

  Serial.print("[SMART SERVO] ID 4: ");
  Serial.println(smartServoIsReady(4) ? "OK" : "FAILED");
}

void printReadings() {
  hd_raw_tof::Readings readings;
  hd_raw_tof::readAll(readings);

  for (uint8_t sensor = 0; sensor < hd_raw_tof::kSensorCount; ++sensor) {
    Serial.print("Sensor ");
    Serial.print(sensor);
    Serial.print(" (");
    Serial.print(hd_raw_tof::sensorModelName(sensor));
    Serial.print("): ");

    if (!hd_raw_tof::isSensorInitialised(sensor)) {
      Serial.print("NOT INITIALISED");
    } else if (readings[sensor].timedOut) {
      Serial.print("TIMEOUT");
    } else if (!readings[sensor].valid) {
      Serial.print("INVALID");
    } else {
      Serial.print(readings[sensor].distanceMm);
      Serial.print(" mm");
    }

    if (sensor + 1 < hd_raw_tof::kSensorCount) {
      Serial.print("    ");
    }
  }

  Serial.println();

  if (!tof8x8Initialised) {
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
  Serial.println(" mm");
}

void printImuReading() {
  if (!imuInitialised) {
    Serial.println("IMU: NOT INITIALISED");
    return;
  }

  hd_raw_imu::Reading reading;
  if (!hd_raw_imu::read(reading)) {
    Serial.println("IMU: READ FAILED");
    return;
  }

  Serial.print("IMU: heading=");
  Serial.print(reading.headingDegrees, 1);
  Serial.print(" roll=");
  Serial.print(reading.rollDegrees, 1);
  Serial.print(" pitch=");
  Serial.print(reading.pitchDegrees, 1);
  Serial.print(" deg  acceleration=");
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
  Serial.print(reading.calibration.magnetometer);
  Serial.print("  temperature=");
  Serial.print(reading.temperatureCelsius);
  Serial.println(" C");
}

void printInductiveProximityReading() {
  const int rawState = inductiveSensorRaw();

  Serial.print("Inductive proximity: metal=");
  Serial.print(inductiveSensorDetected() ? "YES" : "NO");
  Serial.print(" raw=");
  Serial.println(rawState == HIGH ? "HIGH" : "LOW");
}

void updateSmartServoMovement(uint32_t nowMs) {
  if (nowMs - lastSmartServoMoveMs < kSmartServoMovePeriodMs) {
    return;
  }

  lastSmartServoMoveMs = nowMs;
  const float angle = kSmartServoTestAngles[nextSmartServoAngle];

  Serial.print("[SMART SERVO TEST] Moving available servos to ");
  Serial.print(angle);
  Serial.println(" degrees");

  for (const uint8_t servoId : kSmartServoIds) {
    if (smartServoIsReady(servoId)) {
      setServoAngle(servoId, angle);
    }
  }

  nextSmartServoAngle =
      (nextSmartServoAngle + 1) %
      (sizeof(kSmartServoTestAngles) / sizeof(kSmartServoTestAngles[0]));
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  while (!Serial && millis() < 3000) {
  }

  Serial.println();
  Serial.println("Robot sensor, servo, and drive motor test");

  // Enable the CPU board's switched I/O supply before accessing the sensors.
  pinMode(kIoPowerPin, OUTPUT);
  digitalWrite(kIoPowerPin, HIGH);
  delay(500);

  tofInitialised = hd_raw_tof::initialise();
  tof8x8Initialised = hd_raw_tof8x8::initialise();
  imuInitialised = hd_raw_imu::initialise();
  inductiveSensorInitialise();
  smartServoInitialise();
  printInitialisationResults();

  testServoArm();
  testDriveMotors();

  lastSmartServoMoveMs = millis();
  Serial.println("Smart servos will cycle through -20, 0, +20, and 0 degrees");
  Serial.println("Starting ToF, IMU, and inductive proximity readings...");
}

void loop() {
  smartServoUpdate();

  const uint32_t now = millis();
  updateSmartServoMovement(now);
  if (now - lastPrintMs < kPrintPeriodMs) {
    return;
  }

  lastPrintMs = now;
  printReadings();
  printImuReading();
  printInductiveProximityReading();
}
