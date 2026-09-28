#include <Arduino.h>
#include <Wire.h>

#include "filter_weightDetect.h"
#include "hd_move_motors.h"
#include "hd_move_servoArm.h"
#include "hd_move_smartServos.h"
#include "hd_raw_inductiveProximity.h"
#include "hd_raw_limitSwitch.h"
#include "hd_raw_tof.h"
#include "hd_raw_tof8x8.h"
#include "hd_raw_ultrasonic.h"
#include "hd_raw_xy.h"

// These modules are intentionally disabled until they have implementations.
// #include "filter_positionData.h"
// #include "hd_raw_encoder.h"

namespace {

constexpr uint8_t kIoPowerPin = 49;
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 3000;
constexpr uint32_t kReportPeriodMs = 500;

bool tof8x8Ready = false;
bool xyReady = false;
bool limitSwitchReady = false;
bool servoArmReady = false;
uint32_t lastReportMs = 0;

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

void initialiseSensors() {
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

  // filter_positionData is not initialised yet because its files are empty.
  // hd_raw_encoder is not initialised yet because its files are empty.
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

  Serial.print("Limit switch: ");
  Serial.print(limitRead ? (limitReading.pressed ? "PRESSED" : "OPEN")
                         : "READ FAILED");
  Serial.print("  Inductive: ");
  Serial.print(inductiveSensorDetected() ? "DETECTED" : "CLEAR");
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

void reportImplementedHardware() {
  Serial.println("----------------------------------------");
  printTofReadings();
  printTof8x8Reading();
  printUltrasonicReadings();
  printDigitalSensorReadings();
  printXyReading();
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
  Wire.begin();
  initialiseSensors();
  testServoArm();
  initialiseSmartServos();
  testMotors();

  // The task scheduler remains disabled until its sensor, navigation,
  // collection, unloading, watchdog, and motor callbacks are implemented.
  // task_init();

  Serial.println("========================================");
  Serial.println("Initialisation complete; starting live readings");
}

void loop() {
  smartServoUpdate();

  const uint32_t now = millis();
  if (now - lastReportMs < kReportPeriodMs) {
    return;
  }

  lastReportMs = now;
  reportImplementedHardware();

  // taskManager.execute();  // Enable when all scheduled callbacks exist.
}
