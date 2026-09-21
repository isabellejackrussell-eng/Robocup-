#pragma once

#include <Arduino.h>

namespace config {

// A negative pin means "not wired/configured yet". Drivers remain compile-safe
// and report the affected sensor as unavailable.
constexpr int8_t PIN_UNASSIGNED = -1;

// Serial/debugging
constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t BLUETOOTH_SERIAL_BAUD = 115200;
constexpr uint32_t DEBUG_PRINT_PERIOD_MS = 500;

// Motors: copied from the working motor/encoder tests.
constexpr uint8_t LEFT_MOTOR_PIN = 0;
constexpr uint8_t RIGHT_MOTOR_PIN = 1;
constexpr int MOTOR_STOP_US = 1500;
constexpr int MOTOR_FULL_FORWARD_US = 1950;
constexpr int MOTOR_FULL_REVERSE_US = 1050;
constexpr int MOTOR_MAX_POWER = 450;
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = true;
constexpr int MOTOR_TEST_SPEED = 250;
constexpr int TURN_TEST_SPEED = 250;

// Claw: positions copied from the current servo test. TODO: tune on robot.
constexpr uint8_t CLAW_SERVO_PIN = 25;
constexpr uint8_t CLAW_RELEASE_ANGLE = 60;
constexpr uint8_t CLAW_GRAB_ANGLE = 120;
constexpr uint32_t CLAW_MOVE_TIME_MS = 1000;

// Encoder pins copied from the working encoder test.
constexpr uint8_t LEFT_ENCODER_A_PIN = 2;
constexpr uint8_t LEFT_ENCODER_B_PIN = 3;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 4;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 5;
// TODO: set after calibration. Zero deliberately disables distance conversion.
constexpr float LEFT_ENCODER_TICKS_PER_MM = 0.0f;
constexpr float RIGHT_ENCODER_TICKS_PER_MM = 0.0f;

// Optical flow: pin copied from the working PMW3901 test.
constexpr uint8_t OPTICAL_FLOW_CS_PIN = 10;
// TODO: calibrate counts/mm and axis direction at the final mounting height.
constexpr float OPTICAL_FLOW_COUNTS_PER_MM_X = 0.0f;
constexpr float OPTICAL_FLOW_COUNTS_PER_MM_Y = 0.0f;
constexpr int8_t OPTICAL_FLOW_X_SIGN = 1;
constexpr int8_t OPTICAL_FLOW_Y_SIGN = 1;

// IMU
constexpr uint8_t BNO055_I2C_ADDRESS = 0x28;
constexpr int32_t BNO055_SENSOR_ID = 55;

// SEN0628 / DFRobot 8x8 Matrix LiDAR
constexpr uint8_t TOF8X8_I2C_ADDRESS = 0x33;
constexpr uint8_t TOF8X8_GRID_SIZE = 8;
constexpr uint8_t TOF8X8_POINT_COUNT = 64;
constexpr float TOF8X8_HORIZONTAL_FOV_DEG = 60.0f;
// Change to -1 if the mounted image is horizontally mirrored.
constexpr int8_t TOF8X8_BEARING_SIGN = 1;
constexpr uint16_t WEIGHT_NEAR_DELTA_MM = 40;
constexpr uint8_t WEIGHT_MIN_CLUSTER_SIZE = 2;
constexpr uint16_t WEIGHT_FLATNESS_THRESHOLD_MM = 40;
constexpr float WEIGHT_DIAMETER_MM = 50.0f;
// TODO: tune usable detection bounds on the mounted robot.
constexpr float WEIGHT_MIN_DISTANCE_MM = 0.0f;
constexpr float WEIGHT_MAX_DISTANCE_MM = 4000.0f;

// SX1509 + four single-point ToFs. XSHUT mappings have not been selected.
constexpr uint8_t SX1509_I2C_ADDRESS = 0x3F;
enum RangeToFIndex : uint8_t {
  CROSS_LEFT_TOF = 0,
  CROSS_RIGHT_TOF = 1,
  CAPTURE_LEFT_TOF = 2,
  CAPTURE_RIGHT_TOF = 3,
  RANGE_TOF_COUNT = 4
};
constexpr int8_t RANGE_TOF_XSHUT_PINS[RANGE_TOF_COUNT] = {
    PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED};
constexpr uint8_t RANGE_TOF_I2C_ADDRESSES[RANGE_TOF_COUNT] = {
    0x2A, 0x2B, 0x2C, 0x2D};
constexpr uint16_t RANGE_TOF_TIMING_BUDGET_US = 50000;
constexpr uint16_t RANGE_TOF_PERIOD_MS = 50;
constexpr uint16_t SENSOR_STALE_TIMEOUT_MS = 500;

// Ultrasonic trigger/echo pins are deliberately disabled until wiring is chosen.
enum UltrasonicIndex : uint8_t {
  ULTRASONIC_LEFT = 0,
  ULTRASONIC_FRONT = 1,
  ULTRASONIC_RIGHT = 2,
  ULTRASONIC_COUNT = 3
};
constexpr int8_t ULTRASONIC_TRIGGER_PINS[ULTRASONIC_COUNT] = {
    PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED};
constexpr int8_t ULTRASONIC_ECHO_PINS[ULTRASONIC_COUNT] = {
    PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED};
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 25000;

// Perception thresholds are initial safe placeholders. TODO: tune with logs.
constexpr float WALL_FILTER_ALPHA = 0.35f;
constexpr float WALL_MIN_VALID_MM = 20.0f;
constexpr float WALL_MAX_VALID_MM = 4000.0f;
constexpr float WALL_WEIGHT_SEPARATION_MARGIN_MM = 100.0f;
constexpr float WALL_CORNER_DIFFERENCE_MM = 150.0f;
constexpr float CAPTURE_DISTANCE_THRESHOLD_MM = 100.0f;
constexpr bool CAPTURE_REQUIRES_BOTH_SENSORS = true;

// Navigation placeholders. All remain centrally tuneable.
constexpr float OBSTACLE_STOP_DISTANCE_MM = 180.0f;
constexpr float APPROACH_STOP_DISTANCE_MM = 90.0f;
constexpr float SEARCH_SPEED = 110.0f;
constexpr float APPROACH_SPEED = 140.0f;
constexpr float AVOID_SPEED = 130.0f;
constexpr float HOME_SPEED = 140.0f;
constexpr float BEARING_STEERING_GAIN = 2.0f;
constexpr float HOME_ARRIVAL_DISTANCE_MM = 100.0f;

// Scheduler periods
constexpr uint32_t SENSOR_READ_PERIOD_MS = 50;
constexpr uint32_t PERCEPTION_UPDATE_PERIOD_MS = 100;
constexpr uint32_t LOCALISATION_UPDATE_PERIOD_MS = 20;
constexpr uint32_t NAV_UPDATE_PERIOD_MS = 50;
constexpr uint32_t MOTION_TEST_ACTION_MS = 2000;
constexpr uint32_t MOTION_TEST_STOP_MS = 1000;
constexpr uint32_t CLAW_TEST_PAUSE_MS = 1000;

}  // namespace config
