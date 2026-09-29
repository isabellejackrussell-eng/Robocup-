#include "hd_move_motors.h"

#include <Arduino.h>
#include <Servo.h>

namespace {

// Motor wiring used by the working sensor_tests/motor_test sketch.
constexpr int kLeftPin = 0;
constexpr int kRightPin = 1;

// Pulse widths for the continuous-rotation servos / ESCs.
constexpr int kStopUs = 1500;
constexpr int kFullForwardUs = 1950;
constexpr int kFullReverseUs = 1050;

constexpr int kMaxPower = 450;

// The motors face opposite directions on the chassis. These values make a
// positive power command move both tracks physically forward.
constexpr bool kLeftInverted = true;
constexpr bool kRightInverted = false;

Servo motorLeft;
Servo motorRight;
bool initialised = false;

int makeMotorPulse(int motorPower) {
  motorPower = constrain(motorPower, -kMaxPower, kMaxPower);

  if (motorPower == 0) {
    return kStopUs;
  }

  if (motorPower > 0) {
    return map(motorPower, 0, kMaxPower, kStopUs, kFullForwardUs);
  }

  return map(motorPower, 0, -kMaxPower, kStopUs, kFullReverseUs);
}

}  // namespace

void motors_init() {
  motorLeft.attach(kLeftPin);
  motorRight.attach(kRightPin);

  // Some ESCs need to receive the stop pulse for a moment before they arm.
  motors_write(0, 0);
  delay(2000);

  initialised = true;
  Serial.println("[MOTORS] Initialised");
}

void motors_write(int leftPower, int rightPower) {
  if (kLeftInverted) {
    leftPower = -leftPower;
  }
  if (kRightInverted) {
    rightPower = -rightPower;
  }

  motorLeft.writeMicroseconds(makeMotorPulse(leftPower));
  motorRight.writeMicroseconds(makeMotorPulse(rightPower));
}

void motors_stop() {
  motors_write(0, 0);
}

bool motors_is_initialised() {
  return initialised;
}

void motors_test_movements() {
  if (!initialised) {
    motors_init();
  }

  Serial.println("[MOTORS TEST] Forward");
  motors_write(250, 250);
  delay(2000);

  Serial.println("[MOTORS TEST] Stop");
  motors_stop();
  delay(1000);

  Serial.println("[MOTORS TEST] Reverse");
  motors_write(-250, -250);
  delay(2000);

  Serial.println("[MOTORS TEST] Stop");
  motors_stop();
  delay(1000);

  Serial.println("[MOTORS TEST] Point turn right");
  motors_write(250, -250);
  delay(2000);

  Serial.println("[MOTORS TEST] Stop");
  motors_stop();
  delay(1000);

  Serial.println("[MOTORS TEST] Point turn left");
  motors_write(-250, 250);
  delay(2000);

  Serial.println("[MOTORS TEST] Stop");
  motors_stop();
  delay(1000);
}
