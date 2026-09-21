#include "localisation/encoders.h"

#include <Arduino.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"

namespace localisation::encoders {
namespace {
volatile long leftPosition = 0;
volatile long rightPosition = 0;
bool leftA = false, leftB = false, rightA = false, rightB = false;
long leftSnapshot = 0;
long rightSnapshot = 0;

void onLeftA() {
  leftA = digitalRead(config::LEFT_ENCODER_A_PIN) == HIGH;
  leftPosition += (leftA != leftB) ? 1 : -1;
  leftB = digitalRead(config::LEFT_ENCODER_B_PIN) == HIGH;
  leftPosition += (leftA == leftB) ? 1 : -1;
}

void onRightA() {
  rightA = digitalRead(config::RIGHT_ENCODER_A_PIN) == HIGH;
  rightPosition += (rightA != rightB) ? 1 : -1;
  rightB = digitalRead(config::RIGHT_ENCODER_B_PIN) == HIGH;
  rightPosition += (rightA == rightB) ? 1 : -1;
}
}  // namespace

void initEncoders() {
  pinMode(config::LEFT_ENCODER_A_PIN, INPUT);
  pinMode(config::LEFT_ENCODER_B_PIN, INPUT);
  pinMode(config::RIGHT_ENCODER_A_PIN, INPUT);
  pinMode(config::RIGHT_ENCODER_B_PIN, INPUT);
  leftA = digitalRead(config::LEFT_ENCODER_A_PIN) == HIGH;
  leftB = digitalRead(config::LEFT_ENCODER_B_PIN) == HIGH;
  rightA = digitalRead(config::RIGHT_ENCODER_A_PIN) == HIGH;
  rightB = digitalRead(config::RIGHT_ENCODER_B_PIN) == HIGH;
  attachInterrupt(digitalPinToInterrupt(config::LEFT_ENCODER_A_PIN), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(config::RIGHT_ENCODER_A_PIN), onRightA, CHANGE);
  debug::Log.println(F("[ENCODER] Ready; ticks/mm calibration still required"));
}

void updateEncoders() {
  noInterrupts();
  leftSnapshot = leftPosition;
  rightSnapshot = rightPosition;
  interrupts();
}

long getLeftTicks() { return leftSnapshot; }
long getRightTicks() { return rightSnapshot; }
float getLeftDistance() {
  return config::LEFT_ENCODER_TICKS_PER_MM > 0.0f
             ? leftSnapshot / config::LEFT_ENCODER_TICKS_PER_MM
             : NAN;
}
float getRightDistance() {
  return config::RIGHT_ENCODER_TICKS_PER_MM > 0.0f
             ? rightSnapshot / config::RIGHT_ENCODER_TICKS_PER_MM
             : NAN;
}
bool calibrated() {
  return config::LEFT_ENCODER_TICKS_PER_MM > 0.0f &&
         config::RIGHT_ENCODER_TICKS_PER_MM > 0.0f;
}

}  // namespace localisation::encoders

