#include "claw/claw.h"

#include <Servo.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"

namespace claw {
namespace {
Servo servo;
uint8_t position = config::CLAW_RELEASE_ANGLE;
uint32_t motionStartedMs = 0;
bool moving = false;
}  // namespace

void initClaw() {
  servo.attach(config::CLAW_SERVO_PIN);
  position = config::CLAW_RELEASE_ANGLE;
  servo.write(position);
  motionStartedMs = millis();
  moving = true;
  debug::Log.print(F("[CLAW] Initialised at release angle "));
  debug::Log.println(position);
}

void setClawPosition(uint8_t angle) {
  position = constrain(angle, 0, 180);
  servo.write(position);
  motionStartedMs = millis();
  moving = true;
}

void grab() {
  if (moving && position == config::CLAW_GRAB_ANGLE) return;
  debug::Log.print(F("[CLAW] GRAB -> angle "));
  debug::Log.println(config::CLAW_GRAB_ANGLE);
  setClawPosition(config::CLAW_GRAB_ANGLE);
}

void release() {
  if (moving && position == config::CLAW_RELEASE_ANGLE) return;
  debug::Log.print(F("[CLAW] RELEASE -> angle "));
  debug::Log.println(config::CLAW_RELEASE_ANGLE);
  setClawPosition(config::CLAW_RELEASE_ANGLE);
}

void update() {
  if (moving && millis() - motionStartedMs >= config::CLAW_MOVE_TIME_MS) {
    moving = false;
    debug::Log.println(F("[CLAW] Motion complete"));
  }
}

bool clawBusy() {
  update();
  return moving;
}

bool clawFinished() { return !clawBusy(); }
uint8_t getClawPosition() { return position; }

}  // namespace claw

