#include <Arduino.h>

#include "debug/debug_port.h"
#include "schedulers/scheduler_claw_test.h"
#include "schedulers/scheduler_full_integration_test.h"
#include "schedulers/scheduler_main.h"
#include "schedulers/scheduler_motion_test.h"
#include "schedulers/scheduler_nav_motion_test.h"
#include "schedulers/scheduler_sensor_claw_test.h"
#include "schedulers/scheduler_sensor_test.h"

#ifndef ROBOT_MODE
#define ROBOT_MODE 3
#endif

namespace {
const __FlashStringHelper* modeName() {
#if ROBOT_MODE == 1
  return F("MOTION_TEST");
#elif ROBOT_MODE == 2
  return F("CLAW_TEST");
#elif ROBOT_MODE == 3
  return F("SENSOR_TEST (default safe mode)");
#elif ROBOT_MODE == 4
  return F("SENSOR_CLAW_TEST");
#elif ROBOT_MODE == 5
  return F("NAV_MOTION_TEST");
#elif ROBOT_MODE == 6
  return F("FULL_INTEGRATION_TEST");
#elif ROBOT_MODE == 7
  return F("COMPETITION");
#else
#error "Unknown ROBOT_MODE"
#endif
}
}  // namespace

void setup() {
  debug::begin();
  debug::printStartupBanner(modeName());
#if ROBOT_MODE == 1
  schedulers::motion_test::setup();
#elif ROBOT_MODE == 2
  schedulers::claw_test::setup();
#elif ROBOT_MODE == 3
  schedulers::sensor_test::setup();
#elif ROBOT_MODE == 4
  schedulers::sensor_claw_test::setup();
#elif ROBOT_MODE == 5
  schedulers::nav_motion_test::setup();
#elif ROBOT_MODE == 6
  schedulers::full_integration_test::setup();
#elif ROBOT_MODE == 7
  schedulers::main_scheduler::setup();
#endif
}

void loop() {
#if ROBOT_MODE == 1
  schedulers::motion_test::loop();
#elif ROBOT_MODE == 2
  schedulers::claw_test::loop();
#elif ROBOT_MODE == 3
  schedulers::sensor_test::loop();
#elif ROBOT_MODE == 4
  schedulers::sensor_claw_test::loop();
#elif ROBOT_MODE == 5
  schedulers::nav_motion_test::loop();
#elif ROBOT_MODE == 6
  schedulers::full_integration_test::loop();
#elif ROBOT_MODE == 7
  schedulers::main_scheduler::loop();
#endif
}
