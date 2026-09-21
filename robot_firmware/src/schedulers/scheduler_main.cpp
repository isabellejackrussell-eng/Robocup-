#include "schedulers/scheduler_main.h"

#include "debug/debug_port.h"
#include "schedulers/scheduler_full_integration_test.h"

namespace schedulers::main_scheduler {

void setup() {
  debug::Log.println(F("[MAIN] Competition orchestration enabled; unfinished sorting/storage remains disabled"));
  full_integration_test::setup();
}

void loop() {
  // Thin orchestration entry point. Detailed work remains in subsystem modules.
  full_integration_test::loop();
}

}  // namespace schedulers::main_scheduler

