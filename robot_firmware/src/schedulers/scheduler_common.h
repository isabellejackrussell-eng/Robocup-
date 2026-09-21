#pragma once

#include <Arduino.h>

namespace schedulers::common {

void initSensorSuite();
void initLocalisationSuite();
void serviceFastTasks();
void readSensorSuite();
void processPerception();
void readLocalisation();
void updateLocalisation();
void applyNavigationCommand();
void pollDebugCommands();
void printAllDebug(bool includeGrid = false);

}  // namespace schedulers::common

