#pragma once

#include <Arduino.h>

// Initialise the inductive proximity sensor connected to A6Z.
void inductiveSensorInitialise();

// Returns true when metal is detected.
bool inductiveSensorDetected();

// Returns the raw digital state (HIGH or LOW).
// Useful for debugging.
int inductiveSensorRaw();