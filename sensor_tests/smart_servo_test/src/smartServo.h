#pragma once

#include <Arduino.h>

// ============================================================
// SMART SERVO PUBLIC INTERFACE
// ============================================================

// Starts Serial7, checks servos 1 and 4, and enables torque.
//
// Call once from setup(), after IO power has been enabled.
void smartServoInitialise();


// Call regularly from loop().
// Keeps the Herkulex bus serviced.
void smartServoUpdate();


// Set a servo angle.
//
// Supported IDs:
//   1
//   4
//
// Angle range:
//   -150 to +150 degrees
//
// 0 degrees corresponds to the Herkulex centre position (512).
//
// Returns true if the servo ID is valid and was detected during
// initialisation.
bool setServoAngle(uint8_t servoID, float angle);


// Read the current measured position of a servo in degrees.
//
// Returns NAN if the servo ID is invalid/not available.
float getServoAngle(uint8_t servoID);


// Returns the current raw Herkulex position, approximately 0..1023.
//
// Returns 0 if the servo ID is invalid/not available.
uint16_t getServoPosition(uint8_t servoID);


// Returns true if the servo responded during initialisation.
bool smartServoIsReady(uint8_t servoID);


// Print the current servo position and Herkulex status bits.
// Useful for debugging.
void smartServoPrintStatus(uint8_t servoID);
