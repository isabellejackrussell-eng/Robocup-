#pragma once
#include <Arduino.h>

// Change these if your wiring changes.
constexpr uint8_t LEFT_MOTOR_PIN  = 0;
constexpr uint8_t RIGHT_MOTOR_PIN = 1;
constexpr uint8_t LEFT_ENCODER_PIN_A  = 2;
constexpr uint8_t LEFT_ENCODER_PIN_B  = 3;
constexpr uint8_t RIGHT_ENCODER_PIN_A = 4;
constexpr uint8_t RIGHT_ENCODER_PIN_B = 5;

// RC-style motor controller pulses
constexpr int MOTOR_STOP_US         = 1500;
constexpr int MOTOR_FULL_FORWARD_US = 1950;

// The right motor is physically mounted in the opposite direction,
// so it needs its command inverted.
constexpr bool LEFT_MOTOR_INVERTED  = false;
constexpr bool RIGHT_MOTOR_INVERTED = true;


// Change one of these if its encoder counts backwards when the corresponding motor drives forward.
constexpr bool LEFT_ENCODER_INVERTED  = false;
constexpr bool RIGHT_ENCODER_INVERTED = false;

// Call once during setup().
void motors_init();

// Call regularly from loop() / scheduler. Updates measured encoder speeds.
void motors_update();

// Set left motor speed. Range: 0 = stopped: 100 = full speed
void motors_set_left_speed(uint8_t percent);

// Set right motor speed. Range: 0 = stopped: 100 = full speed
void motors_set_right_speed(uint8_t percent);

// Convenience function for setting both at once.
void motors_set_speed(uint8_t leftPercent, uint8_t rightPercent);

// Stop both motors.
void motors_stop();

// Returns the currently requested motor speed, 0-100%.
uint8_t motors_get_left_command();
uint8_t motors_get_right_command();

// Raw accumulated encoder ticks.
long motors_get_left_ticks();
long motors_get_right_ticks();

// Encoder speed measured over time.
// Units: ticks / second
float motors_get_left_ticks_per_second();
float motors_get_right_ticks_per_second();

// Reset encoder positions back to zero.
void motors_reset_encoders();