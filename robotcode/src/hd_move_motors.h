#pragma once

// Attaches the left and right motor signals to pins 0 and 1, sends the stop
// pulse, and waits two seconds for the motor controllers to arm.
void motors_init();

// Sets the left and right motor power. Each value is clamped to -450..450.
// Positive values drive forward, negative values reverse, and zero stops.
// The per-track direction settings account for the robot's motor mounting.
void motors_write(int leftPower, int rightPower);

// Stops both motors.
void motors_stop();

// Returns true after motors_init() has completed.
bool motors_is_initialised();

// Runs a blocking forward, backward, point-turn-right, and point-turn-left test.
// Both motors are stopped for one second between movements.
void motors_test_movements();
