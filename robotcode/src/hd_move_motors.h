#pragma once

// Attaches the left and right motor signals to pins 0 and 1, sends the stop
// pulse, and waits two seconds for the motor controllers to arm.
void motors_init();

// Sets the left and right motor power. Each value is clamped to -450..450.
// Positive values drive forward, negative values reverse, and zero stops.
// The right motor direction is inverted to match the robot's wiring.
void motors_write(int leftPower, int rightPower);

// Stops both motors.
void motors_stop();

// Returns true after motors_init() has completed.
bool motors_is_initialised();

// Runs the same blocking forward/stop/reverse/stop pattern as the working
// sensor_tests/motor_test sketch, using a power of 250.
void motors_test_forward_back();
