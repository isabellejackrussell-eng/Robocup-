//************************************
//         motor_encoder.h    
//************************************
// Motors + their encoders. Also maintains a running (x, y, heading)
// estimate from wheel encoders alone (dead-reckoning odometry), which
// will later be combined with the optical flow and IMU estimates.

#ifndef MOTORS_H_
#define MOTORS_H_

// ============================================================
// PINS — CHANGE THESE TO MATCH YOUR WIRING
// ============================================================
#define LEFT_MOTOR_PIN         0   // D0
#define RIGHT_MOTOR_PIN        1   // D1

#define LEFT_ENCODER_PIN_A     2
#define LEFT_ENCODER_PIN_B     3
#define RIGHT_ENCODER_PIN_A    4
#define RIGHT_ENCODER_PIN_B    5

// ============================================================
// SELF-TEST CONFIG — used only by motors_test_forward_back()
// ============================================================
#define TEST_DRIVE_SPEED   450    // power used for the test, same scale as motors_write()
#define TEST_DRIVE_MS     2000    // how long to drive each direction
#define TEST_PAUSE_MS     1000    // pause between phases
#define TEST_PRINT_MS      200    // how often to print encoder counts during the test

// ============================================================
// MOTOR SIGNAL TUNING
// ============================================================
#define MOTOR_STOP_US           1500
#define MOTOR_FULL_FORWARD_US   1950
#define MOTOR_FULL_REVERSE_US   1050

#define MIN_SPEED_CAP  -450   // full reverse
#define MAX_SPEED_CAP   450   // full forward

#define LEFT_MOTOR_INVERTED    false
#define RIGHT_MOTOR_INVERTED   true

// ============================================================
// ROBOT GEOMETRY — NEEDED TO TURN ENCODER TICKS INTO REAL DISTANCE
// TODO: measure these once the chassis is finalised and calibrate
// TICKS_PER_MM by driving a known distance and counting ticks.
// ============================================================
#define WHEEL_TRACK_WIDTH_MM   150.0f   // distance between the two tracks' centre lines
#define TICKS_PER_MM              1.0f  // placeholder — MUST be calibrated

// ============================================================
// PUBLIC FUNCTIONS
// ============================================================

// Call once at startup. Attaches servos, arms ESCs, sets up encoder
// pins/interrupts, and zeroes the pose estimate.
void motors_init();

// Resets encoder counts and the (x, y, heading) estimate back to zero.
// Call this whenever you want to redefine "here" as the origin
// (e.g. at the start of a run).
void motors_zero();

// Call this regularly (e.g. every scheduler tick). Reads how far each
// wheel has turned since the last call and updates the pose estimate.
// This is the "2D transform" step: encoder ticks -> real-world position.
void motors_poll();

// Sends power values to the motors. Range: MIN_SPEED_CAP..MAX_SPEED_CAP.
void motors_write(int leftPower, int rightPower);

// test for motors and encoders 
void test_motors_encoders();

// Current odometry estimate, built purely from wheel encoders.
float motors_get_x_mm();
float motors_get_y_mm();
float motors_get_heading_deg();

// Raw tick counts, useful for debugging/self-test (easier to sanity
// check than the computed pose above).
long motors_get_left_ticks();
long motors_get_right_ticks();

// ============================================================
// SELF-TEST
// ============================================================
// Blocking test: drives forward, stops, reverses, stops — printing
// encoder counts throughout. Call this once from main.cpp (e.g. in
// setup(), or behind a debug switch) to sanity-check the motors and
// encoders are wired and responding correctly BEFORE trusting them
// inside real control logic.
void motors_test_forward_back();

#endif /* MOTORS_H_ */