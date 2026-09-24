//************************************
//         optical_flow.h    
//************************************
// PMW3901 optical flow sensor. Measures how far the floor has moved
// underneath it (translation only — it cannot sense rotation).
// This file corrects the raw reading for the sensor's own mounting
// (angle + offset from the robot's turning centre) and outputs a
// clean "forward / right" delta in the ROBOT's own frame.
//
// NOTE: turning that into a position in the arena requires a heading
// estimate (from motor_encoder.cpp or imu.cpp) — that combining step
// happens in the fusion/localisation code, not in here.

#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

// ============================================================
// PINS — CHANGE THESE TO MATCH YOUR WIRING
// ============================================================
#define OPTICAL_FLOW_CS_PIN   10   // SPI chip-select pin

// ============================================================
// CALIBRATION — TODO: measure by sliding the robot a known
// distance and comparing to the raw counts reported.
// Using 0.0 as the "not calibrated yet" default is deliberate:
// it means every reading converts to 0mm until you set a real
// value, rather than crashing on a divide-by-zero.
// ============================================================
#define OPTICAL_FLOW_MM_PER_COUNT_X   0.0f
#define OPTICAL_FLOW_MM_PER_COUNT_Y   0.0f

// Flip either of these to -1 if that axis reads backwards once tested
#define OPTICAL_FLOW_X_SIGN   1
#define OPTICAL_FLOW_Y_SIGN   1

// ============================================================
// MOUNTING CORRECTION — TODO: measure once the sensor is mounted
// ============================================================
// If the sensor isn't mounted facing exactly the same way as the
// robot's forward direction, its X/Y axes need rotating to match.
#define OPTICAL_FLOW_MOUNTING_ANGLE_DEG   0.0f

// Distance from the robot's zero-point-turn origin to the sensor.
// Not yet used for correction (see note in optical_flow.cpp) — kept
// here as a placeholder so the value is measured and ready when the
// lever-arm correction is added later.
#define OPTICAL_FLOW_OFFSET_X_MM   0.0f
#define OPTICAL_FLOW_OFFSET_Y_MM   0.0f

// ============================================================
// PUBLIC FUNCTIONS
// ============================================================

// Call once at startup. Sets up SPI and the sensor itself.
// Returns true if the sensor was detected and configured successfully.
bool optical_flow_init();

// Clears any accumulated/stale state. Call at the start of a run.
void optical_flow_zero();

// Call this regularly (e.g. every scheduler tick). Reads the sensor
// and updates the latest forward/right delta (see getters below).
void optical_flow_poll();

// Most recent movement since the last poll, in the ROBOT's own
// forward/right frame (already corrected for mounting angle), in mm.
float optical_flow_get_delta_forward_mm();
float optical_flow_get_delta_right_mm();

// True if the last poll got a valid reading (sensor connected,
// not reporting a fault/surface-too-far error).
bool optical_flow_is_valid();

#endif /* OPTICAL_FLOW_H_ */