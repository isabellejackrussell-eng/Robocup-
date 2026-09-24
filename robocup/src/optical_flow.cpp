#include "optical_flow.h"
#include "Arduino.h"
#include <math.h>

// NOTE: this assumes the commonly-used Bitcraze_PMW3901 library.
// If your already-tested PMW3901 example used a different library,
// swap the #include and the calls inside optical_flow_init()/poll()
// below to match it — the rest of this file (the maths) doesn't change.
#include <Bitcraze_PMW3901.h>

// ============================================================
// HARDWARE OBJECT
// ============================================================
static Bitcraze_PMW3901 flowSensor(OPTICAL_FLOW_CS_PIN);

// ============================================================
// STATE
// ============================================================
static float lastDeltaForwardMm = 0;
static float lastDeltaRightMm   = 0;
static bool  lastReadingValid   = false;

// ============================================================
// LOW-LEVEL HELPERS
// ============================================================

// Rotates a local (x, y) reading by the sensor's fixed mounting
// angle, so the result lines up with the robot's own forward/right
// axes instead of however the sensor itself happens to be turned.
static void rotate_by_mounting_angle(float xMm, float yMm, float &forwardMmOut, float &rightMmOut)
{
    float angleRad = OPTICAL_FLOW_MOUNTING_ANGLE_DEG * DEG_TO_RAD;

    forwardMmOut = xMm * cos(angleRad) - yMm * sin(angleRad);
    rightMmOut   = xMm * sin(angleRad) + yMm * cos(angleRad);
}

// ============================================================
// PUBLIC FUNCTIONS
// ============================================================

bool optical_flow_init()
{
    bool ok = flowSensor.begin();

    if (ok)
    {
        Serial.println("[OPTICAL FLOW] Initialised");
    }
    else
    {
        Serial.println("[OPTICAL FLOW] NOT DETECTED — check wiring/CS pin");
    }

    optical_flow_zero();
    return ok;
}

void optical_flow_zero()
{
    lastDeltaForwardMm = 0;
    lastDeltaRightMm   = 0;
    lastReadingValid   = false;
}

void optical_flow_poll()
{
    int16_t rawDeltaX = 0;
    int16_t rawDeltaY = 0;

    flowSensor.readMotionCount(&rawDeltaX, &rawDeltaY);

    // Raw counts -> local mm, with calibration scale and sign applied.
    float localXmm = rawDeltaX * OPTICAL_FLOW_MM_PER_COUNT_X * OPTICAL_FLOW_X_SIGN;
    float localYmm = rawDeltaY * OPTICAL_FLOW_MM_PER_COUNT_Y * OPTICAL_FLOW_Y_SIGN;

    // --- 2D transform: correct for how the sensor is actually mounted ---
    float forwardMm, rightMm;
    rotate_by_mounting_angle(localXmm, localYmm, forwardMm, rightMm);

    lastDeltaForwardMm = forwardMm;
    lastDeltaRightMm   = rightMm;

    // TODO: once calibrated, add a validity check here (e.g. the
    // library may report a "surface too far/featureless" flag).
    lastReadingValid = true;
}

float optical_flow_get_delta_forward_mm() { return lastDeltaForwardMm; }
float optical_flow_get_delta_right_mm()   { return lastDeltaRightMm; }
bool  optical_flow_is_valid()             { return lastReadingValid; }