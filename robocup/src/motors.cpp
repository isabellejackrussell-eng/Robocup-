#include "motors.h"
#include "Arduino.h"
#include <Servo.h>
#include <math.h>

// ============================================================
// HARDWARE OBJECTS
// ============================================================
static Servo motorLeft;
static Servo motorRight;

// ============================================================
// RAW ENCODER COUNTS — updated only inside the interrupt handlers
// ============================================================
volatile long encoderCountLeft  = 0;
volatile long encoderCountRight = 0;

static volatile bool aStateLeft  = false;
static volatile bool bStateLeft  = false;
static volatile bool aStateRight = false;
static volatile bool bStateRight = false;

// ============================================================
// POSE STATE — the robot's estimated position/heading, built from
// wheel encoders only. Updated inside motors_poll().
// ============================================================
static float poseXmm       = 0;
static float poseYmm       = 0;
static float poseHeadingDeg = 0;

// Encoder counts as of the last time motors_poll() ran, so we can
// work out how much each wheel has moved since then.
static long lastCountLeft  = 0;
static long lastCountRight = 0;

long motors_get_left_ticks()  { return encoderCountLeft; }
long motors_get_right_ticks() { return encoderCountRight; }

// ============================================================
// INTERRUPT HANDLERS
// Kept deliberately tiny — just decode direction and update the
// raw tick count. No other logic belongs in here.
// ============================================================
static void handleLeftEncoderA()
{
    aStateLeft = digitalRead(LEFT_ENCODER_PIN_A) == HIGH;
    encoderCountLeft += (aStateLeft != bStateLeft) ? +1 : -1;

    bStateLeft = digitalRead(LEFT_ENCODER_PIN_B) == HIGH;
    encoderCountLeft += (aStateLeft == bStateLeft) ? +1 : -1;
}

static void handleRightEncoderA()
{
    aStateRight = digitalRead(RIGHT_ENCODER_PIN_A) == HIGH;
    encoderCountRight += (aStateRight != bStateRight) ? +1 : -1;

    bStateRight = digitalRead(RIGHT_ENCODER_PIN_B) == HIGH;
    encoderCountRight += (aStateRight == bStateRight) ? +1 : -1;
}

// ============================================================
// LOW-LEVEL HELPERS
// ============================================================

// Converts a power value into a servo pulse width in microseconds.
static int power_to_pulse_us(int power)
{
    power = constrain(power, MIN_SPEED_CAP, MAX_SPEED_CAP);

    if (power == 0)
    {
        return MOTOR_STOP_US;
    }
    if (power > 0)
    {
        return map(power, 0, MAX_SPEED_CAP, MOTOR_STOP_US, MOTOR_FULL_FORWARD_US);
    }
    return map(power, 0, MIN_SPEED_CAP, MOTOR_STOP_US, MOTOR_FULL_REVERSE_US);
}

// ============================================================
// PUBLIC FUNCTIONS
// ============================================================

void motors_init()
{
    motorLeft.attach(LEFT_MOTOR_PIN);
    motorRight.attach(RIGHT_MOTOR_PIN);
    motorLeft.writeMicroseconds(MOTOR_STOP_US);
    motorRight.writeMicroseconds(MOTOR_STOP_US);
    delay(2000); // let ESCs arm at stop before doing anything
    

    pinMode(LEFT_ENCODER_PIN_A, INPUT);
    pinMode(LEFT_ENCODER_PIN_B, INPUT);
    pinMode(RIGHT_ENCODER_PIN_A, INPUT);
    pinMode(RIGHT_ENCODER_PIN_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A),  handleLeftEncoderA,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), handleRightEncoderA, CHANGE);

    motors_zero();

    Serial.println("[MOTORS] Initialised");
}

void motors_zero()
{
    encoderCountLeft  = 0;
    encoderCountRight = 0;
    lastCountLeft     = 0;
    lastCountRight    = 0;

    poseXmm        = 0;
    poseYmm        = 0;
    poseHeadingDeg = 0;
}

void motors_poll()
{
    // Snapshot the counts once, so they can't change mid-calculation
    // if an interrupt fires while we're working with them.
    long currentLeft  = encoderCountLeft;
    long currentRight = encoderCountRight;

    long deltaTicksLeft  = currentLeft  - lastCountLeft;
    long deltaTicksRight = currentRight - lastCountRight;
    lastCountLeft  = currentLeft;
    lastCountRight = currentRight;

    // --- 2D transform: ticks -> real-world distance -> updated pose ---
    float distLeftMm   = deltaTicksLeft  / TICKS_PER_MM;
    float distRightMm  = deltaTicksRight / TICKS_PER_MM;
    float distCenterMm = (distLeftMm + distRightMm) / 2.0f;

    float deltaHeadingRad = (distRightMm - distLeftMm) / WHEEL_TRACK_WIDTH_MM;

    // Use the heading at the MIDDLE of this movement step for a more
    // accurate small-step approximation than using the old heading alone.
    float headingBeforeRad = poseHeadingDeg * DEG_TO_RAD;
    float midHeadingRad    = headingBeforeRad + deltaHeadingRad / 2.0f;

    poseXmm += distCenterMm * cos(midHeadingRad);
    poseYmm += distCenterMm * sin(midHeadingRad);
    poseHeadingDeg += deltaHeadingRad * RAD_TO_DEG;

    // Keep heading within -180..180 degrees
    while (poseHeadingDeg > 180.0f)  poseHeadingDeg -= 360.0f;
    while (poseHeadingDeg < -180.0f) poseHeadingDeg += 360.0f;
}

void motors_write(int leftPower, int rightPower)
{
    leftPower  = constrain(leftPower,  MIN_SPEED_CAP, MAX_SPEED_CAP);
    rightPower = constrain(rightPower, MIN_SPEED_CAP, MAX_SPEED_CAP);

    if (LEFT_MOTOR_INVERTED)  leftPower  = -leftPower;
    if (RIGHT_MOTOR_INVERTED) rightPower = -rightPower;

    motorLeft.writeMicroseconds(power_to_pulse_us(leftPower));
    motorRight.writeMicroseconds(power_to_pulse_us(rightPower));
}


float motors_get_x_mm()       { return poseXmm; }
float motors_get_y_mm()       { return poseYmm; }
float motors_get_heading_deg() { return poseHeadingDeg; }



// ============================================================
// SELF-TEST
// ============================================================

// Small helper used only by the self-test — prints both raw tick
// counts on one line.
static void print_ticks()
{
    Serial.print("[MOTORS TEST] Left ticks: ");
    Serial.print(motors_get_left_ticks());
    Serial.print("   Right ticks: ");
    Serial.println(motors_get_right_ticks());
}

// Runs one phase of the test: drives at the given power for durationMs,
// printing tick counts every TEST_PRINT_MS along the way.
static void run_test_phase(int power, unsigned long durationMs)
{
    motors_write(power, power);

    unsigned long phaseStart = millis();
    unsigned long lastPrint  = 0;

    while (millis() - phaseStart < durationMs)
    {
        if (millis() - lastPrint >= TEST_PRINT_MS)
        {
            lastPrint = millis();
            print_ticks();
        }
    }
}

void motors_test_forward_back()
{
    Serial.println("[MOTORS TEST] Starting forward/back test");
    motors_zero();

    Serial.println("[MOTORS TEST] Forward");
    run_test_phase(TEST_DRIVE_SPEED, TEST_DRIVE_MS);

    Serial.println("[MOTORS TEST] Stop");
    run_test_phase(0, TEST_PAUSE_MS);

    Serial.println("[MOTORS TEST] Reverse");
    run_test_phase(-TEST_DRIVE_SPEED, TEST_DRIVE_MS);

    Serial.println("[MOTORS TEST] Stop");
    run_test_phase(0, TEST_PAUSE_MS);

    Serial.println("[MOTORS TEST] Done");
}