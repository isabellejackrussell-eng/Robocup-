#include "motors.h"
#include <Servo.h>

static Servo motorLeft;
static Servo motorRight;
static uint8_t leftCommandPercent  = 0;
static uint8_t rightCommandPercent = 0;


static volatile long encoderCountLeft  = 0;
static volatile long encoderCountRight = 0;

static long previousEncoderCountLeft  = 0;
static long previousEncoderCountRight = 0;

static float leftTicksPerSecond  = 0.0f;
static float rightTicksPerSecond = 0.0f;

static unsigned long lastEncoderUpdateMs = 0;


// How often encoder speed is recalculated.
//
// 50 ms gives 20 updates per second.
static constexpr unsigned long ENCODER_UPDATE_INTERVAL_MS = 50;


// Left encoder interupt
static void handleLeftEncoder() {
    bool a = digitalRead(LEFT_ENCODER_PIN_A);
    bool b = digitalRead(LEFT_ENCODER_PIN_B);

    int direction = (a == b) ? 1 : -1;

    if (LEFT_ENCODER_INVERTED){
        direction = -direction;
    }

    encoderCountLeft += direction;
}

// Right encoder interupt
static void handleRightEncoder() {
    bool a = digitalRead(RIGHT_ENCODER_PIN_A);
    bool b = digitalRead(RIGHT_ENCODER_PIN_B);

    int direction = (a == b) ? 1 : -1;

    if (RIGHT_ENCODER_INVERTED)
    {
        direction = -direction;
    }

    encoderCountRight += direction;
}

static int speedPercentToPulse(uint8_t percent) {
    percent = constrain(percent, 0, 100);

    return map(percent,0, 100, MOTOR_STOP_US, MOTOR_FULL_FORWARD_US);
}

static void readEncoderCounts(long &left, long &right)
{
    noInterrupts();

    left  = encoderCountLeft;
    right = encoderCountRight;

    interrupts();
}

void motors_init() {
    motorLeft.attach(LEFT_MOTOR_PIN);
    motorRight.attach(RIGHT_MOTOR_PIN);

    // Make sure both motors start stopped.
    motorLeft.writeMicroseconds(MOTOR_STOP_US);
    motorRight.writeMicroseconds(MOTOR_STOP_US);

    leftCommandPercent  = 0;
    rightCommandPercent = 0;


    // Encoder inputs
    pinMode(LEFT_ENCODER_PIN_A, INPUT);
    pinMode(LEFT_ENCODER_PIN_B, INPUT);
    pinMode(RIGHT_ENCODER_PIN_A, INPUT);
    pinMode(RIGHT_ENCODER_PIN_B, INPUT);


    // Interrupt on channel A.
    // CHANGE gives an interrupt on both rising and falling edges.
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN_A), handleLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN_A), handleRightEncoder, CHANGE);

    // Reset encoder state
    motors_reset_encoders();
    lastEncoderUpdateMs = millis();

    // Give the motor controllers time to see the stop pulse before  starting commanding movement.
    delay(2000);

    Serial.println("[MOTORS] Initialised");
}


void motors_set_left_speed(uint8_t percent){
    percent = constrain(percent, 0, 100);
    leftCommandPercent = percent;
    int pulse = speedPercentToPulse(percent);

    // The left motor may need its direction electrically inverted because of how it is mounted.
    if (LEFT_MOTOR_INVERTED && percent > 0) {
        pulse = MOTOR_STOP_US - (pulse - MOTOR_STOP_US);
    }

    motorLeft.writeMicroseconds(pulse);
}


void motors_set_right_speed(uint8_t percent){
    percent = constrain(percent, 0, 100);
    rightCommandPercent = percent;
    int pulse = speedPercentToPulse(percent);

    // Right motor is usually physically mirrored relative to left.
    if (RIGHT_MOTOR_INVERTED && percent > 0){
        pulse = MOTOR_STOP_US - (pulse - MOTOR_STOP_US);
    }

    motorRight.writeMicroseconds(pulse);
}


void motors_set_speed(uint8_t leftPercent, uint8_t rightPercent){
    motors_set_left_speed(leftPercent);
    motors_set_right_speed(rightPercent);
}


void motors_stop()
{
    motors_set_left_speed(0);
    motors_set_right_speed(0);
}


uint8_t motors_get_left_command()
{
    return leftCommandPercent;
}


uint8_t motors_get_right_command()
{
    return rightCommandPercent;
}


void motors_update(){
    unsigned long now = millis();
    unsigned long elapsedMs = now - lastEncoderUpdateMs;

    // Only calculate speed every 50 ms.
    if (elapsedMs < ENCODER_UPDATE_INTERVAL_MS){
        return;
    }

    long currentLeft;
    long currentRight;

    readEncoderCounts(currentLeft, currentRight);


    long leftDelta = currentLeft - previousEncoderCountLeft;
    long rightDelta = currentRight - previousEncoderCountRight;


    // Convert ticks measured during this interval into ticks/sec.
    float elapsedSeconds =elapsedMs / 1000.0f;

    leftTicksPerSecond = leftDelta / elapsedSeconds;
    rightTicksPerSecond = rightDelta / elapsedSeconds;
    previousEncoderCountLeft = currentLeft;
    previousEncoderCountRight = currentRight;
    lastEncoderUpdateMs = now;
}


long motors_get_left_ticks(){
    long count;
    noInterrupts();
    count = encoderCountLeft;
    interrupts();

    return count;
}


long motors_get_right_ticks(){
    long count;
    noInterrupts();
    count = encoderCountRight;
    interrupts();
    return count;
}


float motors_get_left_ticks_per_second(){
    return leftTicksPerSecond;
}


float motors_get_right_ticks_per_second(){
    return rightTicksPerSecond;
}


void motors_reset_encoders(){
    noInterrupts();

    encoderCountLeft  = 0;
    encoderCountRight = 0;

    interrupts();

    previousEncoderCountLeft  = 0;
    previousEncoderCountRight = 0;

    leftTicksPerSecond  = 0.0f;
    rightTicksPerSecond = 0.0f;

    lastEncoderUpdateMs = millis();
}