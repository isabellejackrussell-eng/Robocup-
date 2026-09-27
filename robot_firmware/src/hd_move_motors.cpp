#include "smartServo.h"

#include <HerkulexServo.h>
#include <math.h>


// ============================================================
// CONFIGURATION
// ============================================================

static constexpr uint32_t SERVO_BAUD = 115200;

static constexpr uint8_t SERVO_1_ID = 1;
static constexpr uint8_t SERVO_4_ID = 4;


// Herkulex position:
//   512 = centre
//   approximately 0.325 degrees per position count
static constexpr int SERVO_CENTRE = 512;

static constexpr float DEGREES_PER_TICK = 0.325f;


// Avoid the extreme ends of the servo's allowed position range.
static constexpr float MIN_ANGLE = -150.0f;
static constexpr float MAX_ANGLE = 150.0f;

static constexpr uint16_t MIN_POSITION = 30;
static constexpr uint16_t MAX_POSITION = 990;


// Movement time.
//
// Herkulex playtime units are approximately 11.2 ms.
//
// 70 * 11.2 ms = about 784 ms.
static constexpr uint8_t MOVE_TIME = 70;


// ============================================================
// HERKULEX OBJECTS
// ============================================================

static HerkulexServoBus servoBus(Serial2);

static HerkulexServo servo1(servoBus, SERVO_1_ID);
static HerkulexServo servo4(servoBus, SERVO_4_ID);


// ============================================================
// STATE
// ============================================================

static bool servo1Ready = false;
static bool servo4Ready = false;


// Remember the most recently commanded angles.
//
// This prevents us from repeatedly sending exactly the same command.
static float servo1TargetAngle = NAN;
static float servo4TargetAngle = NAN;


// ============================================================
// INTERNAL HELPERS
// ============================================================

static bool detectServo(uint8_t servoID)
{
    HerkulexPacket response{};

    return servoBus.sendPacketAndReadResponse(
        response,
        servoID,
        HerkulexCommand::Stat
    );
}


// ------------------------------------------------------------
// Detect with retries
// ------------------------------------------------------------

static bool detectServoWithRetries(uint8_t servoID)
{
    static constexpr int ATTEMPTS = 5;

    for (int attempt = 1; attempt <= ATTEMPTS; attempt++)
    {
        Serial.print("[SMART SERVO] Looking for ID ");
        Serial.print(servoID);

        Serial.print(" - attempt ");
        Serial.print(attempt);
        Serial.print("/");
        Serial.println(ATTEMPTS);

        if (detectServo(servoID))
        {
            return true;
        }

        delay(100);
    }

    return false;
}


// ------------------------------------------------------------
// Convert angle -> Herkulex position
// ------------------------------------------------------------

static uint16_t angleToPosition(float angle)
{
    angle = constrain(
        angle,
        MIN_ANGLE,
        MAX_ANGLE
    );

    int position =
        SERVO_CENTRE +
        lroundf(angle / DEGREES_PER_TICK);

    position = constrain(
        position,
        MIN_POSITION,
        MAX_POSITION
    );

    return static_cast<uint16_t>(position);
}


// ------------------------------------------------------------
// Convert Herkulex position -> degrees
// ------------------------------------------------------------

static float positionToAngle(uint16_t position)
{
    return
        (static_cast<int>(position) - SERVO_CENTRE)
        * DEGREES_PER_TICK;
}


// ============================================================
// INITIALISATION
// ============================================================

void smartServoInitialise() {
    Serial.println();
    Serial.println("==============================");
    Serial.println("Initialising smart servos");
    Serial.println("==============================");

    // Start hardware UART connected to the Smart Servo board.
    Serial2.begin(SERVO_BAUD);

    // Give the servos time to power up.
    delay(500);


    // ========================================================
    // SERVO 1
    // ========================================================

    servo1Ready = detectServoWithRetries(SERVO_1_ID);

    if (servo1Ready)
    {
        Serial.println("[SMART SERVO] Servo 1 detected");

        // Position control is the normal boot mode.
        // Enable motor torque so that it can physically move.
        servo1.setTorqueOn();

        delay(50);

        uint16_t position = servo1.getPosition();

        Serial.print("[SMART SERVO] Servo 1 position: ");
        Serial.println(position);

        Serial.print("[SMART SERVO] Servo 1 angle: ");
        Serial.println(positionToAngle(position));
    } else
    {
        Serial.println(
            "[SMART SERVO] WARNING: Servo 1 NOT detected"
        );
    }


    // ========================================================
    // SERVO 4
    // ========================================================

    servo4Ready = detectServoWithRetries(SERVO_4_ID);

    if (servo4Ready)
    {
        Serial.println("[SMART SERVO] Servo 4 detected");

        servo4.setTorqueOn();

        delay(50);

        uint16_t position = servo4.getPosition();

        Serial.print("[SMART SERVO] Servo 4 position: ");
        Serial.println(position);

        Serial.print("[SMART SERVO] Servo 4 angle: ");
        Serial.println(positionToAngle(position));
    }
    else
    {
        Serial.println(
            "[SMART SERVO] WARNING: Servo 4 NOT detected"
        );
    }


    Serial.println("==============================");
    Serial.println("Smart servo init complete");
    Serial.println("==============================");
    Serial.println();
}


// ============================================================
// BUS UPDATE
// ============================================================

void smartServoUpdate()
{
    servoBus.update();
}


// ============================================================
// CHECK READY
// ============================================================

bool smartServoIsReady(uint8_t servoID)
{
    switch (servoID)
    {
        case SERVO_1_ID:
            return servo1Ready;

        case SERVO_4_ID:
            return servo4Ready;

        default:
            return false;
    }
}


// ============================================================
// SET ANGLE
// ============================================================

bool setServoAngle(uint8_t servoID, float angle)
{
    // Limit angle to allowed range.
    angle = constrain(
        angle,
        MIN_ANGLE,
        MAX_ANGLE
    );


    // Convert degrees into the native Herkulex position.
    uint16_t position = angleToPosition(angle);


    // ========================================================
    // SERVO 1
    // ========================================================

    if (servoID == SERVO_1_ID)
    {
        if (!servo1Ready)
        {
            Serial.println(
                "[SMART SERVO] Cannot move servo 1: not ready"
            );

            return false;
        }


        // Do not repeatedly transmit the exact same target.
        if (!isnan(servo1TargetAngle) &&
            fabsf(angle - servo1TargetAngle) < 0.01f)
        {
            return true;
        }


        servo1TargetAngle = angle;

        Serial.print("[SMART SERVO] Servo 1 -> ");
        Serial.print(angle);
        Serial.print(" degrees (position ");
        Serial.print(position);
        Serial.println(")");

        servo1.setPosition(
            position,
            MOVE_TIME,
            HerkulexLed::Blue
        );

        return true;
    }


    // ========================================================
    // SERVO 4
    // ========================================================

    if (servoID == SERVO_4_ID)
    {
        if (!servo4Ready)
        {
            Serial.println(
                "[SMART SERVO] Cannot move servo 4: not ready"
            );

            return false;
        }


        // Do not repeatedly transmit the exact same target.
        if (!isnan(servo4TargetAngle) &&
            fabsf(angle - servo4TargetAngle) < 0.01f)
        {
            return true;
        }


        servo4TargetAngle = angle;

        Serial.print("[SMART SERVO] Servo 4 -> ");
        Serial.print(angle);
        Serial.print(" degrees (position ");
        Serial.print(position);
        Serial.println(")");

        servo4.setPosition(
            position,
            MOVE_TIME,
            HerkulexLed::Green
        );

        return true;
    }


    // ========================================================
    // UNKNOWN SERVO
    // ========================================================

    Serial.print(
        "[SMART SERVO] Unknown servo ID: "
    );

    Serial.println(servoID);

    return false;
}


// ============================================================
// GET RAW POSITION
// ============================================================

uint16_t getServoPosition(uint8_t servoID)
{
    if (servoID == SERVO_1_ID)
    {
        if (!servo1Ready)
        {
            return 0;
        }

        return servo1.getPosition();
    }


    if (servoID == SERVO_4_ID)
    {
        if (!servo4Ready)
        {
            return 0;
        }

        return servo4.getPosition();
    }


    return 0;
}


// ============================================================
// GET ANGLE
// ============================================================

float getServoAngle(uint8_t servoID)
{
    if (!smartServoIsReady(servoID))
    {
        return NAN;
    }

    uint16_t position =
        getServoPosition(servoID);

    return positionToAngle(position);
}


// ============================================================
// PRINT STATUS
// ============================================================

void smartServoPrintStatus(uint8_t servoID)
{
    HerkulexServo *servo = nullptr;


    if (servoID == SERVO_1_ID)
    {
        if (!servo1Ready)
        {
            Serial.println(
                "[SMART SERVO] Servo 1 not ready"
            );

            return;
        }

        servo = &servo1;
    }
    else if (servoID == SERVO_4_ID)
    {
        if (!servo4Ready)
        {
            Serial.println(
                "[SMART SERVO] Servo 4 not ready"
            );

            return;
        }

        servo = &servo4;
    }
    else
    {
        Serial.println(
            "[SMART SERVO] Invalid servo ID"
        );

        return;
    }


    // --------------------------------------------------------
    // Read position
    // --------------------------------------------------------

    uint16_t position =
        servo->getPosition();

    float angle =
        positionToAngle(position);


    // --------------------------------------------------------
    // Read status registers
    // --------------------------------------------------------

    HerkulexStatusError statusError;
    HerkulexStatusDetail statusDetail;

    servo->getStatus(
        statusError,
        statusDetail
    );


    uint8_t error =
        static_cast<uint8_t>(statusError);

    uint8_t detail =
        static_cast<uint8_t>(statusDetail);


    // --------------------------------------------------------
    // Print
    // --------------------------------------------------------

    Serial.println();
    Serial.print("[SMART SERVO] ID ");
    Serial.println(servoID);

    Serial.print("  Position: ");
    Serial.println(position);

    Serial.print("  Angle: ");
    Serial.print(angle);
    Serial.println(" degrees");

    Serial.print("  Error byte: 0x");
    Serial.println(error, HEX);

    Serial.print("  Detail byte: 0x");
    Serial.println(detail, HEX);


    // Torque/motor status
    Serial.print("  Torque: ");

    if (detail & 0x40)
    {
        Serial.println("ON");
    }
    else
    {
        Serial.println("OFF");
    }


    // --------------------------------------------------------
    // Decode errors
    // --------------------------------------------------------

    if (error == 0)
    {
        Serial.println("  Errors: none");
    }
    else
    {
        Serial.println("  Errors:");

        if (error & 0x01)
            Serial.println("    Input voltage");

        if (error & 0x02)
            Serial.println("    Position limit");

        if (error & 0x04)
            Serial.println("    Temperature");

        if (error & 0x08)
            Serial.println("    Invalid packet");

        if (error & 0x10)
            Serial.println("    Overload");

        if (error & 0x20)
            Serial.println("    Driver fault");

        if (error & 0x40)
            Serial.println("    EEPROM fault");
    }

    Serial.println();
}