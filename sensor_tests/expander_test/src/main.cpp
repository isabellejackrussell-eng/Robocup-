#include "tof.h"

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

// ============================================================
// HARDWARE OBJECTS
// ============================================================

static SX1509 io;   // 505_TOF_Expander board

static VL53L1X l1Sensors[RANGE_TOF_COUNT];
static VL53L0X l0Sensors[RANGE_TOF_COUNT];

// ============================================================
// STATE
// ============================================================

static uint16_t rangeDistanceMm[RANGE_TOF_COUNT];
static bool rangeValid[RANGE_TOF_COUNT];


// ============================================================
// SENSOR TYPES
// ============================================================

// Change these to match which physical positions contain L0/L1
static const bool sensorIsL0[RANGE_TOF_COUNT] = {
    false,  // Sensor 0 = L1
    false,  // Sensor 1 = L1
    false,  // Sensor 2 = L1
    true,   // Sensor 3 = L0
    true,   // Sensor 4 = L0
    true,   // Sensor 5 = L0
    true,   // Sensor 6 = L0
    true    // Sensor 7 = L0
};


// ============================================================
// INITIALISATION
// ============================================================

bool range_tof_init()
{
    Wire.begin();
    Wire.setClock(400000);

    // Initialise SX1509
    if (!io.begin(SX1509_I2C_ADDRESS))
    {
        Serial.println("[TOF] Failed to find SX1509 expander");
        return false;
    }

    // --------------------------------------------------------
    // Hold all sensors in reset
    // --------------------------------------------------------

    for (uint8_t i = 0; i < RANGE_TOF_COUNT; i++)
    {
        if (RANGE_TOF_XSHUT_PINS[i] == PIN_UNASSIGNED)
            continue;

        io.pinMode(RANGE_TOF_XSHUT_PINS[i], OUTPUT);
        io.digitalWrite(RANGE_TOF_XSHUT_PINS[i], LOW);
    }

    // --------------------------------------------------------
    // Bring sensors up one at a time
    // --------------------------------------------------------

    bool allOk = true;

    for (uint8_t i = 0; i < RANGE_TOF_COUNT; i++)
    {
        if (RANGE_TOF_XSHUT_PINS[i] == PIN_UNASSIGNED)
        {
            Serial.print("[TOF] Sensor ");
            Serial.print(i);
            Serial.println(" NOT WIRED — skipping");

            rangeValid[i] = false;
            continue;
        }

        // Release XSHUT
        io.pinMode(RANGE_TOF_XSHUT_PINS[i], INPUT);

        delay(10);

        // ====================================================
        // VL53L0X
        // ====================================================

        if (sensorIsL0[i])
        {
            l0Sensors[i].setTimeout(500);

            if (!l0Sensors[i].init())
            {
                Serial.print("[TOF] L0 sensor ");
                Serial.print(i);
                Serial.println(" FAILED to initialise");

                rangeValid[i] = false;
                allOk = false;
                continue;
            }

            // Give this sensor a unique I2C address
            l0Sensors[i].setAddress(
                RANGE_TOF_I2C_ADDRESSES[i]
            );

            l0Sensors[i].startContinuous(
                RANGE_TOF_PERIOD_MS
            );

            rangeValid[i] = true;

            Serial.print("[TOF] L0 sensor ");
            Serial.print(i);
            Serial.println(" initialised");
        }

        // ====================================================
        // VL53L1X
        // ====================================================

        else
        {
            l1Sensors[i].setTimeout(500);

            if (!l1Sensors[i].init())
            {
                Serial.print("[TOF] L1 sensor ");
                Serial.print(i);
                Serial.println(" FAILED to initialise");

                rangeValid[i] = false;
                allOk = false;
                continue;
            }

            // Give this sensor a unique I2C address
            l1Sensors[i].setAddress(
                RANGE_TOF_I2C_ADDRESSES[i]
            );

            // L1-specific settings
            l1Sensors[i].setDistanceMode(VL53L1X::Long);

            l1Sensors[i].setMeasurementTimingBudget(
                RANGE_TOF_TIMING_BUDGET_US
            );

            l1Sensors[i].startContinuous(
                RANGE_TOF_PERIOD_MS
            );

            rangeValid[i] = true;

            Serial.print("[TOF] L1 sensor ");
            Serial.print(i);
            Serial.println(" initialised");
        }
    }

    range_tof_zero();

    return allOk;
}


// ============================================================
// ZERO
// ============================================================

void range_tof_zero()
{
    for (uint8_t i = 0; i < RANGE_TOF_COUNT; i++)
    {
        rangeDistanceMm[i] = 0;
    }
}


// ============================================================
// POLL
// ============================================================

void range_tof_poll()
{
    for (uint8_t i = 0; i < RANGE_TOF_COUNT; i++)
    {
        if (RANGE_TOF_XSHUT_PINS[i] == PIN_UNASSIGNED)
            continue;

        if (!rangeValid[i])
            continue;

        // Read using the correct sensor class
        if (sensorIsL0[i])
        {
            rangeDistanceMm[i] =
                l0Sensors[i].readRangeContinuousMillimeters();

            rangeValid[i] =
                !l0Sensors[i].timeoutOccurred();
        }
        else
        {
            rangeDistanceMm[i] =
                l1Sensors[i].read();

            rangeValid[i] =
                !l1Sensors[i].timeoutOccurred();
        }
    }
}


// ============================================================
// GET DISTANCE
// ============================================================

uint16_t range_tof_get_distance_mm(uint8_t index)
{
    if (index >= RANGE_TOF_COUNT)
        return 0;

    return rangeDistanceMm[index];
}


// ============================================================
// VALIDITY
// ============================================================

bool range_tof_is_valid(uint8_t index)
{
    if (index >= RANGE_TOF_COUNT)
        return false;

    return rangeValid[index];
}


// ============================================================
// PRINT READINGS
// ============================================================

void range_tof_print_readings()
{
    for (uint8_t i = 0; i < RANGE_TOF_COUNT; i++)
    {
        Serial.print("Sensor ");
        Serial.print(i);

        if (sensorIsL0[i])
            Serial.print(" (L0): ");
        else
            Serial.print(" (L1): ");

        Serial.print(range_tof_get_distance_mm(i));
        Serial.print("mm  valid: ");
        Serial.print(range_tof_is_valid(i));
        Serial.print("   ");
    }

    Serial.println();
}


// ============================================================
// SELF-TEST
// ============================================================

void range_tof_test()
{
    Serial.println("[TOF TEST] Starting range ToF test");

    unsigned long testStart = millis();

    while (millis() - testStart < TOF_TEST_DURATION_MS)
    {
        range_tof_poll();
        range_tof_print_readings();

        delay(TOF_TEST_PRINT_MS);
    }

    Serial.println("[TOF TEST] Done");
}


// ============================================================
// ARDUINO ENTRY POINTS
// ============================================================

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    if (!range_tof_init())
    {
        Serial.println("[TOF] One or more sensors failed to initialise");
    }
}

void loop()
{
    range_tof_poll();
    range_tof_print_readings();
    delay(TOF_TEST_PRINT_MS);
}
