#include "tof.h"

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

// ============================================================
// HARDWARE OBJECTS
// ============================================================

static SX1509 io;   // the one 505_TOF_Expander board, at 0x71
static VL53L1X rangeSensors[RANGE_TOF_COUNT];

// ============================================================
// STATE
// ============================================================

static uint16_t rangeDistanceMm[RANGE_TOF_COUNT];
static bool rangeValid[RANGE_TOF_COUNT];


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

        // Release XSHUT.
        // The XSHUT line is not level shifted, so allow it to
        // float high rather than driving it HIGH.
        io.pinMode(RANGE_TOF_XSHUT_PINS[i], INPUT);

        delay(10);

        // Configure VL53L1X
        rangeSensors[i].setTimeout(500);

        if (!rangeSensors[i].init())
        {
            Serial.print("[TOF] Sensor ");
            Serial.print(i);
            Serial.println(" FAILED to initialise");

            rangeValid[i] = false;
            allOk = false;

            continue;
        }

        // Give this sensor a unique I2C address
        rangeSensors[i].setAddress(
            RANGE_TOF_I2C_ADDRESSES[i]
        );

        // Same settings as your working test
        rangeSensors[i].setDistanceMode(VL53L1X::Long);

        rangeSensors[i].setMeasurementTimingBudget(
            RANGE_TOF_TIMING_BUDGET_US
        );

        rangeSensors[i].startContinuous(
            RANGE_TOF_PERIOD_MS
        );

        rangeValid[i] = true;

        Serial.print("[TOF] Sensor ");
        Serial.print(i);
        Serial.println(" initialised");
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

        rangeDistanceMm[i] = rangeSensors[i].read();

        rangeValid[i] =
            !rangeSensors[i].timeoutOccurred();
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
// SELF-TEST
// ============================================================

void range_tof_test()
{
    Serial.println("[TOF TEST] Starting range ToF test");

    unsigned long testStart = millis();

    while (millis() - testStart < TOF_TEST_DURATION_MS)
    {
        range_tof_poll();

        for (uint8_t i = 0; i < RANGE_TOF_COUNT; i++)
        {
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(range_tof_get_distance_mm(i));
            Serial.print("mm  valid: ");
            Serial.print(range_tof_is_valid(i));
            Serial.print("   ");
        }
        Serial.println();

        delay(TOF_TEST_PRINT_MS);
    }

    Serial.println("[TOF TEST] Done");
}