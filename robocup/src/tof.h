#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

// ============================================================
// CONFIGURATION
// ============================================================

// Number of VL53L1X range sensors
#define RANGE_TOF_COUNT 8

// SX1509 I2C address — all 8 sensors on the 505_TOF_Expander board
#define SX1509_I2C_ADDRESS 0x71

// Used when a sensor isn't connected
#define PIN_UNASSIGNED 255

// SX1509 pin connected to XSHUT of each VL53L1X
const uint8_t RANGE_TOF_XSHUT_PINS[RANGE_TOF_COUNT] = {
    0, 1, PIN_UNASSIGNED, PIN_UNASSIGNED,
    PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED, PIN_UNASSIGNED
};

// I2C addresses assigned to the VL53L1X sensors — all 8 must be
// different from each other and from 0x33 (8x8 array) / 0x71 (expander)
const uint8_t RANGE_TOF_I2C_ADDRESSES[RANGE_TOF_COUNT] = {
    0x30, 0x31, 0x32, 0x34, 0x35, 0x36, 0x37, 0x38
};

// VL53L1X measurement settings
#define RANGE_TOF_TIMING_BUDGET_US 50000
#define RANGE_TOF_PERIOD_MS 50

// Self-test config
#define TOF_TEST_DURATION_MS   10000   // how long the test runs
#define TOF_TEST_PRINT_MS        200   // how often to print during the test


// ============================================================
// RANGE ToF FUNCTIONS
// ============================================================

bool range_tof_init();

void range_tof_zero();

void range_tof_poll();

uint16_t range_tof_get_distance_mm(uint8_t index);

bool range_tof_is_valid(uint8_t index);

// Blocking self-test: polls and prints all RANGE_TOF_COUNT sensors'
// distance + validity for TOF_TEST_DURATION_MS. Call once from
// main.cpp AFTER range_tof_init() to sanity-check wiring before
// trusting these sensors in real logic.
void range_tof_test();

#endif