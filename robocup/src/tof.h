#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

// ============================================================
// CONFIGURATION
// ============================================================

// Number of VL53L1X range sensors
#define RANGE_TOF_COUNT 1

// SX1509 I2C address
#define SX1509_I2C_ADDRESS 0x3F

// SX1509 pin connected to XSHUT of each VL53L1X
const uint8_t RANGE_TOF_XSHUT_PINS[RANGE_TOF_COUNT] = {0};

// I2C addresses assigned to the VL53L1X sensors
const uint8_t RANGE_TOF_I2C_ADDRESSES[RANGE_TOF_COUNT] = {0x30};

// VL53L1X measurement settings
#define RANGE_TOF_TIMING_BUDGET_US 50000
#define RANGE_TOF_PERIOD_MS 50

// Used when a sensor isn't connected
#define PIN_UNASSIGNED 255


// ============================================================
// RANGE ToF FUNCTIONS
// ============================================================

bool range_tof_init();

void range_tof_zero();

void range_tof_poll();

uint16_t range_tof_get_distance_mm(uint8_t index);

bool range_tof_is_valid(uint8_t index);

#endif