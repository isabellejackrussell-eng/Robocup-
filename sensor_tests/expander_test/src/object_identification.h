#ifndef OBJECT_IDENTIFICATION_H
#define OBJECT_IDENTIFICATION_H

#include <Arduino.h>

// Tune these values after testing on the robot.
constexpr uint16_t OBJECT_MAX_DISTANCE_MM = 2000;
constexpr uint8_t OBJECT_STABLE_SAMPLE_COUNT = 3;
constexpr uint8_t OBJECT_SENSOR_SET_COUNT = 2;

enum class ObjectType : uint8_t
{
    NONE,
    WEIGHT,
    WALL,
    UNKNOWN
};

struct ObjectDetection
{
    ObjectType type;
    uint16_t distanceMm;
};

// Clear the debounce/filter state for both three-sensor sets.
void object_identification_reset();

// Classify the latest ToF readings and update the filtered results.
// Call this once after range_tof_poll().
void object_identification_update();

// Set 0 uses bottom/top sensors 0 and 1.
// Set 1 uses bottom/top sensors 2 and 3.
ObjectDetection object_identification_get(uint8_t setIndex);

const char *object_type_name(ObjectType type);
void object_identification_print();

#endif
