#include "sensors/wall_sensors.h"

#include <Arduino.h>

#include "config/robot_config.h"
#include "debug/debug_port.h"
#include "sensors/range_tof.h"

namespace sensors::wall {
namespace {
WallSensorRaw raw;
volatile uint32_t echoRiseUs[config::ULTRASONIC_COUNT] = {};
volatile uint32_t echoDurationUs[config::ULTRASONIC_COUNT] = {};
volatile bool echoComplete[config::ULTRASONIC_COUNT] = {};
uint8_t activeUltrasonic = 0;
uint32_t triggerStartedUs = 0;
bool waitingForEcho = false;

bool ultrasonicConfigured(uint8_t index) {
  return config::ULTRASONIC_TRIGGER_PINS[index] != config::PIN_UNASSIGNED &&
         config::ULTRASONIC_ECHO_PINS[index] != config::PIN_UNASSIGNED;
}

void handleEcho(uint8_t index) {
  if (digitalRead(config::ULTRASONIC_ECHO_PINS[index]) == HIGH) {
    echoRiseUs[index] = micros();
  } else if (echoRiseUs[index] != 0) {
    echoDurationUs[index] = micros() - echoRiseUs[index];
    echoComplete[index] = true;
    echoRiseUs[index] = 0;
  }
}

void echo0() { handleEcho(0); }
void echo1() { handleEcho(1); }
void echo2() { handleEcho(2); }
using EchoHandler = void (*)();
EchoHandler handlers[config::ULTRASONIC_COUNT] = {echo0, echo1, echo2};

DistanceReading& ultrasonicReading(uint8_t index) {
  if (index == config::ULTRASONIC_LEFT) return raw.ultrasonicLeft;
  if (index == config::ULTRASONIC_FRONT) return raw.ultrasonicFront;
  return raw.ultrasonicRight;
}

void trigger(uint8_t index) {
  const int8_t pin = config::ULTRASONIC_TRIGGER_PINS[index];
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  echoComplete[index] = false;
  triggerStartedUs = micros();
  waitingForEcho = true;
}
}  // namespace

bool initWallSensors() {
  bool anyUltrasonic = false;
  for (uint8_t i = 0; i < config::ULTRASONIC_COUNT; ++i) {
    if (!ultrasonicConfigured(i)) continue;
    anyUltrasonic = true;
    pinMode(config::ULTRASONIC_TRIGGER_PINS[i], OUTPUT);
    digitalWrite(config::ULTRASONIC_TRIGGER_PINS[i], LOW);
    pinMode(config::ULTRASONIC_ECHO_PINS[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(config::ULTRASONIC_ECHO_PINS[i]), handlers[i], CHANGE);
  }
  if (!anyUltrasonic) {
    debug::Log.println(F("[WALL] Ultrasonics NOT CONFIGURED: set trigger/echo pins"));
  }
  return anyUltrasonic || range_tof::configured(config::CROSS_LEFT_TOF) ||
         range_tof::configured(config::CROSS_RIGHT_TOF);
}

void serviceUltrasonics() {
  uint8_t checked = 0;
  while (checked < config::ULTRASONIC_COUNT && !ultrasonicConfigured(activeUltrasonic)) {
    activeUltrasonic = (activeUltrasonic + 1) % config::ULTRASONIC_COUNT;
    ++checked;
  }
  if (checked == config::ULTRASONIC_COUNT) return;

  if (!waitingForEcho) {
    trigger(activeUltrasonic);
    return;
  }

  const uint32_t nowUs = micros();
  if (echoComplete[activeUltrasonic]) {
    noInterrupts();
    const uint32_t duration = echoDurationUs[activeUltrasonic];
    echoComplete[activeUltrasonic] = false;
    interrupts();
    DistanceReading& reading = ultrasonicReading(activeUltrasonic);
    reading.distanceMm = duration * 0.343f * 0.5f;
    reading.valid = reading.distanceMm >= config::WALL_MIN_VALID_MM &&
                    reading.distanceMm <= config::WALL_MAX_VALID_MM;
    reading.timestampMs = millis();
    waitingForEcho = false;
    activeUltrasonic = (activeUltrasonic + 1) % config::ULTRASONIC_COUNT;
  } else if (nowUs - triggerStartedUs >= config::ULTRASONIC_TIMEOUT_US) {
    DistanceReading& reading = ultrasonicReading(activeUltrasonic);
    reading.distanceMm = NAN;
    reading.valid = false;
    reading.timestampMs = millis();
    waitingForEcho = false;
    activeUltrasonic = (activeUltrasonic + 1) % config::ULTRASONIC_COUNT;
  }
}

void readUltrasonicLeft() { serviceUltrasonics(); }
void readUltrasonicFront() { serviceUltrasonics(); }
void readUltrasonicRight() { serviceUltrasonics(); }

void readCrossToFLeft() { raw.crossToFLeft = range_tof::get(config::CROSS_LEFT_TOF); }
void readCrossToFRight() { raw.crossToFRight = range_tof::get(config::CROSS_RIGHT_TOF); }

void readAllWallSensors() {
  serviceUltrasonics();
  readCrossToFLeft();
  readCrossToFRight();
  raw.timestampMs = millis();
}

const WallSensorRaw& getWallSensorRaw() { return raw; }

}  // namespace sensors::wall

