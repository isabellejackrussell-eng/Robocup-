#include "tof.h"

#include <Wire.h>
#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

namespace tof_diagnostic {
namespace {

constexpr uint8_t kXshutPins[kSensorCount] = {0, 1, 2, 3, 4, 5, 6, 7};
constexpr uint8_t kAssignedAddresses[kSensorCount] = {
    0x30, 0x31, 0x32, 0x34, 0x35, 0x36, 0x37, 0x38};
constexpr uint8_t kPossibleExpanderAddresses[] = {0x71, 0x70, 0x3F, 0x3E};

enum class SensorModel : uint8_t { unknown, vl53l0x, vl53l1x };

SX1509 expander;
VL53L0X l0Sensors[kSensorCount];
VL53L1X l1Sensors[kSensorCount];
SensorModel sensorModels[kSensorCount] = {};
bool sensorReady[kSensorCount] = {};
uint32_t readCount[kSensorCount] = {};
uint32_t timeoutCount[kSensorCount] = {};
uint8_t expanderAddress = 0;

bool addressAcknowledges(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void printHexAddress(uint8_t address) {
  Serial.print("0x");
  if (address < 0x10) {
    Serial.print('0');
  }
  Serial.print(address, HEX);
}

void scanBus(const char* label) {
  Serial.print("\n[I2C] ");
  Serial.println(label);
  uint8_t found = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    if (addressAcknowledges(address)) {
      Serial.print("  ACK at ");
      printHexAddress(address);
      if (address == kDefaultTofAddress) {
        Serial.print(" (default ToF address)");
      }
      Serial.println();
      ++found;
    }
  }
  if (found == 0) {
    Serial.println("  No devices found. Check SDA, SCL, power, ground, and pull-ups.");
  }
}

bool readRegister8(uint8_t deviceAddress, uint8_t registerAddress,
                   uint8_t& value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(deviceAddress, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

bool readRegister16(uint8_t deviceAddress, uint16_t registerAddress,
                    uint8_t& value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(static_cast<uint8_t>(registerAddress >> 8));
  Wire.write(static_cast<uint8_t>(registerAddress));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(deviceAddress, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

SensorModel identifySensor(uint8_t address, uint8_t& modelId) {
  // VL53L0X IDENTIFICATION_MODEL_ID (8-bit register address).
  if (readRegister8(address, 0xC0, modelId) && modelId == 0xEE) {
    return SensorModel::vl53l0x;
  }

  // VL53L1X IDENTIFICATION__MODEL_ID (16-bit register address).
  if (readRegister16(address, 0x010F, modelId) && modelId == 0xEA) {
    return SensorModel::vl53l1x;
  }
  return SensorModel::unknown;
}

const char* modelName(SensorModel model) {
  switch (model) {
    case SensorModel::vl53l0x:
      return "VL53L0X";
    case SensorModel::vl53l1x:
      return "VL53L1X";
    default:
      return "unknown";
  }
}

bool findExpander() {
  Serial.println("\n[EXPANDER] Trying SX1509 addresses 0x71, 0x70, 0x3F, 0x3E");
  for (uint8_t address : kPossibleExpanderAddresses) {
    Serial.print("  ");
    printHexAddress(address);
    if (!addressAcknowledges(address)) {
      Serial.println(" no ACK");
      continue;
    }

    Serial.print(" ACK, SX1509 begin ");
    if (expander.begin(address)) {
      expanderAddress = address;
      Serial.println("passed");
      return true;
    }
    Serial.println("failed");
  }
  return false;
}

void holdSensorInReset(uint8_t sensorIndex) {
  expander.pinMode(kXshutPins[sensorIndex], OUTPUT);
  expander.digitalWrite(kXshutPins[sensorIndex], LOW);
}

void releaseSensor(uint8_t sensorIndex) {
  // XSHUT is not level-shifted, so release it to its pull-up instead of
  // driving it high from the SX1509.
  expander.pinMode(kXshutPins[sensorIndex], INPUT);
}

bool initialiseSensor(uint8_t sensorIndex, SensorModel model) {
  const uint8_t newAddress = kAssignedAddresses[sensorIndex];

  if (model == SensorModel::vl53l0x) {
    VL53L0X& sensor = l0Sensors[sensorIndex];
    sensor.setTimeout(500);
    if (!sensor.init()) {
      return false;
    }
    sensor.setAddress(newAddress);
    sensor.startContinuous(kMeasurementPeriodMs);
    return true;
  }

  if (model == SensorModel::vl53l1x) {
    VL53L1X& sensor = l1Sensors[sensorIndex];
    sensor.setTimeout(500);
    if (!sensor.init()) {
      return false;
    }
    sensor.setAddress(newAddress);
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(kTimingBudgetUs);
    sensor.startContinuous(kMeasurementPeriodMs);
    return true;
  }

  return false;
}

}  // namespace

uint8_t begin() {
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("========================================");
  Serial.println(" ToF expander diagnostic");
  Serial.println("========================================");
  scanBus("Bus state at startup");

  if (!findExpander()) {
    Serial.println("\n[FATAL] No working SX1509 found.");
    Serial.println("Check address straps and compare the scan with 0x71/0x70/0x3F/0x3E.");
    return 0;
  }

  Serial.print("[PASS] SX1509 detected at ");
  printHexAddress(expanderAddress);
  if (expanderAddress != kPreferredExpanderAddress) {
    Serial.print(" (code expected ");
    printHexAddress(kPreferredExpanderAddress);
    Serial.print(')');
  }
  Serial.println();

  Serial.println("\n[XSHUT] Holding all eight channels in reset");
  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    sensorReady[sensorIndex] = false;
    sensorModels[sensorIndex] = SensorModel::unknown;
    holdSensorInReset(sensorIndex);
  }
  delay(20);

  if (addressAcknowledges(kDefaultTofAddress)) {
    Serial.println("[WARN] 0x29 still ACKs with all channels reset.");
    Serial.println("       A sensor may have an uncontrolled/miswired XSHUT line.");
  } else {
    Serial.println("[PASS] Default ToF address 0x29 disappeared with all channels reset");
  }

  uint8_t readyCount = 0;
  Serial.println("\n[SENSORS] Releasing, identifying, and re-addressing one channel at a time");
  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    Serial.print("  channel ");
    Serial.print(sensorIndex);
    Serial.print(" / SX pin ");
    Serial.print(kXshutPins[sensorIndex]);
    Serial.print(": ");

    releaseSensor(sensorIndex);
    delay(20);

    if (!addressAcknowledges(kDefaultTofAddress)) {
      Serial.println("FAIL - nothing appeared at 0x29");
      holdSensorInReset(sensorIndex);
      continue;
    }

    uint8_t modelId = 0;
    const SensorModel model = identifySensor(kDefaultTofAddress, modelId);
    sensorModels[sensorIndex] = model;
    Serial.print(modelName(model));
    Serial.print(" (ID ");
    printHexAddress(modelId);
    Serial.print(") -> ");
    printHexAddress(kAssignedAddresses[sensorIndex]);

    if (model == SensorModel::unknown) {
      Serial.println(" FAIL - unrecognised sensor ID");
      holdSensorInReset(sensorIndex);
      continue;
    }

    if (!initialiseSensor(sensorIndex, model)) {
      Serial.println(" FAIL - library initialisation failed");
      holdSensorInReset(sensorIndex);
      continue;
    }

    delay(5);
    if (!addressAcknowledges(kAssignedAddresses[sensorIndex])) {
      Serial.println(" FAIL - new address did not ACK");
      holdSensorInReset(sensorIndex);
      continue;
    }

    sensorReady[sensorIndex] = true;
    ++readyCount;
    Serial.println(" PASS");
  }

  Wire.setClock(400000);
  Serial.print("\n[SUMMARY] ");
  Serial.print(readyCount);
  Serial.print('/');
  Serial.print(kSensorCount);
  Serial.println(" channels initialised");
  Serial.println("Live output follows: distance in mm, TO=timeout, OFF=not initialised.");
  return readyCount;
}

void printReadings() {
  Serial.print("[RANGE] ");
  for (uint8_t sensorIndex = 0; sensorIndex < kSensorCount; ++sensorIndex) {
    Serial.print('S');
    Serial.print(sensorIndex);
    Serial.print('=');

    if (!sensorReady[sensorIndex]) {
      Serial.print("OFF");
    } else {
      uint16_t distanceMm = 0;
      bool timedOut = false;
      if (sensorModels[sensorIndex] == SensorModel::vl53l0x) {
        distanceMm = l0Sensors[sensorIndex].readRangeContinuousMillimeters();
        timedOut = l0Sensors[sensorIndex].timeoutOccurred();
      } else {
        distanceMm = l1Sensors[sensorIndex].read();
        timedOut = l1Sensors[sensorIndex].timeoutOccurred();
      }

      ++readCount[sensorIndex];
      if (timedOut) {
        ++timeoutCount[sensorIndex];
        Serial.print("TO");
      } else {
        Serial.print(distanceMm);
        Serial.print("mm");
      }
      if (timeoutCount[sensorIndex] > 0) {
        Serial.print("(timeouts=");
        Serial.print(timeoutCount[sensorIndex]);
        Serial.print('/');
        Serial.print(readCount[sensorIndex]);
        Serial.print(')');
      }
    }

    if (sensorIndex + 1 < kSensorCount) {
      Serial.print("  ");
    }
  }
  Serial.println();
}

}  // namespace tof_diagnostic
