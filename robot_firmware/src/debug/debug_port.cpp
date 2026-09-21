#include "debug/debug_port.h"

#include "config/robot_config.h"

namespace debug {

DebugPort Log;

size_t DebugPort::write(uint8_t value) {
  Serial.write(value);
  Serial2.write(value);
  return 1;
}

size_t DebugPort::write(const uint8_t* buffer, size_t size) {
  Serial.write(buffer, size);
  Serial2.write(buffer, size);
  return size;
}

void begin() {
  Serial.begin(config::USB_SERIAL_BAUD);
  Serial2.begin(config::BLUETOOTH_SERIAL_BAUD);
}

void printStartupBanner(const __FlashStringHelper* modeName) {
  Log.println();
  Log.println(F("========================================"));
  Log.println(F("[BOOT] RoboCup robot firmware"));
  Log.print(F("[BOOT] Mode: "));
  Log.println(modeName);
  Log.println(F("[BOOT] Debug output: USB Serial + Bluetooth Serial2"));
  Log.println(F("========================================"));
}

}  // namespace debug
