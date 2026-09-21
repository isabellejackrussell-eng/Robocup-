#pragma once

#include <Arduino.h>

namespace debug {

class DebugPort : public Print {
 public:
  size_t write(uint8_t value) override;
  size_t write(const uint8_t* buffer, size_t size) override;
};

extern DebugPort Log;

void begin();
void printStartupBanner(const __FlashStringHelper* modeName);

}  // namespace debug

