#pragma once

#include "BTHomeBLE.h"

// Auto-detect Bluefruit availability
#ifdef __has_include
#if __has_include(<bluefruit.h>)
#define BTHOME_BLUEFRUIT_AVAILABLE 1
#endif
#endif

#ifdef BTHOME_BLUEFRUIT_AVAILABLE

#include <bluefruit.h>

class BTHomeBLE_Bluefruit : public BTHomeBLE {
 public:
  bool init(const char* deviceName, int8_t txPower = 0) override;
  bool updateAdvertising(const uint8_t* serviceData, uint16_t len,
                         uint16_t interval625us = 160) override;
  bool supportsExtended() override { return true; }
};

#endif  // BTHOME_BLUEFRUIT_AVAILABLE
