#pragma once

#include "BTHomeBLE.h"

// Auto-detect nRF54 availability
#ifdef __has_include
#if __has_include("nrf54l15_hal.h")
#define BTHOME_NRF54_AVAILABLE 1
#endif
#endif

#ifdef BTHOME_NRF54_AVAILABLE

#include "nrf54l15_hal.h"

class BTHomeBLE_NRF54 : public BTHomeBLE {
 public:
  bool init(const char* deviceName, int8_t txPower = 0) override;
  bool updateAdvertising(const uint8_t* serviceData, uint16_t len,
                         uint16_t interval625us = 160) override;
  bool supportsExtended() override { return true; }
  bool getAddress(uint8_t addr[6]) override;

 private:
  bool updateLegacy(const uint8_t* serviceData, uint16_t len);
  bool updateExtended(const uint8_t* serviceData, uint16_t len);

  xiao_nrf54l15::BleRadio m_ble;
  const char* m_name = nullptr;
  int8_t m_txPower = 0;
};

#endif  // BTHOME_NRF54_AVAILABLE
