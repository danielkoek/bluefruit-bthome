#pragma once

#include <stdint.h>

// Max measurement payload sizes
#define BTHOME_LEGACY_MAX_LEN \
  23  // 31 - 3(flags) - 2(AD hdr) - 2(UUID) - 1(encrypt)
#define BTHOME_EXTENDED_MAX_LEN \
  248  // 255 - 2(AD hdr) - 2(UUID) - 1(encrypt) - 2(safety)

// Abstract BLE backend interface for BTHome advertising.
// Implement this class to add support for a new BLE platform.
class BTHomeBLE {
 public:
  virtual ~BTHomeBLE() {}

  // Initialize BLE stack with device name and TX power.
  // txPower values (dBm): -40, -20, -16, -12, -8, -4, 0, +2, +3, +4, +5, +6,
  // +7, +8
  virtual bool init(const char* deviceName, int8_t txPower = 0) = 0;

  // Update advertising payload and (re)start advertising.
  // serviceData: raw BTHome service data [UUID16_lo, UUID16_hi, device_info,
  // measurements...]
  // len: total byte length of serviceData
  // interval625us: advertising interval in units of 0.625 ms (default 160 = 100
  // ms)
  virtual bool updateAdvertising(const uint8_t* serviceData, uint16_t len,
                                 uint16_t interval625us = 160) = 0;

  // Returns true if backend supports extended advertising (BLE 5.0+)
  virtual bool supportsExtended() { return false; }
};
