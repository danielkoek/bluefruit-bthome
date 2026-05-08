#include "BTHomeBLE_Bluefruit.h"

#ifdef BTHOME_BLUEFRUIT_AVAILABLE

bool BTHomeBLE_Bluefruit::init(const char* deviceName, int8_t txPower) {
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(txPower);
  Bluefruit.setName(deviceName);
  return true;
}

bool BTHomeBLE_Bluefruit::updateAdvertising(const uint8_t* serviceData,
                                            uint16_t len,
                                            uint16_t interval625us) {
  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
  }

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  if (!Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA, serviceData,
                                     len)) {
    return false;
  }

  Bluefruit.ScanResponse.addName();
  Bluefruit.ScanResponse.addTxPower();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(interval625us, interval625us);
  Bluefruit.Advertising.setFastTimeout(30);

  return Bluefruit.Advertising.start(0);
}

#endif  // BTHOME_BLUEFRUIT_AVAILABLE
