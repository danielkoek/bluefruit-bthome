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
  // Legacy overhead: 3(flags) + 2(AD header) = 5; leaves 26 bytes for svc data
  if (len > 26) {
    return updateExtended(serviceData, len, interval625us);
  }
  return updateLegacy(serviceData, len, interval625us);
}

bool BTHomeBLE_Bluefruit::updateLegacy(const uint8_t* serviceData, uint16_t len,
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

bool BTHomeBLE_Bluefruit::updateExtended(const uint8_t* serviceData,
                                         uint16_t len, uint16_t interval625us) {
  // Stop any existing advertising on our extended handle
  if (m_extAdvHandle != BLE_GAP_ADV_SET_HANDLE_NOT_SET) {
    sd_ble_gap_adv_stop(m_extAdvHandle);
  }

  // Also stop legacy advertising if running
  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
  }

  // Build full AD payload: Flags + Service Data
  // MUST be static - SoftDevice reads from this buffer while advertising
  static uint8_t advPayload[BLE_GAP_ADV_SET_DATA_SIZE_EXTENDED_MAX_SUPPORTED];
  uint16_t pos = 0;

  // Flags AD structure
  advPayload[pos++] = 0x02;  // length
  advPayload[pos++] = 0x01;  // type: Flags
  advPayload[pos++] = 0x06;  // LE General Discoverable + BR/EDR Not Supported

  // Service Data AD structure
  if (pos + 2 + len > sizeof(advPayload)) return false;
  advPayload[pos++] = static_cast<uint8_t>(len + 1);  // AD length
  advPayload[pos++] = 0x16;  // type: Service Data - 16-bit UUID
  memcpy(&advPayload[pos], serviceData, len);
  pos += len;

  // Complete Local Name AD structure
  char name[32];
  uint8_t nameLen = Bluefruit.getName(name, sizeof(name));
  if (nameLen > 0 && pos + 2 + nameLen <= sizeof(advPayload)) {
    advPayload[pos++] = nameLen + 1;  // AD length
    advPayload[pos++] = 0x09;         // type: Complete Local Name
    memcpy(&advPayload[pos], name, nameLen);
    pos += nameLen;
  }

  // TX Power Level AD structure
  if (pos + 3 <= sizeof(advPayload)) {
    advPayload[pos++] = 0x02;  // AD length
    advPayload[pos++] = 0x0A;  // type: TX Power Level
    advPayload[pos++] = static_cast<uint8_t>(Bluefruit.getTxPower());
  }

  // Configure extended advertising via SoftDevice directly
  ble_gap_adv_params_t adv_params = {};
  adv_params.properties.type =
      BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
  adv_params.p_peer_addr = NULL;
  adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
  adv_params.interval = interval625us;
  adv_params.duration = 0;  // advertise indefinitely
  adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
  adv_params.secondary_phy = BLE_GAP_PHY_1MBPS;

  static ble_gap_adv_data_t gap_adv;
  gap_adv.adv_data.p_data = advPayload;
  gap_adv.adv_data.len = pos;
  gap_adv.scan_rsp_data.p_data = NULL;
  gap_adv.scan_rsp_data.len = 0;

  uint32_t err =
      sd_ble_gap_adv_set_configure(&m_extAdvHandle, &gap_adv, &adv_params);
  if (err != NRF_SUCCESS) return false;

  err = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_extAdvHandle,
                                Bluefruit.getTxPower());
  if (err != NRF_SUCCESS) return false;

  err = sd_ble_gap_adv_start(m_extAdvHandle, CONN_CFG_PERIPHERAL);
  return (err == NRF_SUCCESS);
}

bool BTHomeBLE_Bluefruit::getAddress(uint8_t addr[6]) {
  ble_gap_addr_t gap_addr;
  if (sd_ble_gap_addr_get(&gap_addr) != NRF_SUCCESS) return false;
  memcpy(addr, gap_addr.addr, 6);
  return true;
}

#endif  // BTHOME_BLUEFRUIT_AVAILABLE
