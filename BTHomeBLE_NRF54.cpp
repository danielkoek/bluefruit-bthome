#include "BTHomeBLE_NRF54.h"

#ifdef BTHOME_NRF54_AVAILABLE

#include <string.h>

using namespace xiao_nrf54l15;

bool BTHomeBLE_NRF54::init(const char* deviceName, int8_t txPower) {
  m_name = deviceName;
  m_txPower = txPower;
  return true;
}

bool BTHomeBLE_NRF54::updateAdvertising(const uint8_t* serviceData,
                                        uint16_t len, uint16_t interval625us) {
  // Use extended advertising if payload won't fit in legacy 31 bytes
  // Legacy overhead: 3(flags) + 2(AD header) = 5; leaves 26 for service data
  if (len > 26) {
    return updateExtended(serviceData, len);
  }
  return updateLegacy(serviceData, len);
}

bool BTHomeBLE_NRF54::updateLegacy(const uint8_t* serviceData, uint16_t len) {
  uint8_t advPayload[31];
  uint8_t pos = 0;

  // Flags AD structure
  advPayload[pos++] = 0x02;  // AD length
  advPayload[pos++] = 0x01;  // AD type: Flags
  advPayload[pos++] = 0x06;  // LE General Discoverable + BR/EDR Not Supported

  // Service Data AD structure
  if (pos + 2 + len > 31) return false;
  advPayload[pos++] = static_cast<uint8_t>(len + 1);
  advPayload[pos++] = 0x16;  // AD type: Service Data - 16-bit UUID
  memcpy(&advPayload[pos], serviceData, len);
  pos += len;

  BoardControl::enableRfPath(BoardAntennaPath::kCeramic);

  if (!m_ble.begin(m_txPower)) {
    BoardControl::collapseRfPathIdle();
    return false;
  }

  m_ble.loadAddressFromFicr(true);
  m_ble.setAdvertisingPduType(BleAdvPduType::kAdvNonConnInd);

  if (!m_ble.setAdvertisingData(advPayload, pos)) {
    m_ble.end();
    BoardControl::collapseRfPathIdle();
    return false;
  }

  bool ok = m_ble.advertiseEvent(350, 700000);
  m_ble.end();
  BoardControl::collapseRfPathIdle();
  return ok;
}

bool BTHomeBLE_NRF54::updateExtended(const uint8_t* serviceData, uint16_t len) {
  // Extended AD payload: Flags + Service Data (up to 255 bytes total)
  uint8_t advPayload[255];
  uint8_t pos = 0;

  // Flags AD structure
  advPayload[pos++] = 0x02;
  advPayload[pos++] = 0x01;
  advPayload[pos++] = 0x06;

  // Service Data AD structure
  if (pos + 2 + len > 255) return false;
  advPayload[pos++] = static_cast<uint8_t>(len + 1);
  advPayload[pos++] = 0x16;
  memcpy(&advPayload[pos], serviceData, len);
  pos += len;

  BoardControl::enableRfPath(BoardAntennaPath::kCeramic);

  if (!m_ble.begin(m_txPower)) {
    BoardControl::collapseRfPathIdle();
    return false;
  }

  m_ble.loadAddressFromFicr(true);

  if (!m_ble.setExtendedAdvertisingData(advPayload, pos)) {
    m_ble.end();
    BoardControl::collapseRfPathIdle();
    return false;
  }

  bool ok = m_ble.advertiseExtendedEvent(3000, 350, 700000);
  m_ble.end();
  BoardControl::collapseRfPathIdle();
  return ok;
}

bool BTHomeBLE_NRF54::getAddress(uint8_t addr[6]) {
  return m_ble.getDeviceAddress(addr);
}

#endif  // BTHOME_NRF54_AVAILABLE
