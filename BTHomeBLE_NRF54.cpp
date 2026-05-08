#include "BTHomeBLE_NRF54.h"

#ifdef BTHOME_NRF54_AVAILABLE

#include <string.h>

using namespace xiao_nrf54l15;

bool BTHomeBLE_NRF54::init(const char* deviceName, int8_t txPower) {
  m_name = deviceName;
  m_txPower = txPower;
  return true;
}

bool BTHomeBLE_NRF54::updateAdvertising(const uint8_t* serviceData, uint8_t len,
                                        uint16_t interval625us) {
  // Build raw AD payload: Flags + Service Data
  uint8_t advPayload[31];
  uint8_t pos = 0;

  // Flags AD structure
  advPayload[pos++] = 0x02;  // AD length
  advPayload[pos++] = 0x01;  // AD type: Flags
  advPayload[pos++] = 0x06;  // LE General Discoverable + BR/EDR Not Supported

  // Service Data AD structure
  if (pos + 2 + len > 31) return false;               // overflow guard
  advPayload[pos++] = static_cast<uint8_t>(len + 1);  // AD length (type + data)
  advPayload[pos++] = 0x16;  // AD type: Service Data - 16-bit UUID
  memcpy(&advPayload[pos], serviceData, len);
  pos += len;

  // Power up the RF path for transmission
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

  // Single advertising event on all 3 channels
  bool ok = m_ble.advertiseEvent(350, 700000);

  m_ble.end();
  BoardControl::collapseRfPathIdle();

  return ok;
}

#endif  // BTHOME_NRF54_AVAILABLE
