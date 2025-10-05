#include "BTHome.h"

void BTHome::begin(const char* device_name) {
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  // -40, -20, -16, -12, -8, -4, 0, +2, +3, +4, +5, +6, +7, +8
  Bluefruit.setTxPower(0);
  Bluefruit.setName(device_name);
}

void BTHome::resetMeasurement() {
  this->m_sensorDataIdx = 0;
  this->last_object_id = 0;
  this->m_sortEnable = false;
}

void BTHome::addMeasurement_state(uint8_t sensor_id, uint8_t state,
                                  uint8_t steps) {
  if ((this->m_sensorDataIdx + 2 + (steps > 0 ? 1 : 0)) <=
      MEASUREMENT_MAX_LEN - 0) {
    this->m_sensorData[this->m_sensorDataIdx] =
        static_cast<byte>(sensor_id & 0xff);
    this->m_sensorDataIdx++;
    this->m_sensorData[this->m_sensorDataIdx] = static_cast<byte>(state & 0xff);
    this->m_sensorDataIdx++;
    if (steps > 0) {
      this->m_sensorData[this->m_sensorDataIdx] =
          static_cast<byte>(steps & 0xff);
      this->m_sensorDataIdx++;
    }
    if (!this->m_sortEnable) {
      if (sensor_id < this->last_object_id) this->m_sortEnable = true;
    }
    last_object_id = sensor_id;
  } else {
    sendPacket();
    addMeasurement_state(sensor_id, state, steps);
  }
}

void BTHome::addMeasurement(uint8_t sensor_id, uint64_t value) {
  uint8_t size = getByteNumber(sensor_id);
  uint16_t factor = getFactor(sensor_id);
  if ((this->m_sensorDataIdx + size + 1) <= MEASUREMENT_MAX_LEN) {
    this->m_sensorData[this->m_sensorDataIdx] =
        static_cast<byte>(sensor_id & 0xff);
    this->m_sensorDataIdx++;
    for (uint8_t i = 0; i < size; i++) {
      this->m_sensorData[this->m_sensorDataIdx] =
          static_cast<byte>(((value * factor) >> (8 * i)) & 0xff);
      this->m_sensorDataIdx++;
    }
    if (!this->m_sortEnable) {
      if (sensor_id < this->last_object_id) this->m_sortEnable = true;
    }
    last_object_id = sensor_id;
  } else {
    sendPacket();
    addMeasurement(sensor_id, value);
  }
}

void BTHome::addMeasurement(uint8_t sensor_id, float value) {
  uint8_t size = getByteNumber(sensor_id);
  uint16_t factor = getFactor(sensor_id);
  if ((this->m_sensorDataIdx + size + 1) <= MEASUREMENT_MAX_LEN) {
    uint64_t value2 = static_cast<uint64_t>(value * factor);
    this->m_sensorData[this->m_sensorDataIdx] =
        static_cast<byte>(sensor_id & 0xff);
    this->m_sensorDataIdx++;
    for (uint8_t i = 0; i < size; i++) {
      this->m_sensorData[this->m_sensorDataIdx] =
          static_cast<byte>((value2 >> (8 * i)) & 0xff);
      this->m_sensorDataIdx++;
    }
    if (!this->m_sortEnable) {
      if (sensor_id < this->last_object_id) this->m_sortEnable = true;
    }
    last_object_id = sensor_id;
  } else {
    sendPacket();
    addMeasurement(sensor_id, value);
  }
}

// TEXT and RAW data
void BTHome::addMeasurement(uint8_t sensor_id, uint8_t* value, uint8_t size) {
  if ((this->m_sensorDataIdx + size + 1) <= MEASUREMENT_MAX_LEN) {
    // Add sensor id
    this->m_sensorData[this->m_sensorDataIdx] =
        static_cast<byte>(sensor_id & 0xff);
    this->m_sensorDataIdx++;
    // Add data size, 1 byte
    this->m_sensorData[this->m_sensorDataIdx] = static_cast<byte>(size & 0xff);
    this->m_sensorDataIdx++;
    // Add data bytes
    for (uint8_t i = 0; i < size; i++) {
      this->m_sensorData[this->m_sensorDataIdx] =
          static_cast<byte>(value[i] & 0xff);
      this->m_sensorDataIdx++;
    }
    if (!this->m_sortEnable) {
      if (sensor_id < this->last_object_id) this->m_sortEnable = true;
    }
    last_object_id = sensor_id;
  } else {
    sendPacket();
    addMeasurement(sensor_id, value, size);
  }
}
void BTHome::sendPacket() {
  if (this->m_sensorDataIdx > 0) {
    startAdv();
    resetMeasurement();
  }
}

void BTHome::sortSensorData() {
  uint8_t i, j, k, data_block_num;

  struct DATA_BLOCK {
    byte object_id;
    byte data[4];
    uint8_t data_len;
  };
  struct DATA_BLOCK data_block[MEASUREMENT_MAX_LEN / 2 + 1];
  struct DATA_BLOCK temp_data_block;

  for (i = 0, j = 0, data_block_num = 0; j < this->m_sensorDataIdx; i++) {
    // copy the object id
    data_block[i].object_id = this->m_sensorData[j];
    data_block_num++;
    // copy the data length
    if (this->m_sensorData[j] == EVENT_DIMMER) {
      if (this->m_sensorData[j + 1] == EVENT_DIMMER_NONE)
        data_block[i].data_len = 1;
      else
        data_block[i].data_len = 2;
    } else {
      data_block[i].data_len = getByteNumber(this->m_sensorData[j]);
    }
    // copy the data
    for (k = 0; k < data_block[i].data_len; k++) {
      data_block[i].data[k] = this->m_sensorData[j + 1 + k];
    }
    // move to the next object id location
    j = j + data_block[i].data_len + 1;
  }

  if (data_block_num > 1) {
    // bubble sort
    for (i = 0; i < data_block_num - 1; i++) {
      for (j = 0; j < data_block_num - 1 - i; j++) {
        if (data_block[j].object_id > data_block[j + 1].object_id) {
          memcpy(&temp_data_block, &data_block[j], sizeof(struct DATA_BLOCK));
          memcpy(&data_block[j], &data_block[j + 1], sizeof(struct DATA_BLOCK));
          memcpy(&data_block[j + 1], &temp_data_block,
                 sizeof(struct DATA_BLOCK));
        }
      }
    }
    // copy the new order to m_sensorData array
    for (i = 0, j = 0; i < data_block_num && j < this->m_sensorDataIdx; i++) {
      this->m_sensorData[j] = data_block[i].object_id;
      for (k = 0; k < data_block[i].data_len; k++) {
        this->m_sensorData[j + 1 + k] = data_block[i].data[k];
      }
      j = j + data_block[i].data_len + 1;
    }
  }
}

void BTHome::startAdv() {
  if (this->m_sortEnable) sortSensorData();
  struct ATTR_PACKED {
    uint16_t bthome_uuid;
    uint8_t encrypt;
    uint8_t sensor_measurements[MEASUREMENT_MAX_LEN];
  } bthome = {
      .bthome_uuid = UUID16_SVC_BTHOME,
      .encrypt = NO_ENCRYPT,
  };
  uint8_t len = 0;
  // Add the sensor data to the Service Data
  for (uint8_t i = 0; i < this->m_sensorDataIdx; i++) {
    bthome.sensor_measurements[len++] = this->m_sensorData[i];
  }
  // Print the data to Serial in HEX format
  Serial.print("Data lenght:");
  Serial.println(len);
  Serial.print("Service Data: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print("0x");
    if (bthome.sensor_measurements[i] < 0x10)
      Serial.print("0");  // pad single digit hex
    Serial.print(bthome.sensor_measurements[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  if (Bluefruit.Advertising.isRunning()) {
    Serial.println("Stopping current advertising");
    if (Bluefruit.Advertising.stop()) {
      Serial.println("Stopped successfully");
      Bluefruit.Advertising.clearData();
      Bluefruit.ScanResponse.clearData();
    }
  }
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  if (!Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA, &bthome,
                                     len + 3)) {
    Serial.println("Couldn't add serviceData");
  }
  Bluefruit.ScanResponse.addName();
  Bluefruit.ScanResponse.addTxPower();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);
  uint8_t* data = Bluefruit.Advertising.getData();
  uint16_t count = Bluefruit.Advertising.count();
  Serial.print("TotalLength: ");
  Serial.println(count);
  Serial.print("Raw: ");
  for (uint8_t i = 0; i < count; i++) {
    Serial.print("0x");
    if (data[i] < 0x10) Serial.print("0");  // pad single digit hex
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  if (Bluefruit.Advertising.start(0)) {
    Serial.println("Started successfully");
  } else {
    Serial.println("Couldn't start");
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
  }
  //}
}

uint8_t BTHome::getByteNumber(uint8_t sens) {
  switch (sens) {
    case ID_PACKET:
    case ID_BATTERY:
    case ID_COUNT:
    case ID_HUMIDITY:
    case ID_MOISTURE:
    case ID_UV:
    case STATE_BATTERY_LOW:
    case STATE_BATTERY_CHARHING:
    case STATE_CO:
    case STATE_COLD:
    case STATE_CONNECTIVITY:
    case STATE_DOOR:
    case STATE_GARAGE_DOOR:
    case STATE_GAS_DETECTED:
    case STATE_GENERIC_BOOLEAN:
    case STATE_HEAT:
    case STATE_LIGHT:
    case STATE_LOCK:
    case STATE_MOISTURE:
    case STATE_MOTION:
    case STATE_MOVING:
    case STATE_OCCUPANCY:
    case STATE_OPENING:
    case STATE_PLUG:
    case STATE_POWER_ON:
    case STATE_PRESENCE:
    case STATE_PROBLEM:
    case STATE_RUNNING:
    case STATE_SAFETY:
    case STATE_SMOKE:
    case STATE_SOUND:
    case STATE_TAMPER:
    case STATE_VIBRATION:
    case STATE_WINDOW:
    case EVENT_BUTTON:
      return 1;
      break;
    case ID_DURATION:
    case ID_ENERGY:
    case ID_GAS:
    case ID_ILLUMINANCE:
    case ID_POWER:
    case ID_PRESSURE:
      return 3;
      break;
    case ID_COUNT4:
    case ID_ENERGY4:
    case ID_GAS4:
    case ID_VOLUME:
    case ID_WATER:
    case ID_TIMESTAMP:
      return 4;
      break;
    default:
      return 2;
  }
}

uint16_t BTHome::getFactor(uint8_t sens) {
  switch (sens) {
    case ID_DISTANCEM:
    case ID_ROTATION:
    case ID_TEMPERATURE:
    case ID_VOLTAGE1:
    case ID_VOLUME1:
    case ID_UV:
      return 10;
      break;
    case ID_DEWPOINT:
    case ID_HUMIDITY_PRECISE:
    case ID_ILLUMINANCE:
    case ID_MASS:
    case ID_MASSLB:
    case ID_MOISTURE_PRECISE:
    case ID_POWER:
    case ID_PRESSURE:
    case ID_SPD:
    case ID_TEMPERATURE_PRECISE:
      return 100;
      break;
    case ID_CURRENT:
    case ID_DURATION:
    case ID_ENERGY:
    case ID_ENERGY4:
    case ID_GAS:
    case ID_GAS4:
    case ID_VOLTAGE:
    case ID_VOLUME:
    case ID_VOLUMEFR:
    case ID_WATER:
      return 1000;
      break;
    default:
      return 1;
  }
}
