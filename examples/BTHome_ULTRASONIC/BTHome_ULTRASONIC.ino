#include <OneWire.h>
#include <DallasTemperature.h>
#include "BTHome.h"

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS D9
#define VBAT_PIN A1

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// Ultrasonic, to measure the water in te tank
char device_name[] = "ULTRASONIC";

BTHome bthome;
// Helper for consistent output
void printToSerial(const String& msg) {
  Serial.println(msg);
}
int readJSNSR04T() {
  Serial1.write(0x55);  // start ping. call before readSerial(). in 19==47Kohm mode.
  byte serialData[4];
  int distance = -1;
  if (Serial1.available() >= 4) {
    int bytesRead = Serial1.readBytes(serialData, 4);
    int sum = (serialData[0] + serialData[1] + serialData[2]) & 0x00FF;
    if (serialData[3] == sum) {
      distance = (((int)serialData[1] << 8) + serialData[2]);
    }
  }
  return distance;
}
float readBattery(void) {
  float raw;
  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);
  return raw * (2.0F * 0.73242188F);  // Compensation factor for the VBAT divider (2* 220k resistors), 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
}

uint8_t battteryToPercent(float mvolts) {
  if (mvolts < 3300)
    return 0;

  if (mvolts < 3600) {
    mvolts -= 3300;
    return mvolts / 30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F);  // thats mvolts /6.66666666
}

void setup() {
  Serial1.begin(9600, SERIAL_8N1);
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12);  // Can be 8, 10, 12 or 14
  // Start up the library
  sensors.begin();
  // Enable DC-DC converter
  NRF_POWER->DCDCEN = 1;

  bthome.begin(device_name);
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature is: ");
    Serial.print(tempC);
    Serial.println("°C");
    bthome.addMeasurement(ID_TEMPERATURE, tempC);  // 3
  }
  float vbat_mv = readBattery();
  uint8_t vbat_per = battteryToPercent(vbat_mv);
  float batteryV = vbat_mv / 1000.0F;
  Serial.print("LIPO = ");
  Serial.print(batteryV);
  Serial.print(" V (");
  Serial.print(vbat_per);
  Serial.println("%)");
  bthome.addMeasurement(ID_VOLTAGE, batteryV);            // 3
  bthome.addMeasurement(ID_BATTERY, (uint64_t)vbat_per);  // 3
  int distance = readJSNSR04T();

  if (distance != -1) {
    Serial.print("Distance is ");
    Serial.print(distance);
    Serial.println(" mm ");
    bthome.addMeasurement(ID_DISTANCE, (uint64_t)distance);  // 3
  }
  bthome.sendPacket();
  __WFE();
  __WFI();
  delay(10000);  // advertising period = 10s
}
