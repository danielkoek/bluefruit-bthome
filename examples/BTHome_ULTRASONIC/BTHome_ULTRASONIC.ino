#include <OneWire.h>
#include <DallasTemperature.h>
#include "BTHome.h"

#define DELAY_MS 3000
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
  pinMode(PIN_LED, OUTPUT);
  Serial1.begin(9600, SERIAL_8N1);
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12);  // Can be 8, 10, 12 or 14
  // Start up the library
  sensors.begin();
  bthome.begin(device_name);
}

void loop() {
  digitalWrite(PIN_LED, HIGH);
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

  digitalWrite(PIN_LED, LOW);

  delay(DELAY_MS);  // Check every 3 seconds
}

// Object ids by order
#if 0
#define ID_PACKET 0x00
#define ID_BATTERY 0x01
#define ID_TEMPERATURE_PRECISE 0x02
#define ID_HUMIDITY_PRECISE 0x03
#define ID_PRESSURE 0x04
#define ID_ILLUMINANCE 0x05
#define ID_MASS 0x06
#define ID_MASSLB 0x07
#define ID_DEWPOINT 0x08
#define ID_COUNT 0x09
#define ID_ENERGY 0x0A
#define ID_POWER 0x0B
#define ID_VOLTAGE 0x0C
#define ID_PM25 0x0D
#define ID_PM10 0x0E
#define STATE_GENERIC_BOOLEAN 0x0F
#define STATE_POWER_ON 0x10
#define STATE_OPENING 0x11
#define ID_CO2 0x12
#define ID_TVOC 0x13
#define ID_MOISTURE_PRECISE 0x14
#define STATE_BATTERY_LOW 0x15
#define STATE_BATTERY_CHARHING 0x16
#define STATE_CO 0x17
#define STATE_COLD 0x18
#define STATE_CONNECTIVITY 0x19
#define STATE_DOOR 0x1A
#define STATE_GARAGE_DOOR 0x1B
#define STATE_GAS_DETECTED 0x1C
#define STATE_HEAT 0x1D
#define STATE_LIGHT 0x1E
#define STATE_LOCK 0x1F
#define STATE_MOISTURE 0x20
#define STATE_MOTION 0x21
#define STATE_MOVING 0x22
#define STATE_OCCUPANCY 0x23
#define STATE_PLUG 0x24
#define STATE_PRESENCE 0x25
#define STATE_PROBLEM 0x26
#define STATE_RUNNING 0x27
#define STATE_SAFETY 0x28
#define STATE_SMOKE 0x29
#define STATE_SOUND 0x2A
#define STATE_TAMPER 0x2B
#define STATE_VIBRATION 0x2C
#define STATE_WINDOW 0x2D
#define ID_HUMIDITY 0x2E
#define ID_MOISTURE 0x2F
#define EVENT_BUTTON 0x3A
#define EVENT_DIMMER 0x3C
#define ID_COUNT2 0x3D
#define ID_COUNT4 0x3E
#define ID_ROTATION 0x3F
#define ID_DISTANCE 0x40
#define ID_DISTANCEM 0x41
#define ID_DURATION 0x42
#define ID_CURRENT 0x43
#define ID_SPD 0x44
#define ID_TEMPERATURE 0x45
#define ID_UV 0x46
#define ID_VOLUME1 0x47
#define ID_VOLUME2 0x48
#define ID_VOLUMEFR 0x49
#define ID_VOLTAGE1 0x4A
#define ID_GAS 0x4B
#define ID_GAS4 0x4C
#define ID_ENERGY4 0x4D
#define ID_VOLUME 0x4E
#define ID_WATER 0x4F
#endif
