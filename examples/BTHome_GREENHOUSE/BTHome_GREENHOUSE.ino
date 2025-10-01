#include "BTHome.h"
#include "Adafruit_SHT4x.h"
#include <RTClib.h>
#include <Adafruit_INA3221.h>
const float maxBatteryVoltage = 12.6;  // Full 3S LiPo
const float minBatteryVoltage = 9.0;   // Empty 3S LiPo
const int BATTERY_CUTOFF_PERCENT = 15;

#define RELAY_PIN D8
// #define MANUAL_BUTTON D10
#define LOAD_CHANNEL 0
// #define SOLAR_IN 1
// #define TO_BATTERY 2
#define TEMP_HUM
#define MAX_RUN_LOGS 144
#define MAX_POWER_LOGS 50
#define DELAY_MS 3000        //
#define HOW_LONG_MANUAL 100  // HOW_LONG_MANUAL*DELAY_MS, for now 300s, or 5 min
struct Schedule {
  bool enabled;
  uint8_t startHour;
  uint8_t startMinute;
  uint8_t endHour;
  uint8_t endMinute;
};
// Greenhouse FAN
char device_name[] = "MINI-GREENHOUSE-FAN";
const int programLength = 24;
int manualOverride = 100;
Schedule programs[programLength] = {
  { false, 0, 0, 0, 10 },    // 12 AM to 12:10 AM
  { false, 1, 0, 1, 10 },    // 1 AM to 1:10 AM
  { false, 2, 0, 2, 10 },    // 2 AM to 2:10 AM
  { false, 3, 0, 3, 10 },    // 3 AM to 3:10 AM
  { false, 4, 0, 4, 20 },    // 4 AM to 4:20 AM
  { false, 5, 0, 5, 10 },    //  5 AM to 5:10 AM
  { false, 6, 0, 6, 10 },    // 6 AM to 6:10 AM
  { false, 7, 0, 7, 10 },    // 7 AM to 7:10 AM
  { false, 8, 0, 8, 10 },    // 8 AM to 8:10 AM
  { false, 9, 0, 9, 10 },    // 9 AM to 9:10 AM
  { false, 10, 0, 10, 10 },  // 10 AM to 10:10 AM
  { false, 11, 0, 11, 10 },  // 11 AM to 11:10 AM
  { true, 12, 0, 12, 10 },   // 12 PM to 12:10 PM
  { true, 13, 0, 13, 10 },   // 1 PM to 1:10 PM
  { true, 14, 0, 14, 10 },   // 2 PM to 2:10 PM
  { false, 15, 0, 15, 10 },  // 3 PM to 3:10 PM
  { false, 16, 0, 16, 10 },  // 4 PM to 4:10 PM
  { false, 17, 0, 17, 15 },  // 5 PM to 5:15 PM
  { false, 18, 0, 18, 10 },  // 6 PM to 6:10 PM
  { false, 19, 0, 19, 10 },  // 7 PM to 7:10 PM
  { false, 20, 0, 20, 10 },  // 8 PM to 8:10 PM
  { false, 21, 0, 21, 10 },  // 9 PM to 9:10 PM
  { false, 22, 0, 22, 10 },  // 10 PM to 10:10 PM
  { false, 23, 0, 23, 10 }   // 11 PM to 11:10 PM
};
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
RTC_DS3231 rtc;
Adafruit_INA3221 ina3221;
BTHome bthome;
// Helper for consistent output
void printToSerial(const String& msg) {
  Serial.println(msg);
}

void checkI2CPorts() {
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found\n");
}

unsigned long lastISRmillis = 0L;
void ISR_button_pressed(void) {
  if (millis() > lastISRmillis + 200)  // 200 mS debounce time
  {
    if (manualOverride != 0) {
      manualOverride = 0;
      digitalWrite(RELAY_PIN, LOW);
    } else {
      manualOverride = 100;
      digitalWrite(RELAY_PIN, HIGH);
    }
    lastISRmillis = millis();
  }
}
// Returns 0 if currently within schedule, >0 if minutes until next start, -1 if
// not scheduled today
int isWithinSchedule(const DateTime& now, const Schedule& s) {
  int nowMin = now.hour() * 60 + now.minute();
  int startMin = s.startHour * 60 + s.startMinute;
  int endMin = s.endHour * 60 + s.endMinute;
  if (nowMin >= startMin && nowMin < endMin) {
    return 0;  // currently running
  } else if (nowMin < startMin) {
    return startMin - nowMin;  // minutes until next start
  } else {
    return -1;  // already passed for today
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  // setup relay
  pinMode(RELAY_PIN, OUTPUT);
  // default to high
  digitalWrite(RELAY_PIN, HIGH);

  Wire.begin();
  rtc.begin();
  // Get build time as DateTime
  DateTime buildTime(F(__DATE__), F(__TIME__));
  DateTime now = rtc.now();

  // If RTC lost power OR RTC time is before build time, set RTC to build time
  if (rtc.lostPower() || now < buildTime) {
    rtc.adjust(buildTime);
  }
  ina3221.begin(0x40, &Wire);
#ifdef TEMP_HUM
  sht4.begin();
  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
    case SHT4X_HIGH_PRECISION:
      Serial.println("High precision");
      break;
    case SHT4X_MED_PRECISION:
      Serial.println("Med precision");
      break;
    case SHT4X_LOW_PRECISION:
      Serial.println("Low precision");
      break;
  }

  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_NO_HEATER);
  switch (sht4.getHeater()) {
    case SHT4X_NO_HEATER:
      Serial.println("No heater");
      break;
    case SHT4X_HIGH_HEATER_1S:
      Serial.println("High heat for 1 second");
      break;
    case SHT4X_HIGH_HEATER_100MS:
      Serial.println("High heat for 0.1 second");
      break;
    case SHT4X_MED_HEATER_1S:
      Serial.println("Medium heat for 1 second");
      break;
    case SHT4X_MED_HEATER_100MS:
      Serial.println("Medium heat for 0.1 second");
      break;
    case SHT4X_LOW_HEATER_1S:
      Serial.println("Low heat for 1 second");
      break;
    case SHT4X_LOW_HEATER_100MS:
      Serial.println("Low heat for 0.1 second");
      break;
  }
#endif

  bthome.begin(device_name);
}

void loop() {
  // 1st method: just addMeasurement as much as you can, the code will split and send the adv packet automatically
  // each adv packet sending lasts for 1500ms
  bthome.resetMeasurement();
#ifdef SOLAR_IN
  float solar_in_voltage = ina3221.getBusVoltage(SOLAR_IN);
  float solar_in_current = ina3221.getCurrentAmps(SOLAR_IN);
  bthome.addMeasurement(ID_VOLTAGE, solar_in_voltage);  //3
  bthome.addMeasurement(ID_CURRENT, solar_in_current);  //3
#endif
#ifdef TO_BATTERY
  float to_battery_voltage = ina3221.getBusVoltage(TO_BATTERY);
  float to_battery_current = ina3221.getCurrentAmps(TO_BATTERY);
  bthome.addMeasurement(ID_VOLTAGE, to_battery_voltage);  //3
  bthome.addMeasurement(ID_CURRENT, to_battery_current);  //3
#endif
  float load_voltage = ina3221.getBusVoltage(LOAD_CHANNEL);
  float load_current = ina3221.getCurrentAmps(LOAD_CHANNEL);
  float batteryVoltage = load_voltage;
  batteryVoltage = constrain(batteryVoltage, minBatteryVoltage, maxBatteryVoltage);
  int batteryPercent = map(batteryVoltage * 100, minBatteryVoltage * 100, maxBatteryVoltage * 100, 0, 100);
  bthome.addMeasurement(ID_BATTERY, (uint64_t)batteryPercent);  //3
  bthome.addMeasurement(ID_VOLTAGE, load_voltage);              //3
  bthome.addMeasurement(ID_CURRENT, load_current);              //3

#ifdef TEMP_HUM
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);
  bthome.addMeasurement(ID_TEMPERATURE, temp.temperature);         //3
  bthome.addMeasurement(ID_HUMIDITY, humidity.relative_humidity);  //3
#endif


  bool relayStatus = digitalRead(RELAY_PIN);
  bthome.addMeasurement_state(STATE_POWER_ON, relayStatus);  //2
  // make sure you sent it anyway
  bthome.sendPacket();

  bool shouldBeOn = false;
  DateTime now = rtc.now();
  for (int i = 0; i < programLength; i++) {
    if (!programs[i].enabled) continue;
    int sched = isWithinSchedule(now, programs[i]);
    if (sched == 0) {
      shouldBeOn = true;
      break;
    }
  }
  if (relayStatus != shouldBeOn) {
    printToSerial("Schedule change, going to go to " + String(shouldBeOn));
    if (manualOverride > 0) {
    } else {
      digitalWrite(RELAY_PIN, shouldBeOn);
    }
  }
  if (manualOverride > 0) {
    printToSerial("Manual on is on for: " + String(manualOverride * DELAY_MS) + "S");
  }
  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));

  if (manualOverride > 0) {
    // makes it always on if you disable this line
    //manualOverride--;
  }
  delay(DELAY_MS);  // Check every 3 seconds
}

//Object ids by order
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
