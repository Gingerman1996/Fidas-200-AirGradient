/**
 * Example that reads data from the Sensirion SEN66 sensor and mirrors the
 * values into the AirGradient Measurements structure so other components can
 * reuse the output just like the main firmware. เพิ่มเติมด้วยการอัปเดตหน้าจอ
 * OLED และแถบไฟ LED ให้คล้ายกับตัวอย่างหลัก OneOpenAir
 */

#include <Arduino.h>
#include <SensirionI2cSen66.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>

#include "AgSchedule.h"
#include "AgValue.h"
#include "AirGradient.h"

#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 6
#define OLED_I2C_ADDR 0x3C

#define SEN66_READ_INTERVAL_MS 1000
#define LOG_INTERVAL_MS 2000
#define TIME_TO_CALIBRATE_CO2_MS 60000

static SensirionI2cSen66 sen66;
static Measurements measurements;
static AirGradient *ag = nullptr;
static bool hasDisplay = false;
static bool hasLedBar = false;

struct Sen66Snapshot {
  float pm1 = NAN;
  float pm25 = NAN;
  float pm4 = NAN;
  float pm10 = NAN;
  float humidity = NAN;
  float temperature = NAN;
  float vocIndex = NAN;
  float noxIndex = NAN;
  uint16_t co2 = 0;
  uint32_t timestamp = 0;
  bool valid = false;
};

static Sen66Snapshot latestSample;

static char errorMessage[64];
static int16_t errorCode = NO_ERROR;
static bool sensorReady = false;
static uint32_t sensorStartTime = 0;
static char sen66ErrorMessage[64];

static void readSen66(void);
static void printMeasurements(void);
static void initSen66(void);
static void initAirGradient(void);
static void updateVisuals(void);
static void updateDisplay(void);
static void updateLedBar(void);

static AgSchedule readSchedule(SEN66_READ_INTERVAL_MS, readSen66);
static AgSchedule logSchedule(LOG_INTERVAL_MS, printMeasurements);

static void initAirGradient(void) {
  BoardType detectedBoard = BoardType::OPEN_AIR_OUTDOOR;

  Wire.beginTransmission(OLED_I2C_ADDR);
  if (Wire.endTransmission() == 0x00) {
    detectedBoard = BoardType::ONE_INDOOR;
    hasDisplay = true;
  }

  ag = new AirGradient(detectedBoard);

  if (hasDisplay) {
    ag->display.begin(Wire);
    ag->display.clear();
    ag->display.setTextColor(ag->display.COLOR_WHILTE);
    ag->display.setTextSize(1);
    ag->display.setCursor(0, 0);
    ag->display.setText("SEN66 monitor\n"
                        "Waiting data...");
    ag->display.show();
  }

  ag->ledBar.begin();
  hasLedBar = (ag->ledBar.getNumberOfLeds() > 0);
  if (hasLedBar) {
    ag->ledBar.setEnable(true);
    ag->ledBar.setBrightness(40); // ค่าเริ่มต้นที่ไม่สว่างจนเกินไป
    ag->ledBar.clear();
    ag->ledBar.show();
  }

  Serial.print(F("Detected board type: "));
  Serial.println(ag->getBoardName());
}

static void initSen66(void) {
  sen66.begin(Wire, SEN66_I2C_ADDR_6B);

  errorCode = sen66.deviceReset();
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print(F("Error: deviceReset() -> "));
    Serial.println(errorMessage);
    return;
  }

  delay(1200);

  errorCode = sen66.startContinuousMeasurement();
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print(F("Error: startContinuousMeasurement() -> "));
    Serial.println(errorMessage);
    return;
  }

  sensorReady = true;
  Serial.println(F("SEN66 ready"));

  if (hasDisplay) {
    ag->display.clear();
    ag->display.setCursor(0, 0);
    ag->display.setText("SEN66 ready\nCollecting data...");
    ag->display.show();
  }
}

static void readSen66(void) {
  if (!sensorReady) {
    return;
  }

  float massConcentrationPm1p0 = NAN;
  float massConcentrationPm2p5 = NAN;
  float massConcentrationPm4p0 = NAN;
  float massConcentrationPm10p0 = NAN;
  float relativeHumidity = NAN;
  float ambientTemperature = NAN;
  float vocIndex = NAN;
  float noxIndex = NAN;
  uint16_t co2 = 0;

  errorCode = sen66.readMeasuredValues(
      massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
      massConcentrationPm10p0, relativeHumidity, ambientTemperature, vocIndex,
      noxIndex, co2);
  if (errorCode != NO_ERROR) {
    errorToString(errorCode, errorMessage, sizeof(errorMessage));
    Serial.print(F("Error: readMeasuredValues() -> "));
    Serial.println(errorMessage);
    return;
  }

  latestSample.pm1 = massConcentrationPm1p0;
  latestSample.pm25 = massConcentrationPm2p5;
  latestSample.pm4 = massConcentrationPm4p0;
  latestSample.pm10 = massConcentrationPm10p0;
  latestSample.humidity = relativeHumidity;
  latestSample.temperature = ambientTemperature;
  latestSample.vocIndex = vocIndex;
  latestSample.noxIndex = noxIndex;
  latestSample.co2 = co2;
  latestSample.timestamp = millis();
  latestSample.valid = true;

  measurements.pm01_1 = lroundf(massConcentrationPm1p0);
  measurements.pm25_1 = lroundf(massConcentrationPm2p5);
  measurements.pm10_1 = lroundf(massConcentrationPm10p0);
  measurements.pm03PCount_1 = -1;

  measurements.Temperature = ambientTemperature;
  measurements.Humidity = lroundf(relativeHumidity);

  measurements.TVOC = lroundf(vocIndex);
  measurements.NOx = lroundf(noxIndex);
  measurements.CO2 = co2;

  updateVisuals();
}

static void updateVisuals(void) {
  updateDisplay();
  updateLedBar();
}

static void calibrateCO2Sensor(void) {
  if (!sensorReady) {
    Serial.println("CO2 calibration requested but SEN66 sensor not ready");
  } else {
    sensorReady = false;
    delay(2000);

    uint16_t correction = 0;
    int16_t stopErr = sen66.stopMeasurement();
    if (stopErr != NO_ERROR) {
      errorToString((uint16_t)stopErr, sen66ErrorMessage,
                    sizeof(sen66ErrorMessage));
      Serial.print("SEN66 stopMeasurement failed: ");
      Serial.println(sen66ErrorMessage);
    } else {
      delay(2000);
      Serial.println("Stopped SEN66 measurement for CO2 calibration");
      int16_t frcErr = sen66.performForcedCo2Recalibration(1000, correction);
      if (frcErr != NO_ERROR) {
        errorToString((uint16_t)frcErr, sen66ErrorMessage,
                      sizeof(sen66ErrorMessage));
        Serial.print("SEN66 forced calibration failed: ");
        Serial.println(sen66ErrorMessage);
      } else if (correction == 0xFFFF) {
        Serial.println("SEN66 forced calibration returned invalid correction");
      } else {
        Serial.println("SEN66 forced CO2 calibration success");
        Serial.print("SEN66 correction: ");
        Serial.println(correction);
      }
    }

    int16_t resetErr = sen66.deviceReset();
    if (resetErr != NO_ERROR) {
      errorToString((uint16_t)resetErr, sen66ErrorMessage,
                    sizeof(sen66ErrorMessage));
      Serial.print("SEN66 deviceReset failed: ");
      Serial.println(sen66ErrorMessage);
    } else {
      delay(1200);
      int16_t startErr = sen66.startContinuousMeasurement();
      if (startErr != NO_ERROR) {
        errorToString((uint16_t)startErr, sen66ErrorMessage,
                      sizeof(sen66ErrorMessage));
        Serial.print("SEN66 startMeasurement failed: ");
        Serial.println(sen66ErrorMessage);
      } else {
        sensorReady = true;
        Serial.println("SEN66 restarted measurement after CO2 calibration");
      }
      delay(2000);
    }
  }
}
static void updateDisplay(void) {
  if (!hasDisplay || ag == nullptr) {
    return;
  }

  ag->display.clear();
  ag->display.setTextSize(1);
  ag->display.setTextColor(ag->display.COLOR_WHILTE);

  if (!latestSample.valid) {
    ag->display.setCursor(0, 0);
    ag->display.setText("Waiting SEN66\nmeasurements...");
    ag->display.show();
    return;
  }

  char line[32];
  const int lineHeight = 12;
  int y = 0;

  snprintf(line, sizeof(line), "PM1 %.1f PM2.5 %.1f", latestSample.pm1,
           latestSample.pm25);
  ag->display.setCursor(0, y);
  ag->display.setText(line);

  y += lineHeight;
  snprintf(line, sizeof(line), "PM4 %.1f PM10 %.1f", latestSample.pm4,
           latestSample.pm10);
  ag->display.setCursor(0, y);
  ag->display.setText(line);

  y += lineHeight;
  snprintf(line, sizeof(line), "T %.1fC  RH %.1f%%", latestSample.temperature,
           latestSample.humidity);
  ag->display.setCursor(0, y);
  ag->display.setText(line);

  y += lineHeight;
  snprintf(line, sizeof(line), "TVOC %.0f NOx %.0f", latestSample.vocIndex,
           latestSample.noxIndex);
  ag->display.setCursor(0, y);
  ag->display.setText(line);

  y += lineHeight;
  snprintf(line, sizeof(line), "CO2 %uppm", latestSample.co2);
  ag->display.setCursor(0, y);
  ag->display.setText(line);

  ag->display.show();
}

static void updateLedBar(void) {
  if (!hasLedBar || ag == nullptr) {
    return;
  }

  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 32; // ค่า default เป็นฟ้าสลัวเพื่อแสดงว่าไม่มีข้อมูล

  if (latestSample.valid && !isnan(latestSample.pm25)) {
    const float pm = latestSample.pm25;
    if (pm <= 12.0f) {
      r = 0;
      g = 255;
      b = 0;
    } else if (pm <= 35.4f) {
      r = 255;
      g = 170;
      b = 0;
    } else if (pm <= 55.4f) {
      r = 255;
      g = 80;
      b = 0;
    } else {
      r = 255;
      g = 0;
      b = 0;
    }
  }

  ag->ledBar.setColor(r, g, b);
  ag->ledBar.show();
}

static void printMeasurements(void) {
  if (!latestSample.valid) {
    return;
  }

  Serial.println();
  Serial.print(F("Timestamp (ms): "));
  Serial.println(latestSample.timestamp);
  Serial.print(F("PM1.0 (ug/m3): "));
  Serial.println(latestSample.pm1, 1);
  Serial.print(F("PM2.5 (ug/m3): "));
  Serial.println(latestSample.pm25, 1);
  Serial.print(F("PM10 (ug/m3): "));
  Serial.println(latestSample.pm10, 1);
  Serial.print(F("PM4.0 (ug/m3): "));
  Serial.println(latestSample.pm4, 1);
  Serial.print(F("Humidity (%RH): "));
  Serial.println(latestSample.humidity, 1);
  Serial.print(F("Temperature (degC): "));
  Serial.println(latestSample.temperature, 1);
  Serial.print(F("TVOC Index: "));
  Serial.println(latestSample.vocIndex, 1);
  Serial.print(F("NOx Index: "));
  Serial.println(latestSample.noxIndex, 1);
  Serial.print(F("CO2 (ppm): "));
  Serial.println(latestSample.co2);
}

void setup() {
  delay(2000);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("Starting SEN66 monitor example"));
  Serial.println(F("Configuring I2C bus"));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  initAirGradient();
  initSen66();

  readSchedule.update();
  logSchedule.update();

  sensorStartTime = millis();
}

void loop() {
  readSchedule.run();
  logSchedule.run();

  if (millis() - sensorStartTime > TIME_TO_CALIBRATE_CO2_MS) {
    calibrateCO2Sensor();
    sensorStartTime = millis();
  }
  printf("Calibration CO2 in %lu seconds\r\n",
         (TIME_TO_CALIBRATE_CO2_MS - (millis() - sensorStartTime)) / 1000);
  delay(1000);
}
