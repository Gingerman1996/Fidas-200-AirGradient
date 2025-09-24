/*
This is the combined firmware code for AirGradient ONE and AirGradient Open Air
open-source hardware Air Quality Monitor with ESP32-C3 Microcontroller.

It is an air quality monitor for PM2.5, CO2, TVOCs, NOx, Temperature and
Humidity with a small display, an RGB led bar and can send data over Wifi.

// Open source air quality monitors and kits are available:
Indoor Monitor: https://www.airgradient.com/indoor/
Outdoor Monitor: https://www.airgradient.com/outdoor/

Build Instructions: AirGradient ONE:
https://www.airgradient.com/documentation/one-v9/ Build Instructions:
AirGradient Open Air:
https://www.airgradient.com/documentation/open-air-pst-kit-1-3/

Please make sure you have esp32 board manager installed. Tested with
version 2.0.11.

Important flashing settings:
- Set board to "ESP32C3 Dev Module"
- Enable "USB CDC On Boot"
- Flash frequency "80Mhz"
- Flash mode "QIO"
- Flash size "4MB"
- Partition scheme "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
- JTAG adapter "Disabled"

Configuration parameters, e.g. Celsius / Fahrenheit or PM unit (US AQI vs ug/m3)
can be set through the AirGradient dashboard.

If you have any questions please visit our forum at
https://forum.airgradient.com/

CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License

*/

#include "AgApiClient.h"
#include "AgConfigure.h"
#include "AgSchedule.h"
#include "AgStateMachine.h"
#include "AgWiFiConnector.h"
#include "AirGradient.h"
#include "EEPROM.h"
#include "ESPmDNS.h"
#include "LocalServer.h"
#include "MqttClient.h"
#include "OpenMetrics.h"
#include "OtaHandler.h"
#include "WebServer.h"
#include <HardwareSerial.h>
#include <SensirionErrors.h>
#include <SensirionI2cSen66.h>
#include <WebServer.h>
#include <WiFi.h>
#include <math.h>

#ifndef NO_ERROR
#define NO_ERROR 0
#endif

#define LED_BAR_ANIMATION_PERIOD 100                  /** ms */
#define DISP_UPDATE_INTERVAL 2500                     /** ms */
#define SERVER_CONFIG_SYNC_INTERVAL 60000             /** ms */
#define SERVER_SYNC_INTERVAL 60000                    /** ms */
#define MQTT_SYNC_INTERVAL 60000                      /** ms */
#define SENSOR_CO2_CALIB_COUNTDOWN_MAX 5              /** sec */
#define SENSOR_TVOC_UPDATE_INTERVAL 1000              /** ms */
#define SENSOR_CO2_UPDATE_INTERVAL 4000               /** ms */
#define SENSOR_PM_UPDATE_INTERVAL 2000                /** ms */
#define SENSOR_TEMP_HUM_UPDATE_INTERVAL 2000          /** ms */
#define DISPLAY_DELAY_SHOW_CONTENT_MS 2000            /** ms */
#define FIRMWARE_CHECK_FOR_UPDATE_MS (60 * 60 * 1000) /** ms */
#define SEN66_READ_INTERVAL_MS 1000                   /** ms */
#define SEN66_LOG_INTERVAL_MS 2000                    /** ms */
#define SEN66_DATA_READY_TIMEOUT_MS 500               /** ms */
#define SEN66_DATA_READY_POLL_MS 50                   /** ms */
#define SEN66_CALIB_WARMUP_MS (3UL * 60UL * 1000UL)   /** ms */
#define SEN66_FRC_MAX_RETRIES 3
#define SEN66_FRC_RETRY_DELAY_MS 200 /** ms */
#define SEN66_INVALID_CO2_CORRECTION 0xFFFF
static const int16_t SEN66_ERR_I2C_ADDRESS_NACK =
    static_cast<int16_t>(static_cast<uint16_t>(HighLevelError::WriteError) |
                         static_cast<uint16_t>(LowLevelError::I2cAddressNack));

/** I2C define */
#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 6
#define OLED_I2C_ADDR 0x3C

static MqttClient mqttClient(Serial);
static TaskHandle_t mqttTask = NULL;
static Configuration configuration(Serial);
static AgApiClient apiClient(Serial, configuration);
static Measurements measurements;
static AirGradient *ag;
static OledDisplay oledDisplay(configuration, measurements, Serial);
static StateMachine stateMachine(oledDisplay, Serial, measurements,
                                 configuration);
static WifiConnector wifiConnector(oledDisplay, Serial, stateMachine,
                                   configuration);
static OpenMetrics openMetrics(measurements, configuration, wifiConnector,
                               apiClient);
static OtaHandler otaHandler;
static LocalServer localServer(Serial, openMetrics, measurements, configuration,
                               wifiConnector);

static uint32_t factoryBtnPressTime = 0;
static int getCO2FailCount = 0;
static AgFirmwareMode fwMode = FW_MODE_I_9PSL;

static bool ledBarButtonTest = false;
static String fwNewVersion;
static SensirionI2cSen66 sen66;

struct Sen66Sample {
  float pm1 = NAN;
  float pm25 = NAN;
  float pm4 = NAN;
  float pm10 = NAN;
  float numberPm0p5 = NAN;
  float numberPm1 = NAN;
  float numberPm2p5 = NAN;
  float numberPm4 = NAN;
  float numberPm10 = NAN;
  float numberSum = NAN;
  float humidity = NAN;
  float temperature = NAN;
  float vocIndex = NAN;
  float noxIndex = NAN;
  uint16_t co2 = 0;
  uint32_t timestamp = 0;
  bool valid = false;
};

static Sen66Sample sen66Sample;
static bool sen66Ready = false;
static bool sen66PollingSuspended = false;
static uint32_t lastSen66Log = 0;
static uint32_t lastSen66Read = 0;
static uint32_t sen66MeasurementStart = 0;
static char sen66ErrorMessage[64];

static void boardInit(void);
static void failedHandler(String msg);
static void configurationUpdateSchedule(void);
static void updateDisplayAndLedBar(void);
static void sendDataToServer(void);
static void sen66Update(void);
static bool ensureSen66Sample(bool forceRead);
static bool sen66Init(void);
static void logSen66Sample(void);
static void mdnsInit(void);
static void createMqttTask(void);
static void initMqtt(void);
static void factoryConfigReset(void);
static void wdgFeedUpdate(void);
static void ledBarEnabledUpdate(void);
static void firmwareCheckForUpdate(void);
static void otaHandlerCallback(OtaState state, String mesasge);
static void displayExecuteOta(OtaState state, String msg, int processing);

AgSchedule dispLedSchedule(DISP_UPDATE_INTERVAL, updateDisplayAndLedBar);
AgSchedule configSchedule(SERVER_CONFIG_SYNC_INTERVAL,
                          configurationUpdateSchedule);
AgSchedule agApiPostSchedule(SERVER_SYNC_INTERVAL, sendDataToServer);
AgSchedule sen66Schedule(SEN66_READ_INTERVAL_MS, sen66Update);
AgSchedule watchdogFeedSchedule(60000, wdgFeedUpdate);
AgSchedule checkForUpdateSchedule(FIRMWARE_CHECK_FOR_UPDATE_MS,
                                  firmwareCheckForUpdate);

void setup() {
  /** Serial for print debug message */
  Serial.begin(115200);
  delay(100); /** For bester show log */

  /** Print device ID into log */
  Serial.println("Serial nr: " + ag->deviceId());

  /** Initialize local configure */
  configuration.begin();

  configuration.hasSensorPMS1 = true;
  configuration.hasSensorPMS2 = false;
  configuration.hasSensorSGP = true;
  configuration.hasSensorSHT = true;
  configuration.hasSensorS8 = true;

  /** Init I2C */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(1000);

  /** Detect board type: ONE_INDOOR has OLED display, Scan the I2C address to
   * identify board type */
  Wire.beginTransmission(OLED_I2C_ADDR);
  if (Wire.endTransmission() == 0x00) {
    ag = new AirGradient(BoardType::ONE_INDOOR);
  } else {
    ag = new AirGradient(BoardType::OPEN_AIR_OUTDOOR);
  }
  Serial.println("Detected " + ag->getBoardName());

  configuration.setAirGradient(ag);
  oledDisplay.setAirGradient(ag);
  stateMachine.setAirGradient(ag);
  wifiConnector.setAirGradient(ag);
  apiClient.setAirGradient(ag);
  openMetrics.setAirGradient(ag);
  localServer.setAirGraident(ag);

  /** Example set custom API root URL */
  // apiClient.setApiRoot("https://example.custom.api");

  /** Init sensor */
  boardInit();
  sen66Schedule.update();

  /** Connecting wifi */
  bool connectToWifi = false;
  if (ag->isOne()) {
    /** Show message confirm offline mode, should me perform if LED bar button
     * test pressed */
    if (ledBarButtonTest == false) {
      oledDisplay.setText(
          "Press now for",
          configuration.isOfflineMode() ? "online mode" : "offline mode", "");
      uint32_t startTime = millis();
      while (true) {
        if (ag->button.getState() == ag->button.BUTTON_PRESSED) {
          configuration.setOfflineMode(!configuration.isOfflineMode());

          oledDisplay.setText(
              "Offline Mode",
              configuration.isOfflineMode() ? " = True" : "  = False", "");
          delay(1000);
          break;
        }
        uint32_t periodMs = (uint32_t)(millis() - startTime);
        if (periodMs >= 3000) {
          break;
        }
      }
      connectToWifi = !configuration.isOfflineMode();
    } else {
      configuration.setOfflineModeWithoutSave(true);
    }
  } else {
    connectToWifi = true;
  }

  if (connectToWifi) {
    apiClient.begin();

    if (wifiConnector.connect()) {
      if (wifiConnector.isConnected()) {
        mdnsInit();
        localServer.begin();
        initMqtt();
        sendDataToAg();

#ifdef ESP8266
        // ota not supported
#else
        firmwareCheckForUpdate();
        checkForUpdateSchedule.update();
#endif

        apiClient.fetchServerConfiguration();
        configSchedule.update();
        if (apiClient.isFetchConfigureFailed()) {
          if (ag->isOne()) {
            if (apiClient.isNotAvailableOnDashboard()) {
              stateMachine.displaySetAddToDashBoard();
              stateMachine.displayHandle(
                  AgStateMachineWiFiOkServerOkSensorConfigFailed);
            } else {
              stateMachine.displayClearAddToDashBoard();
            }
          }
          stateMachine.handleLeds(
              AgStateMachineWiFiOkServerOkSensorConfigFailed);
          delay(DISPLAY_DELAY_SHOW_CONTENT_MS);
        } else {
          ledBarEnabledUpdate();
        }
      } else {
        if (wifiConnector.isConfigurePorttalTimeout()) {
          oledDisplay.showRebooting();
          delay(2500);
          oledDisplay.setText("", "", "");
          ESP.restart();
        }
      }
    }
  }
  /** Set offline mode without saving, cause wifi is not configured */
  if (wifiConnector.hasConfigurated() == false) {
    Serial.println("Set offline mode cause wifi is not configurated");
    configuration.setOfflineModeWithoutSave(true);
  }

  /** Show display Warning up */
  if (ag->isOne()) {
    oledDisplay.setText("Warming Up", "Serial Number:", ag->deviceId().c_str());
    delay(DISPLAY_DELAY_SHOW_CONTENT_MS);

    Serial.println("Display brightness: " +
                   String(configuration.getDisplayBrightness()));
    oledDisplay.setBrightness(configuration.getDisplayBrightness());
  }

  // Update display and led bar after finishing setup to show dashboard
  updateDisplayAndLedBar();
}

void loop() {
  /** Handle schedule */
  dispLedSchedule.run();
  configSchedule.run();
  agApiPostSchedule.run();
  sen66Schedule.run();

  watchdogFeedSchedule.run();

  /** Check for handle WiFi reconnect */
  wifiConnector.handle();

  /** factory reset handle */
  factoryConfigReset();

  /** check that local configura changed then do some action */
  configUpdateHandle();

  /** Firmware check for update handle */
  checkForUpdateSchedule.run();
}

static void mdnsInit(void) {
  if (!MDNS.begin(localServer.getHostname().c_str())) {
    Serial.println("Init mDNS failed");
    return;
  }

  MDNS.addService("_airgradient", "_tcp", 80);
  MDNS.addServiceTxt("_airgradient", "_tcp", "model",
                     AgFirmwareModeName(fwMode));
  MDNS.addServiceTxt("_airgradient", "_tcp", "serialno", ag->deviceId());
  MDNS.addServiceTxt("_airgradient", "_tcp", "fw_ver", ag->getVersion());
  MDNS.addServiceTxt("_airgradient", "_tcp", "vendor", "AirGradient");
}

static void createMqttTask(void) {
  if (mqttTask) {
    vTaskDelete(mqttTask);
    mqttTask = NULL;
    Serial.println("Delete old MQTT task");
  }

  Serial.println("Create new MQTT task");
  xTaskCreate(
      [](void *param) {
        for (;;) {
          delay(MQTT_SYNC_INTERVAL);

          /** Send data */
          if (mqttClient.isConnected()) {
            String payload = measurements.toString(
                true, fwMode, wifiConnector.RSSI(), ag, &configuration);
            String topic = "airgradient/readings/" + ag->deviceId();

            if (mqttClient.publish(topic.c_str(), payload.c_str(),
                                   payload.length())) {
              Serial.println("MQTT sync success");
            } else {
              Serial.println("MQTT sync failure");
            }
          }
        }
      },
      "mqtt-task", 1024 * 4, NULL, 6, &mqttTask);

  if (mqttTask == NULL) {
    Serial.println("Creat mqttTask failed");
  }
}

static void initMqtt(void) {
  String mqttUri = configuration.getMqttBrokerUri();
  if (mqttUri.isEmpty()) {
    Serial.println(
        "MQTT is not configured, skipping initialization of MQTT client");
    return;
  }

  if (mqttClient.begin(mqttUri)) {
    Serial.println("Successfully connected to MQTT broker");
    createMqttTask();
  } else {
    Serial.println("Connection to MQTT broker failed");
  }
}

static void factoryConfigReset(void) {
  if (ag->button.getState() == ag->button.BUTTON_PRESSED) {
    if (factoryBtnPressTime == 0) {
      factoryBtnPressTime = millis();
    } else {
      uint32_t ms = (uint32_t)(millis() - factoryBtnPressTime);
      if (ms >= 2000) {
        // Show display message: For factory keep for x seconds
        if (ag->isOne()) {
          oledDisplay.setText("Factory reset", "keep pressed", "for 8 sec");
        } else {
          Serial.println("Factory reset, keep pressed for 8 sec");
        }

        int count = 7;
        while (ag->button.getState() == ag->button.BUTTON_PRESSED) {
          delay(1000);
          if (ag->isOne()) {

            String str = "for " + String(count) + " sec";
            oledDisplay.setText("Factory reset", "keep pressed", str.c_str());
          } else {
            Serial.printf("Factory reset, keep pressed for %d sec\r\n", count);
          }
          count--;
          if (count == 0) {
            /** Stop MQTT task first */
            if (mqttTask) {
              vTaskDelete(mqttTask);
              mqttTask = NULL;
            }

            /** Reset WIFI */
            WiFi.disconnect(true, true);

            /** Reset local config */
            configuration.reset();

            if (ag->isOne()) {
              oledDisplay.setText("Factory reset", "successful", "");
            } else {
              Serial.println("Factory reset successful");
            }
            delay(3000);
            oledDisplay.setText("", "", "");
            ESP.restart();
          }
        }

        /** Show current content cause reset ignore */
        factoryBtnPressTime = 0;
        if (ag->isOne()) {
          updateDisplayAndLedBar();
        }
      }
    }
  } else {
    if (factoryBtnPressTime != 0) {
      if (ag->isOne()) {
        /** Restore last display content */
        updateDisplayAndLedBar();
      }
    }
    factoryBtnPressTime = 0;
  }
}

static void wdgFeedUpdate(void) {
  ag->watchdog.reset();
  Serial.println("External watchdog feed!");
}

static void ledBarEnabledUpdate(void) {
  if (ag->isOne()) {
    int brightness = configuration.getLedBarBrightness();
    Serial.println("LED bar brightness: " + String(brightness));
    if ((brightness == 0) || (configuration.getLedBarMode() == LedBarModeOff)) {
      ag->ledBar.setEnable(false);
    } else {
      ag->ledBar.setBrightness(brightness);
      ag->ledBar.setEnable(configuration.getLedBarMode() != LedBarModeOff);
    }
    ag->ledBar.show();
  }
}

static void firmwareCheckForUpdate(void) {
  Serial.println();
  Serial.println("firmwareCheckForUpdate:");

  if (wifiConnector.isConnected()) {
    Serial.println("firmwareCheckForUpdate: Perform");
    otaHandler.setHandlerCallback(otaHandlerCallback);
    otaHandler.updateFirmwareIfOutdated(ag->deviceId());
  } else {
    Serial.println("firmwareCheckForUpdate: Ignored");
  }
  Serial.println();
}

static void otaHandlerCallback(OtaState state, String mesasge) {
  Serial.println("OTA message: " + mesasge);
  switch (state) {
  case OtaState::OTA_STATE_BEGIN:
    displayExecuteOta(state, fwNewVersion, 0);
    break;
  case OtaState::OTA_STATE_FAIL:
    displayExecuteOta(state, "", 0);
    break;
  case OtaState::OTA_STATE_PROCESSING:
    displayExecuteOta(state, "", mesasge.toInt());
    break;
  case OtaState::OTA_STATE_SUCCESS:
    displayExecuteOta(state, "", mesasge.toInt());
    break;
  default:
    break;
  }
}

static void displayExecuteOta(OtaState state, String msg, int processing) {
  switch (state) {
  case OtaState::OTA_STATE_BEGIN: {
    if (ag->isOne()) {
      oledDisplay.showFirmwareUpdateVersion(msg);
    } else {
      Serial.println("New firmware: " + msg);
    }
    delay(2500);
    break;
  }
  case OtaState::OTA_STATE_FAIL: {
    if (ag->isOne()) {
      oledDisplay.showFirmwareUpdateFailed();
    } else {
      Serial.println("Error: Firmware update: failed");
    }

    delay(2500);
    break;
  }
  case OtaState::OTA_STATE_SKIP: {
    if (ag->isOne()) {
      oledDisplay.showFirmwareUpdateSkipped();
    } else {
      Serial.println("Firmware update: Skipped");
    }

    delay(2500);
    break;
  }
  case OtaState::OTA_STATE_UP_TO_DATE: {
    if (ag->isOne()) {
      oledDisplay.showFirmwareUpdateUpToDate();
    } else {
      Serial.println("Firmware update: up to date");
    }

    delay(2500);
    break;
  }
  case OtaState::OTA_STATE_PROCESSING: {
    if (ag->isOne()) {
      oledDisplay.showFirmwareUpdateProgress(processing);
    } else {
      Serial.println("Firmware update: " + String(processing) + String("%"));
    }

    break;
  }
  case OtaState::OTA_STATE_SUCCESS: {
    int i = 6;
    while (i != 0) {
      i = i - 1;
      Serial.println("OTA update performed, restarting ...");
      int i = 6;
      while (i != 0) {
        i = i - 1;
        if (ag->isOne()) {
          oledDisplay.showFirmwareUpdateSuccess(i);
        } else {
          Serial.println("Rebooting... " + String(i));
        }

        delay(1000);
      }
      oledDisplay.setBrightness(0);
      esp_restart();
    }
    break;
  }
  default:
    break;
  }
}

static void sendDataToAg() {
  /** Change oledDisplay and led state */
  if (ag->isOne()) {
    stateMachine.displayHandle(AgStateMachineWiFiOkServerConnecting);
  }
  stateMachine.handleLeds(AgStateMachineWiFiOkServerConnecting);

  /** Task handle led connecting animation */
  xTaskCreate(
      [](void *obj) {
        for (;;) {
          // ledSmHandler();
          stateMachine.handleLeds();
          if (stateMachine.getLedState() !=
              AgStateMachineWiFiOkServerConnecting) {
            break;
          }
          delay(LED_BAR_ANIMATION_PERIOD);
        }
        vTaskDelete(NULL);
      },
      "task_led", 2048, NULL, 5, NULL);

  delay(1500);
  if (apiClient.sendPing(wifiConnector.RSSI(), measurements.bootCount)) {
    if (ag->isOne()) {
      stateMachine.displayHandle(AgStateMachineWiFiOkServerConnected);
    }
    stateMachine.handleLeds(AgStateMachineWiFiOkServerConnected);
  } else {
    if (ag->isOne()) {
      stateMachine.displayHandle(AgStateMachineWiFiOkServerConnectFailed);
    }
    stateMachine.handleLeds(AgStateMachineWiFiOkServerConnectFailed);
  }
  delay(DISPLAY_DELAY_SHOW_CONTENT_MS);
  stateMachine.handleLeds(AgStateMachineNormal);
}

void dispSensorNotFound(String ss) {
  ss = ss + " not found";
  oledDisplay.setText("Sensor init", "Error:", ss.c_str());
  delay(2000);
}

static void oneIndoorInit(void) {
  configuration.hasSensorPMS2 = false;

  /** Display init */
  oledDisplay.begin();

  /** Show boot display */
  Serial.println("Firmware Version: " + ag->getVersion());

  oledDisplay.setText("AirGradient ONE",
                      "FW Version: ", ag->getVersion().c_str());
  delay(DISPLAY_DELAY_SHOW_CONTENT_MS);

  ag->ledBar.begin();
  ag->button.begin();
  ag->watchdog.begin();

  /** Run LED test on start up if button pressed */
  oledDisplay.setText("Press now for", "LED test", "");
  ledBarButtonTest = false;
  uint32_t stime = millis();
  while (true) {
    if (ag->button.getState() == ag->button.BUTTON_PRESSED) {
      ledBarButtonTest = true;
      stateMachine.executeLedBarPowerUpTest();
      break;
    }
    delay(1);
    uint32_t ms = (uint32_t)(millis() - stime);
    if (ms >= 3000) {
      break;
    }
  }

  /** Check for button to reset WiFi connecto to "airgraident" after test LED
   * bar */
  if (ledBarButtonTest) {
    if (ag->button.getState() == ag->button.BUTTON_PRESSED) {
      WiFi.begin("airgradient", "cleanair");
      oledDisplay.setText("Configure WiFi", "connect to", "\'airgradient\'");
      delay(2500);
      oledDisplay.setText("Rebooting...", "", "");
      delay(2500);
      oledDisplay.setText("", "", "");
      ESP.restart();
    }
  }
  ledBarEnabledUpdate();

  /** Show message init sensor */
  oledDisplay.setText("Monitor", "initializing...", "");

  if (!sen66Init()) {
    configuration.hasSensorPMS1 = false;
    configuration.hasSensorS8 = false;
    configuration.hasSensorSGP = false;
    configuration.hasSensorSHT = false;
    dispSensorNotFound("SEN66");
  }
}
static void openAirInit(void) {
  configuration.hasSensorSHT = true;
  configuration.hasSensorS8 = true;
  configuration.hasSensorPMS1 = true;
  configuration.hasSensorPMS2 = false;
  configuration.hasSensorSGP = true;

  fwMode = FW_MODE_O_1PST;
  Serial.println("Firmware Version: " + ag->getVersion());

  ag->watchdog.begin();
  ag->button.begin();
  ag->statusLed.begin();

  if (!sen66Init()) {
    configuration.hasSensorPMS1 = false;
    configuration.hasSensorS8 = false;
    configuration.hasSensorSGP = false;
    configuration.hasSensorSHT = false;
    Serial.println("SEN66 sensor not found");
    dispSensorNotFound("SEN66");
  }

  Serial.printf("Firmware Mode: %s\r\n", AgFirmwareModeName(fwMode));
}

static void boardInit(void) {
  if (ag->isOne()) {
    oneIndoorInit();
  } else {
    openAirInit();
  }

  localServer.setFwMode(fwMode);
}

static void failedHandler(String msg) {
  while (true) {
    Serial.println(msg);
    vTaskDelay(1000);
  }
}

static void configurationUpdateSchedule(void) {
  if (apiClient.fetchServerConfiguration()) {
    configUpdateHandle();
  }
}

static void configUpdateHandle() {
  if (configuration.isUpdated() == false) {
    return;
  }

  if (configuration.isCo2CalibrationRequested()) {
    if (!sen66Ready) {
      Serial.println("CO2 calibration requested but SEN66 sensor not ready");
    } else if ((millis() - sen66MeasurementStart) < SEN66_CALIB_WARMUP_MS) {
      Serial.println("SEN66 forced calibration skipped: sensor not warmed up "
                     "(>=3 min) yet");
    } else {
      uint16_t correction = 0;

      sen66PollingSuspended = true;
      sen66Sample.valid = false;

      int16_t stopErr = sen66.stopMeasurement();
      if (stopErr != NO_ERROR) {
        errorToString((uint16_t)stopErr, sen66ErrorMessage,
                      sizeof(sen66ErrorMessage));
        Serial.print("SEN66 stopMeasurement failed: ");
        Serial.println(sen66ErrorMessage);
      } else {
        delay(60000);

        Serial.println("Stopped SEN66 measurement for CO2 calibration");
        int16_t frcErr = NO_ERROR;
        bool frcSuccess = false;

        for (int attempt = 0; attempt < SEN66_FRC_MAX_RETRIES; ++attempt) {
          frcErr = sen66.performForcedCo2Recalibration(400, correction);
          if (frcErr == NO_ERROR) {
            if (correction == SEN66_INVALID_CO2_CORRECTION) {
              Serial.println(
                  "SEN66 forced calibration returned invalid correction");
            } else {
              frcSuccess = true;
              Serial.println("SEN66 forced CO2 calibration success");
              Serial.print("SEN66 correction: ");
              Serial.println(correction);
            }
            break;
          }

          if (frcErr == SEN66_ERR_I2C_ADDRESS_NACK &&
              (attempt + 1) < SEN66_FRC_MAX_RETRIES) {
            Serial.println(
                "SEN66 forced calibration got address NACK, retrying...");
            delay(SEN66_FRC_RETRY_DELAY_MS);
            continue;
          }

          errorToString((uint16_t)frcErr, sen66ErrorMessage,
                        sizeof(sen66ErrorMessage));
          Serial.print("SEN66 forced calibration failed: ");
          Serial.println(sen66ErrorMessage);
          break;
        }

        if (!frcSuccess && frcErr == NO_ERROR &&
            correction == SEN66_INVALID_CO2_CORRECTION) {
          // Invalid correction already logged.
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
          Serial.print("SEN66 startContinuousMeasurement failed: ");
          Serial.println(sen66ErrorMessage);
        } else {
          sen66Ready = true;
          sen66MeasurementStart = millis();
          Serial.println("SEN66 restarted measurement after CO2 calibration");
        }
        delay(1200);
      }

      sen66PollingSuspended = false;
    }
  }

  String mqttUri = configuration.getMqttBrokerUri();
  if (mqttClient.isCurrentUri(mqttUri) == false) {
    mqttClient.end();
    initMqtt();
  }

  if (ag->isOne()) {
    if (configuration.isLedBarBrightnessChanged()) {
      if (configuration.getLedBarBrightness() == 0) {
        ag->ledBar.setEnable(false);
      } else {
        if (configuration.getLedBarMode() != LedBarMode::LedBarModeOff) {
          ag->ledBar.setEnable(true);
        }
        ag->ledBar.setBrightness(configuration.getLedBarBrightness());
      }
      ag->ledBar.show();
    }

    if (configuration.isLedBarModeChanged()) {
      if (configuration.getLedBarBrightness() == 0) {
        ag->ledBar.setEnable(false);
      } else {
        if (configuration.getLedBarMode() == LedBarMode::LedBarModeOff) {
          ag->ledBar.setEnable(false);
        } else {
          ag->ledBar.setEnable(true);
          ag->ledBar.setBrightness(configuration.getLedBarBrightness());
        }
      }
      ag->ledBar.show();
    }

    if (configuration.isDisplayBrightnessChanged()) {
      oledDisplay.setBrightness(configuration.getDisplayBrightness());
    }

    stateMachine.executeLedBarTest();
  } else if (ag->isOpenAir()) {
    stateMachine.executeLedBarTest();
  }

  // Update display and led bar notification based on updated configuration
  updateDisplayAndLedBar();
}

static void updateDisplayAndLedBar(void) {
  if (factoryBtnPressTime != 0) {
    // Do not distrub factory reset sequence countdown
    return;
  }

  if (configuration.isOfflineMode()) {
    // Ignore network related status when in offline mode
    stateMachine.displayHandle(AgStateMachineNormal);
    stateMachine.handleLeds(AgStateMachineNormal);
    return;
  }

  AgStateMachineState state = AgStateMachineNormal;
  if (wifiConnector.isConnected() == false) {
    state = AgStateMachineWiFiLost;
  } else if (apiClient.isFetchConfigureFailed()) {
    state = AgStateMachineSensorConfigFailed;
    if (apiClient.isNotAvailableOnDashboard()) {
      stateMachine.displaySetAddToDashBoard();
    } else {
      stateMachine.displayClearAddToDashBoard();
    }
  } else if (apiClient.isPostToServerFailed() &&
             configuration.isPostDataToAirGradient()) {
    state = AgStateMachineServerLost;
  }

  stateMachine.displayHandle(state);
  stateMachine.handleLeds(state);
}

static bool sen66Init(void) {
  if (sen66Ready) {
    return true;
  }

  sen66.begin(Wire, SEN66_I2C_ADDR_6B);

  int16_t err = sen66.deviceReset();
  if (err != NO_ERROR) {
    errorToString((uint16_t)err, sen66ErrorMessage, sizeof(sen66ErrorMessage));
    Serial.print("SEN66 deviceReset failed: ");
    Serial.println(sen66ErrorMessage);
    return false;
  }

  delay(1200);

  err = sen66.startContinuousMeasurement();
  if (err != NO_ERROR) {
    errorToString((uint16_t)err, sen66ErrorMessage, sizeof(sen66ErrorMessage));
    Serial.print("SEN66 startContinuousMeasurement failed: ");
    Serial.println(sen66ErrorMessage);
    return false;
  }

  sen66Ready = true;
  sen66Sample.valid = false;
  lastSen66Read = 0;
  lastSen66Log = 0;
  sen66MeasurementStart = millis();

  Serial.println("SEN66 ready");
  return true;
}

static bool ensureSen66Sample(bool forceRead) {
  if (!sen66Ready) {
    return false;
  }

  uint32_t now = millis();
  if (!forceRead && sen66Sample.valid) {
    if ((now - lastSen66Read) < SEN66_READ_INTERVAL_MS) {
      return true;
    }
  }

  bool dataReady = false;
  uint8_t padding = 0;
  int16_t err = NO_ERROR;
  const uint32_t waitStart = millis();
  while ((millis() - waitStart) < SEN66_DATA_READY_TIMEOUT_MS) {
    err = sen66.getDataReady(padding, dataReady);
    if (err != NO_ERROR) {
      errorToString((uint16_t)err, sen66ErrorMessage,
                    sizeof(sen66ErrorMessage));
      Serial.print("SEN66 getDataReady failed: ");
      Serial.println(sen66ErrorMessage);
      sen66Sample.valid = false;
      return false;
    }

    Serial.printf("SEN66 dataReady=%d padding=%u\r\n", dataReady ? 1 : 0,
                  static_cast<unsigned>(padding));

    if (dataReady) {
      break;
    }

    delay(SEN66_DATA_READY_POLL_MS);
  }

  if (!dataReady) {
    Serial.println("SEN66 data not ready in time");
    sen66Sample.valid = false;
    return false;
  }

  now = millis();

  float massConcentrationPm1p0 = NAN;
  float massConcentrationPm2p5 = NAN;
  float massConcentrationPm4p0 = NAN;
  float massConcentrationPm10p0 = NAN;
  float numberPm0p5 = NAN;
  float numberPm1p0 = NAN;
  float numberPm2p5 = NAN;
  float numberPm4p0 = NAN;
  float numberPm10p0 = NAN;
  float relativeHumidity = NAN;
  float ambientTemperature = NAN;
  float vocIndex = NAN;
  float noxIndex = NAN;
  uint16_t co2 = 0;

  err = sen66.readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                 massConcentrationPm4p0,
                                 massConcentrationPm10p0, relativeHumidity,
                                 ambientTemperature, vocIndex, noxIndex, co2);
  if (err != NO_ERROR) {
    errorToString((uint16_t)err, sen66ErrorMessage, sizeof(sen66ErrorMessage));
    Serial.print("SEN66 readMeasuredValues failed: ");
    Serial.println(sen66ErrorMessage);
    sen66Sample.valid = false;
    return false;
  }

  err = sen66.readNumberConcentrationValues(
      numberPm0p5, numberPm1p0, numberPm2p5, numberPm4p0, numberPm10p0);
  if (err != NO_ERROR) {
    errorToString((uint16_t)err, sen66ErrorMessage, sizeof(sen66ErrorMessage));
    Serial.print("SEN66 readNumberConcentrationValues failed: ");
    Serial.println(sen66ErrorMessage);
    numberPm0p5 = NAN;
    numberPm1p0 = NAN;
    numberPm2p5 = NAN;
    numberPm4p0 = NAN;
    numberPm10p0 = NAN;
  }

  bool numberValid =
      !(isnan(numberPm0p5) || isnan(numberPm1p0) || isnan(numberPm2p5) ||
        isnan(numberPm4p0) || isnan(numberPm10p0));
  float numberSum = numberValid ? (numberPm0p5 + numberPm1p0 + numberPm2p5 +
                                   numberPm4p0 + numberPm10p0)
                                : NAN;

  sen66Sample.pm1 = massConcentrationPm1p0;
  sen66Sample.pm25 = massConcentrationPm2p5;
  sen66Sample.pm4 = massConcentrationPm4p0;
  sen66Sample.pm10 = massConcentrationPm10p0;
  sen66Sample.numberPm0p5 = numberPm0p5;
  sen66Sample.numberPm1 = numberPm1p0;
  sen66Sample.numberPm2p5 = numberPm2p5;
  sen66Sample.numberPm4 = numberPm4p0;
  sen66Sample.numberPm10 = numberPm10p0;
  sen66Sample.numberSum = numberSum;
  sen66Sample.humidity = relativeHumidity;
  sen66Sample.temperature = ambientTemperature;
  sen66Sample.vocIndex = vocIndex;
  sen66Sample.noxIndex = noxIndex;
  sen66Sample.co2 = co2;
  sen66Sample.timestamp = now;
  sen66Sample.valid = true;

  lastSen66Read = now;
  return true;
}

static void logSen66Sample(void) {
  if (!sen66Sample.valid) {
    return;
  }

  uint32_t now = millis();
  if ((now - lastSen66Log) < SEN66_LOG_INTERVAL_MS) {
    return;
  }

  lastSen66Log = now;

  Serial.println();
  Serial.printf("SEN66 Timestamp (ms): %lu\r\n",
                (unsigned long)sen66Sample.timestamp);
  Serial.printf("PM1.0 ug/m3: %.1f\r\n", sen66Sample.pm1);
  Serial.printf("PM2.5 ug/m3: %.1f\r\n", sen66Sample.pm25);
  Serial.printf("PM4.0 ug/m3: %.1f\r\n", sen66Sample.pm4);
  Serial.printf("PM10 ug/m3: %.1f\r\n", sen66Sample.pm10);
  Serial.printf("#0.5 um: %.1f/cm3\r\n", sen66Sample.numberPm0p5);
  Serial.printf("#1.0 um: %.1f/cm3\r\n", sen66Sample.numberPm1);
  Serial.printf("#2.5 um: %.1f/cm3\r\n", sen66Sample.numberPm2p5);
  Serial.printf("#4.0 um: %.1f/cm3\r\n", sen66Sample.numberPm4);
  Serial.printf("#10 um: %.1f/cm3\r\n", sen66Sample.numberPm10);
  if (!isnan(sen66Sample.numberSum)) {
    Serial.printf("#0.5-10 um sum: %.1f/cm3\r\n", sen66Sample.numberSum);
  }
  Serial.printf("Temperature C: %.2f\r\n", sen66Sample.temperature);
  Serial.printf("Humidity %%: %.1f\r\n", sen66Sample.humidity);
  Serial.printf("TVOC index: %.1f\r\n", sen66Sample.vocIndex);
  Serial.printf("NOx index: %.1f\r\n", sen66Sample.noxIndex);
  Serial.printf("CO2 ppm: %u\r\n", sen66Sample.co2);
}

static void sen66Update(void) {
  if (sen66PollingSuspended) {
    return;
  }

  if (!ensureSen66Sample(false)) {
    measurements.pm01_1 = utils::getInvalidPmValue();
    measurements.pm25_1 = utils::getInvalidPmValue();
    measurements.pm10_1 = utils::getInvalidPmValue();
    measurements.pm03PCount_1 = utils::getInvalidPmValue();

    measurements.pm01_2 = utils::getInvalidPmValue();
    measurements.pm25_2 = utils::getInvalidPmValue();
    measurements.pm10_2 = utils::getInvalidPmValue();
    measurements.pm03PCount_2 = utils::getInvalidPmValue();

    measurements.Temperature = utils::getInvalidTemperature();
    measurements.temp_1 = utils::getInvalidTemperature();
    measurements.temp_2 = utils::getInvalidTemperature();

    measurements.Humidity = utils::getInvalidHumidity();
    measurements.hum_1 = utils::getInvalidHumidity();
    measurements.hum_2 = utils::getInvalidHumidity();

    measurements.TVOC = utils::getInvalidVOC();
    measurements.TVOCRaw = -1;
    measurements.NOx = utils::getInvalidNOx();
    measurements.NOxRaw = -1;
    measurements.CO2 = utils::getInvalidCO2();
    return;
  }

  auto pmOrInvalid = [](float value) -> int {
    if (isnan(value)) {
      return utils::getInvalidPmValue();
    }
    return static_cast<int>(lroundf(value));
  };

  measurements.pm01_1 = pmOrInvalid(sen66Sample.pm1);
  measurements.pm25_1 = pmOrInvalid(sen66Sample.pm25);
  measurements.pm10_1 = pmOrInvalid(sen66Sample.pm10);
  if (!isnan(sen66Sample.numberSum)) {
    measurements.pm03PCount_1 =
        static_cast<int>(lroundf(sen66Sample.numberSum));
  } else {
    measurements.pm03PCount_1 = utils::getInvalidPmValue();
  }

  measurements.pm01_2 = utils::getInvalidPmValue();
  measurements.pm25_2 = utils::getInvalidPmValue();
  measurements.pm10_2 = utils::getInvalidPmValue();
  measurements.pm03PCount_2 = utils::getInvalidPmValue();

  if (!isnan(sen66Sample.temperature)) {
    measurements.Temperature = sen66Sample.temperature;
    measurements.temp_1 = sen66Sample.temperature;
  } else {
    measurements.Temperature = utils::getInvalidTemperature();
    measurements.temp_1 = utils::getInvalidTemperature();
  }
  measurements.temp_2 = utils::getInvalidTemperature();

  if (!isnan(sen66Sample.humidity)) {
    measurements.Humidity = static_cast<int>(lroundf(sen66Sample.humidity));
    measurements.hum_1 = measurements.Humidity;
  } else {
    measurements.Humidity = utils::getInvalidHumidity();
    measurements.hum_1 = utils::getInvalidHumidity();
  }
  measurements.hum_2 = utils::getInvalidHumidity();

  measurements.TVOC = isnan(sen66Sample.vocIndex)
                          ? utils::getInvalidVOC()
                          : static_cast<int>(lroundf(sen66Sample.vocIndex));
  measurements.TVOCRaw = -1;
  measurements.NOx = isnan(sen66Sample.noxIndex)
                         ? utils::getInvalidNOx()
                         : static_cast<int>(lroundf(sen66Sample.noxIndex));
  measurements.NOxRaw = -1;

  if (utils::isValidCO2(static_cast<int>(sen66Sample.co2))) {
    measurements.CO2 = sen66Sample.co2;
    getCO2FailCount = 0;
  } else {
    getCO2FailCount++;
    Serial.printf("SEN66 CO2 read invalid %d\r\n", getCO2FailCount);
    if (getCO2FailCount >= 3) {
      measurements.CO2 = utils::getInvalidCO2();
    }
  }

  logSen66Sample();
}

static void sendDataToServer(void) {
  /** Ignore send data to server if postToAirGradient disabled */
  if (configuration.isPostDataToAirGradient() == false ||
      configuration.isOfflineMode()) {
    return;
  }

  String syncData = measurements.toString(false, fwMode, wifiConnector.RSSI(),
                                          ag, &configuration);
  if (apiClient.postToServer(syncData)) {
    Serial.println();
    Serial.println(
        "Online mode and isPostToAirGradient = true: watchdog reset");
    Serial.println();
  }
  measurements.bootCount++;
}
