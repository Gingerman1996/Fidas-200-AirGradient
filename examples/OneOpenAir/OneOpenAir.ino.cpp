# 1 "C:\\Users\\santa\\AppData\\Local\\Temp\\tmpu1c1m9gv"
#include <Arduino.h>
# 1 "C:/Users/santa/OneDrive/Dokumen/AirGradient/Git/Fidas-200-AirGradient/examples/OneOpenAir/OneOpenAir.ino"
# 39 "C:/Users/santa/OneDrive/Dokumen/AirGradient/Git/Fidas-200-AirGradient/examples/OneOpenAir/OneOpenAir.ino"
#include <HardwareSerial.h>
#include <WebServer.h>
#include <WiFi.h>

#include "AgApiClient.h"
#include "AgConfigure.h"
#include "AgSchedule.h"
#include "AgStateMachine.h"
#include "AgWiFiConnector.h"
#include "AirGradient.h"
#include "EEPROM.h"
#include "ESPNowSender.h"
#include "ESPmDNS.h"
#include "LocalServer.h"
#include "MqttClient.h"
#include "OpenMetrics.h"
#include "OtaHandler.h"
#include "WebServer.h"

#define LED_BAR_ANIMATION_PERIOD 100
#define DISP_UPDATE_INTERVAL 2500
#define SERVER_CONFIG_SYNC_INTERVAL 60000
#define SERVER_SYNC_INTERVAL 60000
#define MQTT_SYNC_INTERVAL 60000
#define SENSOR_CO2_CALIB_COUNTDOWN_MAX 5
#define SENSOR_TVOC_UPDATE_INTERVAL 1000
#define SENSOR_CO2_UPDATE_INTERVAL 4000
#define SENSOR_PM_UPDATE_INTERVAL 2000
#define SENSOR_TEMP_HUM_UPDATE_INTERVAL 2000
#define DISPLAY_DELAY_SHOW_CONTENT_MS 2000
#define FIRMWARE_CHECK_FOR_UPDATE_MS (60 * 60 * 1000)


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


uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


ESPNowSender sender(broadcastAddress);
ESPNow_message ESPNowData;

static void boardInit(void);
static void failedHandler(String msg);
static void configurationUpdateSchedule(void);
static void updateDisplayAndLedBar(void);
static void updateTvoc(void);
static void updatePm(void);
static void sendDataToServer(void);
static void tempHumUpdate(void);
static void co2Update(void);
static void mdnsInit(void);
static void createMqttTask(void);
static void initMqtt(void);
static void factoryConfigReset(void);
static void wdgFeedUpdate(void);
static void ledBarEnabledUpdate(void);
static bool sgp41Init(void);
static void firmwareCheckForUpdate(void);
static void otaHandlerCallback(OtaState state, String mesasge);
static void displayExecuteOta(OtaState state, String msg, int processing);
static void sendPmViaESPNow(void);

AgSchedule dispLedSchedule(DISP_UPDATE_INTERVAL, updateDisplayAndLedBar);
AgSchedule configSchedule(SERVER_CONFIG_SYNC_INTERVAL,
                          configurationUpdateSchedule);
AgSchedule agApiPostSchedule(SERVER_SYNC_INTERVAL, sendDataToServer);
AgSchedule co2Schedule(SENSOR_CO2_UPDATE_INTERVAL, co2Update);
AgSchedule pmsSchedule(SENSOR_PM_UPDATE_INTERVAL, updatePm);
AgSchedule tempHumSchedule(SENSOR_TEMP_HUM_UPDATE_INTERVAL, tempHumUpdate);
AgSchedule tvocSchedule(SENSOR_TVOC_UPDATE_INTERVAL, updateTvoc);
AgSchedule watchdogFeedSchedule(60000, wdgFeedUpdate);
AgSchedule checkForUpdateSchedule(FIRMWARE_CHECK_FOR_UPDATE_MS,
                                  firmwareCheckForUpdate);
AgSchedule sendESPNow(1000, sendPmViaESPNow);
void setup();
void loop();
static void sendDataToAg();
void dispSensorNotFound(String ss);
static void oneIndoorInit(void);
static void openAirInit(void);
static void configUpdateHandle();
#line 141 "C:/Users/santa/OneDrive/Dokumen/AirGradient/Git/Fidas-200-AirGradient/examples/OneOpenAir/OneOpenAir.ino"
void setup() {

  Serial.begin(115200);
  delay(100);


  Serial.println("Serial nr: " + ag->deviceId());


  configuration.begin();


  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(1000);



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





  boardInit();


  bool connectToWifi = false;
  if (ag->isOne()) {


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

  if (wifiConnector.hasConfigurated() == false) {
    Serial.println("Set offline mode cause wifi is not configurated");
    configuration.setOfflineModeWithoutSave(true);
  }


  if (ag->isOne()) {
    oledDisplay.setText("Warming Up", "Serial Number:", ag->deviceId().c_str());
    delay(DISPLAY_DELAY_SHOW_CONTENT_MS);

    Serial.println("Display brightness: " +
                   String(configuration.getDisplayBrightness()));
    oledDisplay.setBrightness(configuration.getDisplayBrightness());
  }


  updateDisplayAndLedBar();

  Serial.println("Begin ESP-NOW");

  WiFi.begin("ESP32_Master_AP");


  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Master AP...");
  }

  Serial.println("Connected to Master AP");

  int channel = WiFi.channel();
  Serial.printf("Connect to channel: %d\n", channel);


  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  ESPNowData.id = ESP.getEfuseMac();


  if (!sender.addBroadcastPeer(&channel)) {
    Serial.println("Failed to add broadcast peer");
    return;
  }
}

void loop() {

  dispLedSchedule.run();
  configSchedule.run();
  agApiPostSchedule.run();

  if (configuration.hasSensorS8) {
    co2Schedule.run();
  }
  if (configuration.hasSensorPMS1 || configuration.hasSensorPMS2) {
    pmsSchedule.run();
  }
  if (ag->isOne()) {
    if (configuration.hasSensorSHT) {
      tempHumSchedule.run();
    }
  }
  if (configuration.hasSensorSGP) {
    tvocSchedule.run();
  }
  if (ag->isOne()) {
    if (configuration.hasSensorPMS1) {
      ag->pms5003.handle();
      static bool pmsConnected = false;
      if (pmsConnected != ag->pms5003.connected()) {
        pmsConnected = ag->pms5003.connected();
        Serial.printf("PMS sensor %s ", pmsConnected ? "connected" : "removed");
      }
    }
  } else {
    if (configuration.hasSensorPMS1) {
      ag->pms5003t_1.handle();
    }
    if (configuration.hasSensorPMS2) {
      ag->pms5003t_2.handle();
    }
  }

  watchdogFeedSchedule.run();


  wifiConnector.handle();


  factoryConfigReset();


  configUpdateHandle();


  checkForUpdateSchedule.run();
  sendESPNow.run();
}

static void co2Update(void) {
  int value = ag->s8.getCo2();
  if (utils::isValidCO2(value)) {
    measurements.CO2 = value;
    getCO2FailCount = 0;
    Serial.printf("CO2 (ppm): %d\r\n", measurements.CO2);
  } else {
    getCO2FailCount++;
    Serial.printf("Get CO2 failed: %d\r\n", getCO2FailCount);
    if (getCO2FailCount >= 3) {
      measurements.CO2 = utils::getInvalidCO2();
    }
  }
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

            if (mqttTask) {
              vTaskDelete(mqttTask);
              mqttTask = NULL;
            }


            WiFi.disconnect(true, true);


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


        factoryBtnPressTime = 0;
        if (ag->isOne()) {
          updateDisplayAndLedBar();
        }
      }
    }
  } else {
    if (factoryBtnPressTime != 0) {
      if (ag->isOne()) {

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

static bool sgp41Init(void) {
  ag->sgp41.setNoxLearningOffset(configuration.getNoxLearningOffset());
  ag->sgp41.setTvocLearningOffset(configuration.getTvocLearningOffset());
  if (ag->sgp41.begin(Wire)) {
    Serial.println("Init SGP41 success");
    configuration.hasSensorSGP = true;
    return true;
  } else {
    Serial.println("Init SGP41 failuire");
    configuration.hasSensorSGP = false;
  }
  return false;
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

  if (ag->isOne()) {
    stateMachine.displayHandle(AgStateMachineWiFiOkServerConnecting);
  }
  stateMachine.handleLeds(AgStateMachineWiFiOkServerConnecting);


  xTaskCreate(
      [](void *obj) {
        for (;;) {

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


  oledDisplay.begin();


  Serial.println("Firmware Version: " + ag->getVersion());

  oledDisplay.setText("AirGradient ONE",
                      "FW Version: ", ag->getVersion().c_str());
  delay(DISPLAY_DELAY_SHOW_CONTENT_MS);

  ag->ledBar.begin();
  ag->button.begin();
  ag->watchdog.begin();


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


  oledDisplay.setText("Monitor", "initializing...", "");


  if (sgp41Init() == false) {
    dispSensorNotFound("SGP41");
  }


  if (ag->sht.begin(Wire) == false) {
    Serial.println("SHTx sensor not found");
    configuration.hasSensorSHT = false;
    dispSensorNotFound("SHT");
  }


  if (ag->s8.begin(Serial1) == false) {
    Serial.println("CO2 S8 sensor not found");
    configuration.hasSensorS8 = false;
    dispSensorNotFound("S8");
  }


  if (ag->pms5003.begin(Serial0) == false) {
    Serial.println("PMS sensor not found");
    configuration.hasSensorPMS1 = false;

    dispSensorNotFound("PMS");
  }
}
static void openAirInit(void) {
  configuration.hasSensorSHT = false;

  fwMode = FW_MODE_O_1PST;
  Serial.println("Firmware Version: " + ag->getVersion());

  ag->watchdog.begin();
  ag->button.begin();
  ag->statusLed.begin();





  bool serial1Available = true;
  bool serial0Available = true;

  if (ag->s8.begin(Serial1) == false) {
    Serial1.end();
    delay(200);
    Serial.println("Can not detect S8 on Serial1, try on Serial0");

    if (ag->s8.begin(Serial0) == false) {
      configuration.hasSensorS8 = false;

      Serial.println("CO2 S8 sensor not found");
      Serial.println("Can not detect S8 run mode 'PPT'");
      fwMode = FW_MODE_O_1PPT;

      Serial0.end();
      delay(200);
    } else {
      Serial.println("Found S8 on Serial0");
      serial0Available = false;
    }
  } else {
    Serial.println("Found S8 on Serial1");
    serial1Available = false;
  }

  if (sgp41Init() == false) {
    Serial.println("SGP sensor not found");

    if (configuration.hasSensorS8 == false) {
      Serial.println("Can not detect SGP run mode 'O-1PP'");
      fwMode = FW_MODE_O_1PP;
    } else {
      Serial.println("Can not detect SGP run mode 'O-1PS'");
      fwMode = FW_MODE_O_1PS;
    }
  }


  if (fwMode == FW_MODE_O_1PST) {
    bool pmInitSuccess = false;
    if (serial0Available) {
      if (ag->pms5003t_1.begin(Serial0) == false) {
        configuration.hasSensorPMS1 = false;
        Serial.println("No PM sensor detected on Serial0");
      } else {
        serial0Available = false;
        pmInitSuccess = true;
        Serial.println("Detected PM 1 on Serial0");
      }
    }
    if (pmInitSuccess == false) {
      if (serial1Available) {
        if (ag->pms5003t_1.begin(Serial1) == false) {
          configuration.hasSensorPMS1 = false;
          Serial.println("No PM sensor detected on Serial1");
        } else {
          serial1Available = false;
          Serial.println("Detected PM 1 on Serial1");
        }
      }
    }
    configuration.hasSensorPMS2 = false;
  } else {
    if (ag->pms5003t_1.begin(Serial0) == false) {
      configuration.hasSensorPMS1 = false;
      Serial.println("No PM sensor detected on Serial0");
    } else {
      Serial.println("Detected PM 1 on Serial0");
    }
    if (ag->pms5003t_2.begin(Serial1) == false) {
      configuration.hasSensorPMS2 = false;
      Serial.println("No PM sensor detected on Serial1");
    } else {
      Serial.println("Detected PM 2 on Serial1");
    }

    if (fwMode == FW_MODE_O_1PP) {
      int count = (configuration.hasSensorPMS1 ? 1 : 0) +
                  (configuration.hasSensorPMS2 ? 1 : 0);
      if (count == 1) {
        fwMode = FW_MODE_O_1P;
      }
    }
  }


  if (fwMode != FW_MODE_O_1PST) {
    if (configuration.hasSensorPMS1 && configuration.hasSensorPMS2) {
      pmsSchedule.setPeriod(2000);
    }
  }
  Serial.printf("Firmware Mode: %s\r\n", AgFirmwareModeName(fwMode));
}

static void boardInit(void) {
  if (ag->isOne()) {
    oneIndoorInit();
  } else {
    openAirInit();
  }


  if (configuration.hasSensorS8) {
    if (ag->s8.setAbcPeriod(configuration.getCO2CalibrationAbcDays() * 24)) {
      Serial.println("Set S8 AbcDays successful");
    } else {
      Serial.println("Set S8 AbcDays failure");
    }
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

  stateMachine.executeCo2Calibration();

  String mqttUri = configuration.getMqttBrokerUri();
  if (mqttClient.isCurrentUri(mqttUri) == false) {
    mqttClient.end();
    initMqtt();
  }

  if (configuration.hasSensorSGP) {
    if (configuration.noxLearnOffsetChanged() ||
        configuration.tvocLearnOffsetChanged()) {
      ag->sgp41.end();

      int oldTvocOffset = ag->sgp41.getTvocLearningOffset();
      int oldNoxOffset = ag->sgp41.getNoxLearningOffset();
      bool result = sgp41Init();
      const char *resultStr = "successful";
      if (!result) {
        resultStr = "failure";
      }
      if (oldTvocOffset != configuration.getTvocLearningOffset()) {
        Serial.printf("Setting tvocLearningOffset from %d to %d hours %s\r\n",
                      oldTvocOffset, configuration.getTvocLearningOffset(),
                      resultStr);
      }
      if (oldNoxOffset != configuration.getNoxLearningOffset()) {
        Serial.printf("Setting noxLearningOffset from %d to %d hours %s\r\n",
                      oldNoxOffset, configuration.getNoxLearningOffset(),
                      resultStr);
      }
    }
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


  updateDisplayAndLedBar();
}

static void updateDisplayAndLedBar(void) {
  if (factoryBtnPressTime != 0) {

    return;
  }

  if (configuration.isOfflineMode()) {

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

static void updateTvoc(void) {
  measurements.TVOC = ag->sgp41.getTvocIndex();
  measurements.TVOCRaw = ag->sgp41.getTvocRaw();
  measurements.NOx = ag->sgp41.getNoxIndex();
  measurements.NOxRaw = ag->sgp41.getNoxRaw();

  Serial.println();
  Serial.printf("TVOC index: %d\r\n", measurements.TVOC);
  Serial.printf("TVOC raw: %d\r\n", measurements.TVOCRaw);
  Serial.printf("NOx index: %d\r\n", measurements.NOx);
  Serial.printf("NOx raw: %d\r\n", measurements.NOxRaw);
}

static void updatePm(void) {
  bool restart = false;
  if (ag->isOne()) {
    if (ag->pms5003.connected()) {
      measurements.pm01_1 = ag->pms5003.getPm01Ae();
      measurements.pm25_1 = ag->pms5003.getPm25Ae();
      measurements.pm10_1 = ag->pms5003.getPm10Ae();
      measurements.pm03PCount_1 = ag->pms5003.getPm03ParticleCount();

      Serial.println();
      Serial.printf("PM1 ug/m3: %d\r\n", measurements.pm01_1);
      Serial.printf("PM2.5 ug/m3: %d\r\n", measurements.pm25_1);
      Serial.printf("PM10 ug/m3: %d\r\n", measurements.pm10_1);
      Serial.printf("PM0.3 Count: %d\r\n", measurements.pm03PCount_1);
      Serial.printf("PM firmware version: %d\r\n",
                    ag->pms5003.getFirmwareVersion());
      ag->pms5003.resetFailCount();
    } else {
      ag->pms5003.updateFailCount();
      Serial.printf("PMS read failed %d times\r\n", ag->pms5003.getFailCount());
      if (ag->pms5003.getFailCount() >= PMS_FAIL_COUNT_SET_INVALID) {
        measurements.pm01_1 = utils::getInvalidPmValue();
        measurements.pm25_1 = utils::getInvalidPmValue();
        measurements.pm10_1 = utils::getInvalidPmValue();
        measurements.pm03PCount_1 = utils::getInvalidPmValue();
      }

      if (ag->pms5003.getFailCount() >= ag->pms5003.getFailCountMax()) {
        restart = true;
      }
    }
  } else {
    bool pmsResult_1 = false;
    bool pmsResult_2 = false;
    if (configuration.hasSensorPMS1 && ag->pms5003t_1.connected()) {
      measurements.pm01_1 = ag->pms5003t_1.getPm01Ae();
      measurements.pm25_1 = ag->pms5003t_1.getPm25Ae();
      measurements.pm10_1 = ag->pms5003t_1.getPm10Ae();
      measurements.pm03PCount_1 = ag->pms5003t_1.getPm03ParticleCount();
      measurements.temp_1 = ag->pms5003t_1.getTemperature();
      measurements.hum_1 = ag->pms5003t_1.getRelativeHumidity();

      pmsResult_1 = true;

      Serial.println();
      Serial.printf("[1] PM1 ug/m3: %d\r\n", measurements.pm01_1);
      Serial.printf("[1] PM2.5 ug/m3: %d\r\n", measurements.pm25_1);
      Serial.printf("[1] PM10 ug/m3: %d\r\n", measurements.pm10_1);
      Serial.printf("[1] PM3.0 Count: %d\r\n", measurements.pm03PCount_1);
      Serial.printf("[1] Temperature in C: %0.2f\r\n", measurements.temp_1);
      Serial.printf("[1] Relative Humidity: %d\r\n", measurements.hum_1);
      Serial.printf("[1] Temperature compensated in C: %0.2f\r\n",
                    ag->pms5003t_1.compensateTemp(measurements.temp_1));
      Serial.printf("[1] Relative Humidity compensated: %0.2f\r\n",
                    ag->pms5003t_1.compensateHum(measurements.hum_1));
      Serial.printf("[1] PM firmware version: %d\r\n",
                    ag->pms5003t_1.getFirmwareVersion());

      ag->pms5003t_1.resetFailCount();
    } else {
      if (configuration.hasSensorPMS1) {
        ag->pms5003t_1.updateFailCount();
        Serial.printf("[1] PMS read failed %d times\r\n",
                      ag->pms5003t_1.getFailCount());

        if (ag->pms5003t_1.getFailCount() >= PMS_FAIL_COUNT_SET_INVALID) {
          measurements.pm01_1 = utils::getInvalidPmValue();
          measurements.pm25_1 = utils::getInvalidPmValue();
          measurements.pm10_1 = utils::getInvalidPmValue();
          measurements.pm03PCount_1 = utils::getInvalidPmValue();
          measurements.temp_1 = utils::getInvalidTemperature();
          measurements.hum_1 = utils::getInvalidHumidity();
        }

        if (ag->pms5003t_1.getFailCount() >= ag->pms5003t_1.getFailCountMax()) {
          restart = true;
        }
      }
    }

    if (configuration.hasSensorPMS2 && ag->pms5003t_2.connected()) {
      measurements.pm01_2 = ag->pms5003t_2.getPm01Ae();
      measurements.pm25_2 = ag->pms5003t_2.getPm25Ae();
      measurements.pm10_2 = ag->pms5003t_2.getPm10Ae();
      measurements.pm03PCount_2 = ag->pms5003t_2.getPm03ParticleCount();
      measurements.temp_2 = ag->pms5003t_2.getTemperature();
      measurements.hum_2 = ag->pms5003t_2.getRelativeHumidity();

      pmsResult_2 = true;

      Serial.println();
      Serial.printf("[2] PM1 ug/m3: %d\r\n", measurements.pm01_2);
      Serial.printf("[2] PM2.5 ug/m3: %d\r\n", measurements.pm25_2);
      Serial.printf("[2] PM10 ug/m3: %d\r\n", measurements.pm10_2);
      Serial.printf("[2] PM3.0 Count: %d\r\n", measurements.pm03PCount_2);
      Serial.printf("[2] Temperature in C: %0.2f\r\n", measurements.temp_2);
      Serial.printf("[2] Relative Humidity: %d\r\n", measurements.hum_2);
      Serial.printf("[2] Temperature compensated in C: %0.2f\r\n",
                    ag->pms5003t_1.compensateTemp(measurements.temp_2));
      Serial.printf("[2] Relative Humidity compensated: %0.2f\r\n",
                    ag->pms5003t_1.compensateHum(measurements.hum_2));
      Serial.printf("[2] PM firmware version: %d\r\n",
                    ag->pms5003t_2.getFirmwareVersion());

      ag->pms5003t_2.resetFailCount();
    } else {
      if (configuration.hasSensorPMS2) {
        ag->pms5003t_2.updateFailCount();
        Serial.printf("[2] PMS read failed %d times\r\n",
                      ag->pms5003t_2.getFailCount());

        if (ag->pms5003t_2.getFailCount() >= PMS_FAIL_COUNT_SET_INVALID) {
          measurements.pm01_2 = utils::getInvalidPmValue();
          measurements.pm25_2 = utils::getInvalidPmValue();
          measurements.pm10_2 = utils::getInvalidPmValue();
          measurements.pm03PCount_2 = utils::getInvalidPmValue();
          measurements.temp_2 = utils::getInvalidTemperature();
          measurements.hum_2 = utils::getInvalidHumidity();
        }

        if (ag->pms5003t_2.getFailCount() >= ag->pms5003t_2.getFailCountMax()) {
          restart = true;
        }
      }
    }

    if (configuration.hasSensorPMS1 && configuration.hasSensorPMS2 &&
        pmsResult_1 && pmsResult_2) {

      measurements.pm1Value01 = measurements.pm1Value01 + measurements.pm01_1;
      measurements.pm1Value25 = measurements.pm1Value25 + measurements.pm25_1;
      measurements.pm1Value10 = measurements.pm1Value10 + measurements.pm10_1;
      measurements.pm1PCount =
          measurements.pm1PCount + measurements.pm03PCount_1;
      measurements.pm1temp = measurements.pm1temp + measurements.temp_1;
      measurements.pm1hum = measurements.pm1hum + measurements.hum_1;


      measurements.pm2Value01 = measurements.pm2Value01 + measurements.pm01_2;
      measurements.pm2Value25 = measurements.pm2Value25 + measurements.pm25_2;
      measurements.pm2Value10 = measurements.pm2Value10 + measurements.pm10_2;
      measurements.pm2PCount =
          measurements.pm2PCount + measurements.pm03PCount_2;
      measurements.pm2temp = measurements.pm2temp + measurements.temp_2;
      measurements.pm2hum = measurements.pm2hum + measurements.hum_2;

      measurements.countPosition++;


      if (measurements.countPosition == measurements.targetCount) {
        measurements.pm01_1 =
            measurements.pm1Value01 / measurements.targetCount;
        measurements.pm25_1 =
            measurements.pm1Value25 / measurements.targetCount;
        measurements.pm10_1 =
            measurements.pm1Value10 / measurements.targetCount;
        measurements.pm03PCount_1 =
            measurements.pm1PCount / measurements.targetCount;
        measurements.temp_1 = measurements.pm1temp / measurements.targetCount;
        measurements.hum_1 = measurements.pm1hum / measurements.targetCount;

        measurements.pm01_2 =
            measurements.pm2Value01 / measurements.targetCount;
        measurements.pm25_2 =
            measurements.pm2Value25 / measurements.targetCount;
        measurements.pm10_2 =
            measurements.pm2Value10 / measurements.targetCount;
        measurements.pm03PCount_2 =
            measurements.pm2PCount / measurements.targetCount;
        measurements.temp_2 = measurements.pm2temp / measurements.targetCount;
        measurements.hum_2 = measurements.pm2hum / measurements.targetCount;

        measurements.countPosition = 0;

        measurements.pm1Value01 = 0;
        measurements.pm1Value25 = 0;
        measurements.pm1Value10 = 0;
        measurements.pm1PCount = 0;
        measurements.pm1temp = 0;
        measurements.pm1hum = 0;
        measurements.pm2Value01 = 0;
        measurements.pm2Value25 = 0;
        measurements.pm2Value10 = 0;
        measurements.pm2PCount = 0;
        measurements.pm2temp = 0;
        measurements.pm2hum = 0;
      }
    }

    if (pmsResult_1 && pmsResult_2) {
      measurements.Temperature =
          (measurements.temp_1 + measurements.temp_2) / 2;
      measurements.Humidity = (measurements.hum_1 + measurements.hum_2) / 2;
    } else {
      if (pmsResult_1) {
        measurements.Temperature = measurements.temp_1;
        measurements.Humidity = measurements.hum_1;
      }
      if (pmsResult_2) {
        measurements.Temperature = measurements.temp_2;
        measurements.Humidity = measurements.hum_2;
      }
    }

    if (configuration.hasSensorSGP) {
      float temp;
      float hum;
      if (pmsResult_1 && pmsResult_2) {
        temp = (measurements.temp_1 + measurements.temp_2) / 2.0f;
        hum = (measurements.hum_1 + measurements.hum_2) / 2.0f;
      } else {
        if (pmsResult_1) {
          temp = measurements.temp_1;
          hum = measurements.hum_1;
        }
        if (pmsResult_2) {
          temp = measurements.temp_2;
          hum = measurements.hum_2;
        }
      }
      ag->sgp41.setCompensationTemperatureHumidity(temp, hum);
    }
  }

  if (restart) {
    Serial.printf("PMS failure count reach to max set %d, restarting...",
                  ag->pms5003.getFailCountMax());
    ESP.restart();
  }
}

static void sendDataToServer(void) {

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

static void tempHumUpdate(void) {
  delay(100);
  if (ag->sht.measure()) {
    measurements.Temperature = ag->sht.getTemperature();
    measurements.Humidity = ag->sht.getRelativeHumidity();

    Serial.printf("Temperature in C: %0.2f\r\n", measurements.Temperature);
    Serial.printf("Relative Humidity: %d\r\n", measurements.Humidity);
    Serial.printf("Temperature compensated in C: %0.2f\r\n",
                  measurements.Temperature);
    Serial.printf("Relative Humidity compensated: %d\r\n",
                  measurements.Humidity);


    if (configuration.hasSensorSGP) {
      ag->sgp41.setCompensationTemperatureHumidity(measurements.Temperature,
                                                   measurements.Humidity);
    }
  } else {
    measurements.Temperature = utils::getInvalidTemperature();
    measurements.Humidity = utils::getInvalidHumidity();
    Serial.println("SHT read failed");
  }
}

static void sendPmViaESPNow(void) {

  ESPNowData.pm25 = measurements.pm25_1;
  ESPNowData.temp = measurements.Temperature;
  ESPNowData.humi = measurements.Humidity;


  if (sender.ensureWiFiConnected("ESP32_Master_AP")) {
    if (!sender.sendData(ESPNowData)) {
      Serial.println("Failed to send data");
    } else {
      Serial.println("Data sent successfully");
    }
  }
}