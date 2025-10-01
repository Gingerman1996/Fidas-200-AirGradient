#include "AgSharpDisplay.h"
#include "Main/utils.h"

// Icon bitmaps for status display
static const unsigned char WIFI_ISSUE_BITS[] = {
    0xd8, 0xc6, 0xde, 0xde, 0xc7, 0xf8, 0xd1, 0xe2, 0xdc, 0xce, 0xcc,
    0xcc, 0xc0, 0xc0, 0xd0, 0xc2, 0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0};

static const unsigned char CLOUD_ISSUE_BITS[] = {
    0x70, 0xc0, 0x88, 0xc0, 0x04, 0xc1, 0x04, 0xcf, 0x02, 0xd0, 0x01,
    0xe0, 0x01, 0xe0, 0x01, 0xe0, 0xa2, 0xd0, 0x4c, 0xce, 0xa0, 0xc0};

static unsigned char OFFLINE_BITS[] = {
    0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x30, 0x00, 0x62, 0x00,
    0xE6, 0x00, 0xFE, 0x1F, 0xFE, 0x1F, 0xE6, 0x00, 0x62, 0x00,
    0x30, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/**
 * @brief Show dashboard temperature and humidity on Sharp display
 * 
 * @param hasStatus Whether status icon is present
 */
void SharpDisplay::showTempHumSharp(bool hasStatus) {
  char buf[20];
  
  // Temperature
  float temp = sharpValue.getCorrectedTempHum(Measurements::Temperature, 1);
  if (utils::isValidTemperature(temp)) {
    float t = 0.0f;
    if (sharpConfig.isTemperatureUnitInF()) {
      t = utils::degreeC_To_F(temp);
    } else {
      t = temp;
    }
    
    if (sharpConfig.isTemperatureUnitInF()) {
      if (hasStatus) {
        snprintf(buf, sizeof(buf), "%0.1f", t);
      } else {
        snprintf(buf, sizeof(buf), "%0.1f°F", t);
      }
    } else {
      if (hasStatus) {
        snprintf(buf, sizeof(buf), "%.1f", t);
      } else {
        snprintf(buf, sizeof(buf), "%.1f°C", t);
      }
    }
  } else {
    if (sharpConfig.isTemperatureUnitInF()) {
      snprintf(buf, sizeof(buf), "-°F");
    } else {
      snprintf(buf, sizeof(buf), "-°C");
    }
  }
  
  sharpDisplay->setCursor(10, 20);
  sharpDisplay->setTextSize(2);
  sharpDisplay->print(buf);
  
  // Humidity
  int rhum = round(sharpValue.getCorrectedTempHum(Measurements::Humidity, 1));
  if (utils::isValidHumidity(rhum)) {
    snprintf(buf, sizeof(buf), "%d%%", rhum);
  } else {
    snprintf(buf, sizeof(buf), "-%%");
  }
  
  sharpDisplay->setCursor(SHARP_WIDTH - 80, 20);
  sharpDisplay->print(buf);
}

/**
 * @brief Set centered text on Sharp display
 * 
 * @param y Y coordinate
 * @param text Text to display
 */
void SharpDisplay::setCentralTextSharp(int y, const char *text) {
  sharpDisplay->setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  sharpDisplay->getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SHARP_WIDTH - w) / 2;
  sharpDisplay->setCursor(x, y);
  sharpDisplay->print(text);
}

/**
 * @brief Show icon on Sharp display
 * 
 * @param x X coordinate
 * @param y Y coordinate
 * @param width Icon width
 * @param height Icon height
 * @param icon Icon bitmap data
 */
void SharpDisplay::showIconSharp(int x, int y, int width, int height, const unsigned char *icon) {
  sharpDisplay->drawBitmap(x, y, icon, width, height, 0);
}

/**
 * @brief Constructor for Sharp Display
 * 
 * @param config Configuration object
 * @param value Measurements object
 * @param log Log stream
 */
SharpDisplay::SharpDisplay(Configuration &config, Measurements &value, Stream &log)
    : OledDisplay(config, value, log), sharpConfig(config), sharpValue(value) {
  logInfo("SharpDisplay constructor");
}

/**
 * @brief Destructor for Sharp Display
 */
SharpDisplay::~SharpDisplay() {
  if (sharpDisplay) {
    delete sharpDisplay;
    sharpDisplay = nullptr;
  }
}

/**
 * @brief Set AirGradient instance
 *
 * @param ag Point to AirGradient instance
 */
void SharpDisplay::setAirGradient(AirGradient *ag) { this->sharpAg = ag; }

/**
 * @brief Initialize Sharp Memory Display
 * 
 * @return true Success
 * @return false Failure
 */
bool SharpDisplay::begin(void) {
  if (sharpIsBegin) {
    logWarning("Sharp display already initialized");
    return true;
  }
  
  // Create Sharp Memory Display instance (400x240) with 4MHz SPI speed
  sharpDisplay = new Adafruit_SharpMem(SHARP_SCK, SHARP_MOSI, SHARP_SS, SHARP_WIDTH, SHARP_HEIGHT, 4000000);
  
  if (sharpDisplay == nullptr) {
    logError("Failed to create Sharp display instance");
    return false;
  }
  
  // Initialize the display
  if (!sharpDisplay->begin()) {
    logError("Sharp display begin() failed");
    delete sharpDisplay;
    sharpDisplay = nullptr;
    return false;
  }
  
  // Clear display
  sharpDisplay->clearDisplay();
  sharpDisplay->setRotation(0);
  sharpDisplay->setTextColor(0);
  sharpDisplay->setTextWrap(true);
  sharpDisplay->refresh();
  
  sharpIsBegin = true;
  logInfo("Sharp display initialized successfully (400x240)");
  return true;
}

/**
 * @brief De-initialize Sharp Memory Display
 */
void SharpDisplay::end(void) {
  if (!sharpIsBegin) {
    logWarning("Sharp display already ended");
    return;
  }
  
  if (sharpDisplay) {
    sharpDisplay->clearDisplay();
    sharpDisplay->refresh();
    delete sharpDisplay;
    sharpDisplay = nullptr;
  }
  
  sharpIsBegin = false;
  logInfo("Sharp display ended");
}

/**
 * @brief Display text on 3 lines (String overload)
 * 
 * @param line1 First line text
 * @param line2 Second line text
 * @param line3 Third line text
 */
void SharpDisplay::setText(String &line1, String &line2, String &line3) {
  setText(line1.c_str(), line2.c_str(), line3.c_str());
}

/**
 * @brief Display text on 3 lines
 * 
 * @param line1 First line text
 * @param line2 Second line text
 * @param line3 Third line text
 */
void SharpDisplay::setText(const char *line1, const char *line2, const char *line3) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(3);
  
  sharpDisplay->setCursor(10, 40);
  sharpDisplay->println(line1);
  
  sharpDisplay->setCursor(10, 100);
  sharpDisplay->println(line2);
  
  sharpDisplay->setCursor(10, 160);
  sharpDisplay->println(line3);
  
  sharpDisplay->refresh();
}

/**
 * @brief Display text on 4 lines (String overload)
 * 
 * @param line1 First line text
 * @param line2 Second line text
 * @param line3 Third line text
 * @param line4 Fourth line text
 */
void SharpDisplay::setText(String &line1, String &line2, String &line3, String &line4) {
  setText(line1.c_str(), line2.c_str(), line3.c_str(), line4.c_str());
}

/**
 * @brief Display text on 4 lines
 * 
 * @param line1 First line text
 * @param line2 Second line text
 * @param line3 Third line text
 * @param line4 Fourth line text
 */
void SharpDisplay::setText(const char *line1, const char *line2, const char *line3, const char *line4) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(2);
  
  sharpDisplay->setCursor(10, 30);
  sharpDisplay->println(line1);
  
  sharpDisplay->setCursor(10, 80);
  sharpDisplay->println(line2);
  
  sharpDisplay->setCursor(10, 130);
  sharpDisplay->println(line3);
  
  sharpDisplay->setCursor(10, 180);
  sharpDisplay->println(line4);
  
  sharpDisplay->refresh();
}

/**
 * @brief Show dashboard without status
 */
void SharpDisplay::showDashboard(void) {
  showDashboard(DashBoardStatusNone);
}

/**
 * @brief Show dashboard with status indicator
 * 
 * @param status Dashboard status to display
 */
void SharpDisplay::showDashboard(DashboardStatus status) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  // Use own references
  Configuration &config = sharpConfig;
  Measurements &value = sharpValue;
  AirGradient *ag = sharpAg;
  
  char strBuf[32];
  const int icon_pos_x = SHARP_WIDTH - 30;
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextColor(0);
  
  // Handle status icons
  switch (status) {
    case DashBoardStatusNone:
      showTempHumSharp(false);
      break;
    case DashBoardStatusWiFiIssue:
      showIconSharp(icon_pos_x, 5, 14, 11, WIFI_ISSUE_BITS);
      showTempHumSharp(true);
      break;
    case DashBoardStatusServerIssue:
      showIconSharp(icon_pos_x, 5, 14, 11, CLOUD_ISSUE_BITS);
      showTempHumSharp(true);
      break;
    case DashBoardStatusAddToDashboard:
      setCentralTextSharp(120, "Add To Dashboard");
      sharpDisplay->refresh();
      return;
    case DashBoardStatusDeviceId:
      if (ag) {
        setCentralTextSharp(120, ag->deviceId().c_str());
      } else {
        setCentralTextSharp(120, "No Device ID");
      }
      sharpDisplay->refresh();
      return;
    case DashBoardStatusOfflineMode:
      showIconSharp(icon_pos_x, 5, 14, 14, OFFLINE_BITS);
      showTempHumSharp(true);
      break;
    default:
      break;
  }
  
  // Draw horizontal separator line
  sharpDisplay->drawLine(0, 40, SHARP_WIDTH, 40, 0);
  
  // Layout for 400x240: Three columns for CO2, PM2.5, and VOC/NOx
  int col1_x = 10;
  int col2_x = 145;
  int col3_x = 280;
  int label_y = 60;
  int value_y = 100;
  int unit_y = 140;
  
  // Column 1: CO2
  sharpDisplay->setTextSize(2);
  sharpDisplay->setCursor(col1_x, label_y);
  sharpDisplay->print("CO2");
  
  sharpDisplay->setTextSize(4);
  int co2 = round(value.getAverage(Measurements::CO2));
  if (utils::isValidCO2(co2)) {
    snprintf(strBuf, sizeof(strBuf), "%d", co2);
  } else {
    snprintf(strBuf, sizeof(strBuf), "-");
  }
  sharpDisplay->setCursor(col1_x, value_y);
  sharpDisplay->print(strBuf);
  
  sharpDisplay->setTextSize(1);
  sharpDisplay->setCursor(col1_x, unit_y);
  sharpDisplay->print("ppm");
  
  // Draw vertical separator
  sharpDisplay->drawLine(135, 45, 135, SHARP_HEIGHT, 0);
  
  // Column 2: PM2.5
  sharpDisplay->setTextSize(2);
  sharpDisplay->setCursor(col2_x, label_y);
  sharpDisplay->print("PM2.5");
  
  int pm25 = round(value.getAverage(Measurements::PM25));
  if (utils::isValidPm(pm25)) {
    if (config.hasSensorSHT && config.isPMCorrectionEnabled()) {
      pm25 = round(value.getCorrectedPM25(true));
    }
    if (config.isPmStandardInUSAQI()) {
      if (ag) {
        snprintf(strBuf, sizeof(strBuf), "%d", ag->pms5003.convertPm25ToUsAqi(pm25));
      } else {
        snprintf(strBuf, sizeof(strBuf), "%d", pm25);
      }
    } else {
      snprintf(strBuf, sizeof(strBuf), "%d", pm25);
    }
  } else {
    snprintf(strBuf, sizeof(strBuf), "-");
  }
  
  sharpDisplay->setTextSize(4);
  sharpDisplay->setCursor(col2_x, value_y);
  sharpDisplay->print(strBuf);
  
  sharpDisplay->setTextSize(1);
  sharpDisplay->setCursor(col2_x, unit_y);
  if (config.isPmStandardInUSAQI()) {
    sharpDisplay->print("AQI");
  } else {
    sharpDisplay->print("ug/m3");
  }
  
  // Draw vertical separator
  sharpDisplay->drawLine(270, 45, 270, SHARP_HEIGHT, 0);
  
  // Column 3: VOC and NOx
  sharpDisplay->setTextSize(2);
  sharpDisplay->setCursor(col3_x, label_y);
  sharpDisplay->print("VOC:");
  
  int tvoc = round(value.getAverage(Measurements::TVOC));
  if (utils::isValidVOC(tvoc)) {
    snprintf(strBuf, sizeof(strBuf), "%d", tvoc);
  } else {
    snprintf(strBuf, sizeof(strBuf), "-");
  }
  sharpDisplay->setCursor(col3_x, value_y);
  sharpDisplay->print(strBuf);
  
  // NOx
  sharpDisplay->setCursor(col3_x, value_y + 40);
  sharpDisplay->print("NOx:");
  
  int nox = round(value.getAverage(Measurements::NOx));
  if (utils::isValidNOx(nox)) {
    snprintf(strBuf, sizeof(strBuf), "%d", nox);
  } else {
    snprintf(strBuf, sizeof(strBuf), "-");
  }
  sharpDisplay->setCursor(col3_x, value_y + 70);
  sharpDisplay->print(strBuf);
  
  sharpDisplay->refresh();
}

/**
 * @brief Set display brightness
 * 
 * @param percent Brightness percentage (0-100)
 */
void SharpDisplay::setBrightness(int percent) {
  if (!sharpDisplay) {
    return;
  }
  
  if (percent == 0) {
    sharpIsDisplayOff = true;
    sharpDisplay->clearDisplay();
    sharpDisplay->refresh();
  } else {
    sharpIsDisplayOff = false;
    // Sharp Memory Display doesn't have brightness control
    // It's either on or off. We could implement PWM backlight control
    // if your hardware has a backlight with PWM control
  }
}

#ifdef ESP32
/**
 * @brief Show firmware update version
 * 
 * @param version Version string
 */
void SharpDisplay::showFirmwareUpdateVersion(String version) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(3);
  
  setCentralTextSharp(80, "Firmware Update");
  setCentralTextSharp(120, "New version");
  setCentralTextSharp(160, version.c_str());
  
  sharpDisplay->refresh();
}

/**
 * @brief Show firmware update progress
 * 
 * @param percent Progress percentage
 */
void SharpDisplay::showFirmwareUpdateProgress(int percent) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(3);
  
  setCentralTextSharp(80, "Firmware Update");
  
  char buf[32];
  snprintf(buf, sizeof(buf), "Updating... %d%%", percent);
  setCentralTextSharp(140, buf);
  
  // Draw progress bar
  int barWidth = 300;
  int barHeight = 20;
  int barX = (SHARP_WIDTH - barWidth) / 2;
  int barY = 180;
  
  sharpDisplay->drawRect(barX, barY, barWidth, barHeight, 0);
  int fillWidth = (barWidth - 4) * percent / 100;
  sharpDisplay->fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, 0);
  
  sharpDisplay->refresh();
}

/**
 * @brief Show firmware update success
 * 
 * @param count Countdown seconds
 */
void SharpDisplay::showFirmwareUpdateSuccess(int count) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(3);
  
  setCentralTextSharp(80, "Firmware Update");
  setCentralTextSharp(120, "Success");
  
  char buf[32];
  snprintf(buf, sizeof(buf), "Rebooting... %d", count);
  setCentralTextSharp(160, buf);
  
  sharpDisplay->refresh();
}

/**
 * @brief Show firmware update failed
 */
void SharpDisplay::showFirmwareUpdateFailed(void) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(3);
  
  setCentralTextSharp(100, "Firmware Update");
  setCentralTextSharp(140, "Failed");
  setCentralTextSharp(180, "Will retry");
  
  sharpDisplay->refresh();
}

/**
 * @brief Show firmware update skipped
 */
void SharpDisplay::showFirmwareUpdateSkipped(void) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(3);
  
  setCentralTextSharp(100, "Firmware Update");
  setCentralTextSharp(140, "Skipped");
  
  sharpDisplay->refresh();
}

/**
 * @brief Show firmware update is up to date
 */
void SharpDisplay::showFirmwareUpdateUpToDate(void) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(3);
  
  setCentralTextSharp(100, "Firmware Update");
  setCentralTextSharp(140, "Up to date");
  
  sharpDisplay->refresh();
}
#endif

/**
 * @brief Show rebooting message
 */
void SharpDisplay::showRebooting(void) {
  if (sharpIsDisplayOff || !sharpDisplay) {
    return;
  }
  
  sharpDisplay->clearDisplay();
  sharpDisplay->setTextSize(4);
  
  setCentralTextSharp(120, "Rebooting...");
  
  sharpDisplay->refresh();
}
