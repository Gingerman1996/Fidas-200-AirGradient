#ifndef _AG_SHARP_DISPLAY_H_
#define _AG_SHARP_DISPLAY_H_

#include "AgConfigure.h"
#include "AgValue.h"
#include "AirGradient.h"
#include "Main/PrintLog.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <Arduino.h>

/**
 * @brief Sharp Memory Display class similar to OledDisplay
 * Uses Adafruit_SharpMem library with 400x240 resolution
 */
class SharpDisplay : public PrintLog {
private:
  Configuration &config;
  AirGradient *ag;
  bool isBegin = false;
  Adafruit_SharpMem *sharpDisplay = nullptr;
  Measurements &value;
  bool isDisplayOff = false;
  
  // Sharp Memory Display pins (adjust these based on your hardware)
  static const uint8_t SHARP_SCK = 14;   // Clock pin
  static const uint8_t SHARP_MOSI = 13;  // Data pin
  static const uint8_t SHARP_SS = 15;    // Chip select pin
  
  // Display dimensions
  static const int SHARP_WIDTH = 400;
  static const int SHARP_HEIGHT = 240;
  
  // Helper methods for Sharp display specific operations
  void showTempHumSharp(bool hasStatus);
  void setCentralTextSharp(int y, const char *text);
  void showIconSharp(int x, int y, int width, int height, const unsigned char *icon);

public:
  SharpDisplay(Configuration &config, Measurements &value, Stream &log);
  ~SharpDisplay();
  
  enum DashboardStatus {
    DashBoardStatusNone,
    DashBoardStatusWiFiIssue,
    DashBoardStatusServerIssue,
    DashBoardStatusAddToDashboard,
    DashBoardStatusDeviceId,
    DashBoardStatusOfflineMode,
  };
  
  void setAirGradient(AirGradient *ag);
  bool begin(void);
  void end(void);
  void setText(String &line1, String &line2, String &line3);
  void setText(const char *line1, const char *line2, const char *line3);
  void setText(String &line1, String &line2, String &line3, String &line4);
  void setText(const char *line1, const char *line2, const char *line3, const char *line4);
  void showDashboard(void);
  void showDashboard(DashboardStatus status);
  void setBrightness(int percent);
  
#ifdef ESP32
  void showFirmwareUpdateVersion(String version);
  void showFirmwareUpdateProgress(int percent);
  void showFirmwareUpdateSuccess(int count);
  void showFirmwareUpdateFailed(void);
  void showFirmwareUpdateSkipped(void);
  void showFirmwareUpdateUpToDate(void);
#endif
  
  void showRebooting(void);
  
  // Sharp display specific methods
  Adafruit_SharpMem* getDisplay() { return sharpDisplay; }
  int getWidth() { return SHARP_WIDTH; }
  int getHeight() { return SHARP_HEIGHT; }
};

#endif /** _AG_SHARP_DISPLAY_H_ */
