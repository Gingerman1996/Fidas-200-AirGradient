#ifndef _AG_SHARP_DISPLAY_H_
#define _AG_SHARP_DISPLAY_H_

#include "AgOledDisplay.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <Arduino.h>

/**
 * @brief Sharp Memory Display class that inherits from OledDisplay
 * Uses Adafruit_SharpMem library with 400x240 resolution
 * Overrides all display methods to use Sharp Memory Display instead of OLED
 */
class SharpDisplay : public OledDisplay {
private:
  Adafruit_SharpMem *sharpDisplay = nullptr;
  bool sharpIsBegin = false;
  bool sharpIsDisplayOff = false;
  
  // Own references to config, value, and ag (parent has them private)
  Configuration &sharpConfig;
  Measurements &sharpValue;
  AirGradient *sharpAg = nullptr;
  
  // Sharp Memory Display pins (adjust these based on your hardware)
  static const uint8_t SHARP_SCK = 5;   // Clock pin
  static const uint8_t SHARP_MOSI = 4;  // Data pin
  static const uint8_t SHARP_SS = 3;    // Chip select pin
  
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
  
  // Override base class methods for Sharp display
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
