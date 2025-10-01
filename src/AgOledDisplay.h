#ifndef _AG_OLED_DISPLAY_H_
#define _AG_OLED_DISPLAY_H_

#include "AgConfigure.h"
#include "AgValue.h"
#include "AirGradient.h"
#include "Main/PrintLog.h"
#include <Arduino.h>

class OledDisplay : public PrintLog {
private:
  Configuration &config;
  AirGradient *ag;
  bool isBegin = false;
  void *u8g2 = NULL;
  Measurements &value;
  bool isDisplayOff = false;

  typedef struct {
    int width;
    int height;
    unsigned char *icon;
  } xbm_icon_t;

  void showTempHum(bool hasStatus);
  void setCentralText(int y, String text);
  void setCentralText(int y, const char *text);
  void showIcon(int x, int y, xbm_icon_t *icon);

public:
  OledDisplay(Configuration &config, Measurements &value, Stream &log);
  ~OledDisplay();

  enum DashboardStatus {
    DashBoardStatusNone,
    DashBoardStatusWiFiIssue,
    DashBoardStatusServerIssue,
    DashBoardStatusAddToDashboard,
    DashBoardStatusDeviceId,
    DashBoardStatusOfflineMode,
  };

  virtual void setAirGradient(AirGradient *ag);
  virtual bool begin(void);
  virtual void end(void);
  virtual void setText(String &line1, String &line2, String &line3);
  virtual void setText(const char *line1, const char *line2, const char *line3);
  virtual void setText(String &line1, String &line2, String &line3, String &line4);
  virtual void setText(const char *line1, const char *line2, const char *line3,
               const char *line4);
  virtual void showDashboard(void);
  virtual void showDashboard(DashboardStatus status);
  virtual void setBrightness(int percent);
#ifdef ESP32
  virtual void showFirmwareUpdateVersion(String version);
  virtual void showFirmwareUpdateProgress(int percent);
  virtual void showFirmwareUpdateSuccess(int count);
  virtual void showFirmwareUpdateFailed(void);
  virtual void showFirmwareUpdateSkipped(void);
  virtual void showFirmwareUpdateUpToDate(void);
#else

#endif
  virtual void showRebooting(void);
};



#endif /** _AG_OLED_DISPLAY_H_ */
