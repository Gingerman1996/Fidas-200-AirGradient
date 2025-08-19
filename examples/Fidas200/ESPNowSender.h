#ifndef ESP_NOW_SENDER_H
#define ESP_NOW_SENDER_H

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <time.h>
#include <sys/time.h>

// Structure to send data
typedef struct {
  float pm02;
  float pm10;
  int wifi_rssi;
  unsigned long timestamp;
} esp_now_pm_data_t;

class ESPNowSender {
public:
    ESPNowSender(const uint8_t broadcastAddress[6]);
    void begin();
    void sendData(esp_now_pm_data_t myData);
    void initNTP();
    unsigned long getNTPTimestamp();
    
private:
    uint8_t _broadcastAddress[6];
    esp_now_peer_info_t _peerInfo;
    uint8_t _currentChannel;
    bool _ntpInitialized;
    
    void checkAndSetWiFiChannel();
    void setWiFiChannelTo11();
    uint8_t getWiFiChannel();
    static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
};

#endif
