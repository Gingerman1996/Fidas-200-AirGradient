#ifndef ESP_NOW_SENDER_H
#define ESP_NOW_SENDER_H

#include <esp_now.h>
#include <WiFi.h>

// Structure to send data
typedef struct struct_message {
    uint64_t id;   // Serial number of ESP (MAC address)
    float pm25;    // PM2.5 value
    float humi;    // Humidity value
    float temp;    // Temperature value
} struct_message;

class ESPNowSender {
public:
    ESPNowSender(const uint8_t broadcastAddress[6]);
    void begin();
    void sendData(struct_message myData);
    
private:
    uint8_t _broadcastAddress[6];
    esp_now_peer_info_t _peerInfo;
    static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
};

#endif
