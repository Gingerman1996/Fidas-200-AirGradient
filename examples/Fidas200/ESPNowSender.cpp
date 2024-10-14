#include "ESPNowSender.h"

// Constructor
ESPNowSender::ESPNowSender(const uint8_t broadcastAddress[6]) {
    memcpy(_broadcastAddress, broadcastAddress, 6);
}

// Initialize ESP-NOW
void ESPNowSender::begin() {
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send callback function
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(_peerInfo.peer_addr, _broadcastAddress, 6);
    _peerInfo.channel = 0;
    _peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&_peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

// Send data
void ESPNowSender::sendData(struct_message myData) {
    esp_err_t result = esp_now_send(_broadcastAddress, (uint8_t*)&myData, sizeof(myData));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }
}

// Callback when data is sent
void ESPNowSender::OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
