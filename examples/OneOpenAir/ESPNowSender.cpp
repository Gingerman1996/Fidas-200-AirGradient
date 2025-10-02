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

bool ESPNowSender::addBroadcastPeer(int *channel) {
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }

    // Register send callback function
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(_peerInfo.peer_addr, _broadcastAddress, 6);
    _peerInfo.channel = *channel; // Dereference the pointer to get the channel value
    _peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&_peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return false;
    }
    return true;
}

// Send data
bool ESPNowSender::sendData(ESPNow_message myData) {
  esp_err_t result =
      esp_now_send(_broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    return true;
  } else {
    return false;
  }
}

// Callback when data is sent
void ESPNowSender::OnDataSent(const uint8_t *mac_addr,
                              esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                : "Delivery Fail");
}

// Ensure WiFi is connected
bool ESPNowSender::ensureWiFiConnected(const char* ssid) {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid);
    int retry_count = 0;
    while (WiFi.status() != WL_CONNECTED && retry_count < 10) {
      delay(1000);
      Serial.println("Attempting to reconnect to WiFi...");
      retry_count++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconnected to WiFi");
      return true;
    } else {
      Serial.println("Failed to reconnect to WiFi");
      return false;
    }
  }
  return true;
}
