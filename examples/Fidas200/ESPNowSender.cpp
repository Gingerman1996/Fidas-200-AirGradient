#include "ESPNowSender.h"

// Constructor
ESPNowSender::ESPNowSender(const uint8_t broadcastAddress[6]) {
  memcpy(_broadcastAddress, broadcastAddress, 6);
  _currentChannel = 1; // Default channel
  _ntpInitialized = false;
}

// Initialize ESP-NOW
void ESPNowSender::begin() {
  // Check and set WiFi channel first
  checkAndSetWiFiChannel();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }

  // Register send callback function
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(_peerInfo.peer_addr, _broadcastAddress, 6);
  _peerInfo.channel = _currentChannel; // Use detected channel
  _peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&_peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  }

  Serial.printf(
      "✅ ESP-NOW peer added: %02X:%02X:%02X:%02X:%02X:%02X on channel %d\n",
      _broadcastAddress[0], _broadcastAddress[1], _broadcastAddress[2],
      _broadcastAddress[3], _broadcastAddress[4], _broadcastAddress[5],
      _currentChannel);

  // Initialize PMS sensor
  // pms.passiveMode();
  // pms.wakeUp();
  Serial.println("🌡️ PMS sensor initialized");

  Serial.println("🚀 === Room Sensor Ready ===");
}

// Send data
void ESPNowSender::sendData(esp_now_pm_data_t myData) {
  // Set WiFi channel to 11 before sending
  // setWiFiChannelTo11();
  uint8_t primary_channel;
  wifi_second_chan_t second_channel;

  esp_wifi_get_channel(&primary_channel, &second_channel);

  // Check WiFi connection status and SSID
  Serial.println("📶 === WiFi Connection Status ===");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("   ✅ WiFi Status: Connected\n");
    Serial.printf("   🏠 SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("   📡 BSSID: %s\n", WiFi.BSSIDstr().c_str());
    Serial.printf("   📊 RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("   🌐 IP Address: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("   ❌ WiFi Status: Disconnected (%d)\n", WiFi.status());
    Serial.println("   ⚠️  No WiFi connection for reference");
  }

  Serial.printf("📶 WiFi Channel Status:\n");
  Serial.printf("   📻 Primary Channel: %d\n", primary_channel);
  Serial.printf("   📡 Secondary Channel: %s\n",
              (second_channel == WIFI_SECOND_CHAN_NONE)    ? "NONE"
              : (second_channel == WIFI_SECOND_CHAN_ABOVE) ? "ABOVE"
                                                           : "BELOW");

  // Print data before sending
  Serial.println("📊 === Data to be sent via ESP-NOW ===");
  Serial.printf("   🌪️  PM2.5: %.2f µg/m³\n", myData.pm02);
  Serial.printf("   💨 PM10:  %.2f µg/m³\n", myData.pm10);
  Serial.printf("   📶 WiFi RSSI: %d dBm\n", myData.wifi_rssi);
  
  // Update timestamp with NTP time before sending
  myData.timestamp = getNTPTimestamp();
  Serial.printf("   ⏰ NTP Timestamp: %lu (Unix time)\n", myData.timestamp);
  
  // Convert timestamp to readable date/time
  time_t timestamp = myData.timestamp;
  struct tm* timeinfo = localtime(&timestamp);
  Serial.printf("   📅 Date/Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  
  Serial.printf("   📦 Data size: %d bytes\n", sizeof(myData));
  Serial.println("=====================================");

  esp_err_t result =
      esp_now_send(_broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("📡 Sent with success");
  } else {
    Serial.println("❌ Error sending the data");
  }
}

// Callback when data is sent
void ESPNowSender::OnDataSent(const uint8_t *mac_addr,
                              esp_now_send_status_t status) {
  Serial.print("\r\n📊 Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Delivery Success"
                                                : "❌ Delivery Fail");
}

// Get current WiFi channel
uint8_t ESPNowSender::getWiFiChannel() {
  if (WiFi.status() == WL_CONNECTED) {
    uint8_t channel = WiFi.channel();
    Serial.printf("📶 Connected WiFi channel: %d\n", channel);
    return channel;
  } else {
    // If not connected, scan for available networks
    Serial.println("🔍 WiFi not connected, scanning for networks...");
    int numNetworks = WiFi.scanNetworks();
    if (numNetworks > 0) {
      // Find the strongest network and use its channel
      int strongestNetworkIndex = 0;
      int32_t strongestRSSI = WiFi.RSSI(0);

      for (int i = 1; i < numNetworks; i++) {
        if (WiFi.RSSI(i) > strongestRSSI) {
          strongestRSSI = WiFi.RSSI(i);
          strongestNetworkIndex = i;
        }
      }

      uint8_t channel = WiFi.channel(strongestNetworkIndex);
      Serial.printf(
          "📡 Using channel %d from strongest network: %s (RSSI: %d)\n",
          channel, WiFi.SSID(strongestNetworkIndex).c_str(), strongestRSSI);
      return channel;
    }
  }
  Serial.println("⚠️ No WiFi networks found, using default channel 1");
  return 1; // Default channel if nothing found
}

// Set WiFi channel to 11 for ESP-NOW transmission
void ESPNowSender::setWiFiChannelTo11() {
  const uint8_t targetChannel = 11;

  if (_currentChannel != targetChannel) {
    Serial.printf(
        "📶 Setting WiFi channel from %d to %d for ESP-NOW transmission\n",
        _currentChannel, targetChannel);

    // Set WiFi mode and channel at hardware level
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(targetChannel, WIFI_SECOND_CHAN_NONE);

    _currentChannel = targetChannel;

    // Update peer info with channel 11
    if (esp_now_is_peer_exist(_broadcastAddress)) {
      esp_now_del_peer(_broadcastAddress);
    }

    _peerInfo.channel = _currentChannel;
    if (esp_now_add_peer(&_peerInfo) != ESP_OK) {
      Serial.println("❌ Failed to update peer to channel 11");
    } else {
      Serial.printf("✅ Peer updated to channel %d\n", _currentChannel);
    }
  } else {
    Serial.printf("📶 Already on channel %d, ready to send\n", _currentChannel);
  }
}

// Check and set WiFi channel for ESP-NOW
void ESPNowSender::checkAndSetWiFiChannel() {
  uint8_t newChannel = getWiFiChannel();

  if (newChannel != _currentChannel) {
    Serial.printf("🔄 WiFi channel changed from %d to %d\n", _currentChannel,
                  newChannel);
    _currentChannel = newChannel;

    // Update peer info with new channel
    if (esp_now_is_peer_exist(_broadcastAddress)) {
      esp_now_del_peer(_broadcastAddress);
    }

    _peerInfo.channel = _currentChannel;
    if (esp_now_add_peer(&_peerInfo) != ESP_OK) {
      Serial.println("❌ Failed to update peer with new channel");
    } else {
      Serial.printf("✅ Peer updated to channel %d\n", _currentChannel);
    }
  }
}

// Initialize NTP client
void ESPNowSender::initNTP() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("🕐 Initializing NTP client...");
    
    // Configure NTP server (Thailand timezone)
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov", "time.cloudflare.com");
    
    // Wait for time synchronization
    Serial.print("🕐 Waiting for NTP sync");
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;
    
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
      Serial.print(".");
      delay(1000);
      time(&now);
      localtime_r(&now, &timeinfo);
    }
    
    if (retry < retry_count) {
      _ntpInitialized = true;
      Serial.println();
      Serial.printf("✅ NTP synchronized! Current time: %04d-%02d-%02d %02d:%02d:%02d\n",
                    timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      Serial.println();
      Serial.println("❌ Failed to synchronize with NTP server");
      _ntpInitialized = false;
    }
  } else {
    Serial.println("⚠️ WiFi not connected, cannot initialize NTP");
    _ntpInitialized = false;
  }
}

// Get current timestamp from NTP
unsigned long ESPNowSender::getNTPTimestamp() {
  if (!_ntpInitialized) {
    Serial.println("⚠️ NTP not initialized, trying to initialize...");
    initNTP();
  }
  
  if (_ntpInitialized) {
    time_t now;
    time(&now);
    
    // Check if time is valid (after year 2020)
    if (now > 1577836800) { // Jan 1, 2020 00:00:00 UTC
      return (unsigned long)now;
    } else {
      Serial.println("⚠️ NTP time seems invalid, using millis() as fallback");
      return millis();
    }
  } else {
    Serial.println("⚠️ NTP unavailable, using millis() as fallback");
    return millis();
  }
}
