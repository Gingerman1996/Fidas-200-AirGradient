#include "UdpSender.h"

// Constructor for specific target IP
UdpSender::UdpSender(const char* targetIP, uint16_t targetPort) {
  _targetIP = String(targetIP);
  _targetPort = targetPort;
  _isBroadcast = false;
  _ntpInitialized = false;
}

// Constructor for broadcast
UdpSender::UdpSender(uint16_t broadcastPort) {
  _targetIP = "255.255.255.255"; // Broadcast address
  _targetPort = broadcastPort;
  _isBroadcast = true;
  _ntpInitialized = false;
}

// Initialize UDP communication
void UdpSender::begin() {
  if (!initializeUdp()) {
    Serial.println("❌ Failed to initialize UDP");
    return;
  }

  Serial.printf("✅ UDP sender initialized for %s on port %d\n", 
                _targetIP.c_str(), _targetPort);

  if (_isBroadcast) {
    Serial.println("📡 Broadcasting mode enabled");
  } else {
    Serial.println("🎯 Point-to-point mode enabled");
  }

  Serial.println("🚀 === UDP Sender Ready ===");
}

// Initialize UDP
bool UdpSender::initializeUdp() {
  if (!_udp.begin(_targetPort)) {
    Serial.printf("❌ Failed to start UDP on port %d\n", _targetPort);
    return false;
  }
  
  Serial.printf("✅ UDP started on port %d\n", _targetPort);
  return true;
}

// Send data via UDP
bool UdpSender::sendData(udp_pm_data_t myData) {
  // Check WiFi connection status
  Serial.println("📶 === WiFi Connection Status ===");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("   ✅ WiFi Status: Connected\n");
    Serial.printf("   🏠 SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("   📡 BSSID: %s\n", WiFi.BSSIDstr().c_str());
    Serial.printf("   📊 RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("   🌐 IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("   📻 Channel: %d\n", WiFi.channel());
  } else {
    Serial.printf("   ❌ WiFi Status: Disconnected (%d)\n", WiFi.status());
    Serial.println("   ⚠️  Cannot send UDP data without WiFi connection");
    return false;
  }

  // Print data before sending
  Serial.println("📊 === Data to be sent via UDP ===");
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
  Serial.printf("   🎯 Target: %s:%d\n", _targetIP.c_str(), _targetPort);
  Serial.println("=====================================");

  // Send UDP packet
  _udp.beginPacket(_targetIP.c_str(), _targetPort);
  size_t bytesWritten = _udp.write((uint8_t*)&myData, sizeof(myData));
  bool success = _udp.endPacket();

  if (success && bytesWritten == sizeof(myData)) {
    if (_isBroadcast) {
      Serial.println("📡 Broadcast sent successfully");
    } else {
      Serial.println("📡 Sent successfully to target");
    }
    return true;
  } else {
    Serial.printf("❌ Error sending UDP data (wrote %d bytes, success: %s)\n", 
                  bytesWritten, success ? "true" : "false");
    return false;
  }
}

// Initialize NTP client
void UdpSender::initNTP() {
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
unsigned long UdpSender::getNTPTimestamp() {
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
