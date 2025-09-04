#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>
#include <sys/time.h>

// Structure to send data via UDP
typedef struct {
  float pm02;
  float pm10;
  int wifi_rssi;
  unsigned long timestamp;
} udp_pm_data_t;

class UdpSender {
public:
    UdpSender(const char* targetIP, uint16_t targetPort);
    UdpSender(uint16_t broadcastPort); // Constructor for broadcast
    void begin();
    bool sendData(udp_pm_data_t myData);
    void initNTP();
    unsigned long getNTPTimestamp();
    
private:
    WiFiUDP _udp;
    String _targetIP;
    uint16_t _targetPort;
    bool _isBroadcast;
    bool _ntpInitialized;
    
    bool initializeUdp();
};

#endif
