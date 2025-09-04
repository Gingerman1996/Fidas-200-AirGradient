# UDP Communication Update for Fidas200

## Changes Made

This update changes the Fidas200 example from ESP-NOW to UDP communication to avoid WiFi channel restrictions.

### Files Modified:
- `Fidas200.ino` - Main firmware file updated to use UDP instead of ESP-NOW
- `UdpSender.h` - New UDP communication header file
- `UdpSender.cpp` - New UDP communication implementation

### Files Removed:
- `ESPNowSender.h` - Removed ESP-NOW header
- `ESPNowSender.cpp` - Removed ESP-NOW implementation

## Benefits of UDP over ESP-NOW:

1. **Channel Independence**: UDP works on any WiFi channel, no need to fix channel to 11
2. **Network Integration**: UDP packets can be routed through existing network infrastructure
3. **Easier Debugging**: Network traffic can be monitored with standard tools
4. **Flexible Targeting**: Can broadcast or send to specific IP addresses
5. **Better Range**: Uses standard WiFi infrastructure instead of direct peer-to-peer

## Configuration:

### Default Settings:
- UDP Port: 8888
- Broadcast IP: 255.255.255.255 (network broadcast)
- Send Interval: 2000ms (2 seconds)

### Customization:
To change the target IP/Port, modify these defines in `Fidas200.ino`:
```cpp
#define UDP_TARGET_IP "192.168.100.255"  // Change to your network broadcast IP
#define UDP_PORT 8888                    // Change to desired port
```

### Network Broadcast IP:
Make sure to set the correct broadcast IP for your network:
- For 192.168.1.x networks: use 192.168.1.255
- For 192.168.100.x networks: use 192.168.100.255
- For 10.0.0.x networks: use 10.0.0.255

## Data Structure:

The UDP packet contains the same data as the previous ESP-NOW implementation:
```cpp
typedef struct {
  float pm02;           // PM2.5 measurement
  float pm10;           // PM10 measurement  
  int wifi_rssi;        // WiFi signal strength
  unsigned long timestamp; // NTP synchronized time
} udp_pm_data_t;
```

## Usage:

1. Flash the updated firmware to your Fidas200 device
2. Configure your network settings (SSID/Password) as usual
3. The device will automatically start broadcasting PM data via UDP
4. Set up a UDP receiver on port 8888 to receive the data

## Compatibility:

This update is compatible with:
- Existing AirGradient dashboard (if using server sync)
- Local server functionality unchanged
- All existing sensor functions
- MQTT functionality (if enabled)

The only change is the communication method for sending PM data over the network.
