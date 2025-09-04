# Room PM Sender

This is the Room ESP32 sender that broadcasts PM sensor data via UDP.

## Features
- Connects to WiFi network
- Simulates PM sensor data (PM1.0, PM2.5, PM10)
- Broadcasts data every 5 seconds via UDP
- Compatible with main PM control system receiver

## Building and Flashing

```bash
cd room_sender
idf.py build
idf.py -p /dev/cu.usbserial-XXXXXX flash monitor
```

## Configuration
- WiFi SSID: ag-diamond_2.4GHz
- WiFi Password: blackdiamond
- UDP Port: 8888
- Broadcast IP: 255.255.255.255

## Data Format
```c
typedef struct {
    float pm1_0;
    float pm2_5;
    float pm10;
    uint32_t timestamp;
    uint8_t sensor_id;
} room_pm_data_t;
```
