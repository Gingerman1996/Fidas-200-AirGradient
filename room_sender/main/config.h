#pragma once

// WiFi Configuration
#define WIFI_SSID           "ag-diamond_2.4GHz"
#define WIFI_PASSWORD       "0505563014466"

// UDP Configuration
#define UDP_PORT            8888
#define BROADCAST_IP        "192.168.100.255"

// Sensor simulation settings
#define PM2_5_BASE          35.0f    // Base PM2.5 value
#define PM10_BASE           50.0f    // Base PM10 value  
#define PM_VARIATION        10.0f    // Random variation range
#define SEND_INTERVAL_MS    5000     // Send every 5 seconds
