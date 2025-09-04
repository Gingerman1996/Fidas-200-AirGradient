#ifndef UDP_COMM_H
#define UDP_COMM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// PM data structure (same as ESP-NOW)
typedef struct {
    float pm1_0;
    float pm2_5;
    float pm10;
    uint32_t timestamp;
    uint8_t sensor_id;
} room_pm_data_t;

// UDP communication settings
#define UDP_PORT 8888
#define UDP_BROADCAST_IP "255.255.255.255"

// Callback function type for received data
typedef void (*udp_data_received_callback_t)(const room_pm_data_t* data, const char* sender_ip);

// Functions for receiver (Pipe ESP32)
bool udp_comm_init_receiver(uint16_t port, udp_data_received_callback_t callback);
void udp_comm_receiver_task(void *parameter);
void udp_comm_stop_receiver(void);

// Functions for sender (Room ESP32)
bool udp_comm_init_sender(void);
bool udp_comm_send_data(const room_pm_data_t* data);
void udp_comm_stop_sender(void);

#ifdef __cplusplus
}
#endif

#endif // UDP_COMM_H
