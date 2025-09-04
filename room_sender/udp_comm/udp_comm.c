#include "udp_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_netif.h"
#include <string.h>

static const char* TAG = "UDP_COMM";

// Global variables for receiver
static int receiver_socket = -1;
static bool receiver_running = false;
static udp_data_received_callback_t data_callback = NULL;
static TaskHandle_t receiver_task_handle = NULL;

// Global variables for sender
static int sender_socket = -1;
static struct sockaddr_in broadcast_addr;

// Receiver functions
bool udp_comm_init_receiver(uint16_t port, udp_data_received_callback_t callback) {
    if (receiver_running) {
        ESP_LOGW(TAG, "UDP receiver already running");
        return false;
    }

    // Create socket
    receiver_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (receiver_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return false;
    }

    // Set socket to allow broadcast
    int broadcast = 1;
    if (setsockopt(receiver_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        ESP_LOGE(TAG, "Failed to set SO_BROADCAST: errno %d", errno);
        close(receiver_socket);
        receiver_socket = -1;
        return false;
    }

    // Bind to port
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    int err = bind(receiver_socket, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(receiver_socket);
        receiver_socket = -1;
        return false;
    }

    data_callback = callback;
    receiver_running = true;

    // Create receiver task
    xTaskCreate(udp_comm_receiver_task, "udp_receiver", 4096, NULL, 5, &receiver_task_handle);
    
    ESP_LOGI(TAG, "UDP receiver initialized on port %d", port);
    return true;
}

void udp_comm_receiver_task(void *parameter) {
    char rx_buffer[128];
    char addr_str[128];
    
    ESP_LOGI(TAG, "UDP receiver task started");
    
    while (receiver_running) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        
        int len = recvfrom(receiver_socket, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                          (struct sockaddr *)&source_addr, &socklen);
        
        if (len < 0) {
            if (receiver_running) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            }
            break;
        } else {
            // Convert IP address to string
            inet_ntoa_r(source_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
            
            // Check if received data is PM data
            if (len == sizeof(room_pm_data_t)) {
                room_pm_data_t *pm_data = (room_pm_data_t *)rx_buffer;
                
                ESP_LOGI(TAG, "Received PM data from %s: PM2.5=%.1f, PM10=%.1f", 
                         addr_str, pm_data->pm2_5, pm_data->pm10);
                
                // Call callback function
                if (data_callback) {
                    data_callback(pm_data, addr_str);
                }
            } else {
                ESP_LOGW(TAG, "Received invalid data size: %d bytes from %s", len, addr_str);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent watchdog issues
    }
    
    ESP_LOGI(TAG, "UDP receiver task ended");
    vTaskDelete(NULL);
}

void udp_comm_stop_receiver(void) {
    receiver_running = false;
    
    if (receiver_socket >= 0) {
        close(receiver_socket);
        receiver_socket = -1;
    }
    
    if (receiver_task_handle) {
        vTaskDelete(receiver_task_handle);
        receiver_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "UDP receiver stopped");
}

// Sender functions
bool udp_comm_init_sender(void) {
    // Create socket
    sender_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sender_socket < 0) {
        ESP_LOGE(TAG, "Unable to create sender socket: errno %d", errno);
        return false;
    }

    // Set socket to allow broadcast
    int broadcast = 1;
    if (setsockopt(sender_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        ESP_LOGE(TAG, "Failed to set SO_BROADCAST on sender: errno %d", errno);
        close(sender_socket);
        sender_socket = -1;
        return false;
    }

    // Setup broadcast address
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(UDP_PORT);

    ESP_LOGI(TAG, "UDP sender initialized for broadcast on port %d", UDP_PORT);
    return true;
}

bool udp_comm_send_data(const room_pm_data_t* data) {
    if (sender_socket < 0) {
        ESP_LOGE(TAG, "Sender socket not initialized");
        return false;
    }

    int err = sendto(sender_socket, data, sizeof(room_pm_data_t), 0, 
                     (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
    
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return false;
    }

    ESP_LOGD(TAG, "Sent PM data: PM2.5=%.1f, PM10=%.1f", data->pm2_5, data->pm10);
    return true;
}

void udp_comm_stop_sender(void) {
    if (sender_socket >= 0) {
        close(sender_socket);
        sender_socket = -1;
    }
    
    ESP_LOGI(TAG, "UDP sender stopped");
}
