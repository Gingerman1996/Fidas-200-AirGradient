#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "config.h"

static const char* TAG = "ROOM_SENDER";

// PM data structure
typedef struct {
    float pm1_0;
    float pm2_5;
    float pm10;
    uint32_t timestamp;
    uint8_t sensor_id;
} room_pm_data_t;

// Global variables
static bool wifi_connected = false;
static int udp_socket = -1;
static struct sockaddr_in broadcast_addr;

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        wifi_connected = false;
        ESP_LOGI(TAG, "Retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
    }
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization finished.");
}

bool udp_init(void) {
    // Create socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return false;
    }

    // Set socket to allow broadcast
    int broadcast = 1;
    if (setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        ESP_LOGE(TAG, "Failed to set SO_BROADCAST: errno %d", errno);
        close(udp_socket);
        udp_socket = -1;
        return false;
    }

    // Setup broadcast address
    broadcast_addr.sin_addr.s_addr = inet_addr(BROADCAST_IP);
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(UDP_PORT);

    ESP_LOGI(TAG, "UDP initialized for broadcast on port %d", UDP_PORT);
    return true;
}

bool send_pm_data(const room_pm_data_t* data) {
    if (udp_socket < 0 || !wifi_connected) {
        return false;
    }

    int err = sendto(udp_socket, data, sizeof(room_pm_data_t), 0, 
                     (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
    
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return false;
    }

    ESP_LOGI(TAG, "Sent PM data: PM2.5=%.1f, PM10=%.1f", data->pm2_5, data->pm10);
    return true;
}

// Simulate PM sensor data
void generate_test_pm_data(room_pm_data_t* data) {
    static float base_pm25 = PM2_5_BASE;
    static float base_pm10 = PM10_BASE;
    static int counter = 0;
    
    // Add random variation
    data->pm1_0 = base_pm25 * 0.7 + (rand() % (int)(PM_VARIATION * 2) - PM_VARIATION) * 0.5;
    data->pm2_5 = base_pm25 + (rand() % (int)(PM_VARIATION * 2) - PM_VARIATION) * 0.5;
    data->pm10 = base_pm10 + (rand() % (int)(PM_VARIATION * 2) - PM_VARIATION) * 0.8;
    
    // Ensure positive values
    if (data->pm1_0 < 0) data->pm1_0 = 0;
    if (data->pm2_5 < 0) data->pm2_5 = 0;
    if (data->pm10 < 0) data->pm10 = 0;
}

void sender_task(void *parameter) {
    room_pm_data_t pm_data;
    
    ESP_LOGI(TAG, "Sender task started");
    
    while (1) {
        if (wifi_connected && udp_socket >= 0) {
            // Generate test PM data
            generate_test_pm_data(&pm_data);
            
            // Send data
            if (send_pm_data(&pm_data)) {
                ESP_LOGI(TAG, "📡 Broadcast: Room PM2.5=%.1f µg/m³, PM10=%.1f µg/m³", 
                         pm_data.pm2_5, pm_data.pm10);
            } else {
                ESP_LOGW(TAG, "Failed to send PM data");
            }
        } else {
            ESP_LOGD(TAG, "Waiting for WiFi connection...");
        }
        
        // Send data every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== Room PM Sender Starting ===");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    wifi_init_sta();
    
    // Wait for WiFi connection
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Initialize UDP
    if (!udp_init()) {
        ESP_LOGE(TAG, "Failed to initialize UDP");
        return;
    }
    
    ESP_LOGI(TAG, "=== System Ready - Starting PM Data Broadcasting ===");
    
    // Create sender task
    xTaskCreate(sender_task, "sender_task", 4096, NULL, 5, NULL);
}
