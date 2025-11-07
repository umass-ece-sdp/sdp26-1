// Define includes
#include <cstring>
#include <cstdint>
#include <cerrno>
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

// Define WIFI constants
// TODO: Edit WiFi constants to match base station
#define WIFI_SSID "jetson_nano_wifi"
#define WIFI_PASS "team1-falcon"
#define HOST_IP "192.168.10.1"
#define PORT 5000
#define WIFI_CONNECTED_BIT BIT0
#define WAIT_TIME 1 // in seconds

// Define functions
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_init_sta(const char *TAG);
void tcp_client_task(void *pvParameters);