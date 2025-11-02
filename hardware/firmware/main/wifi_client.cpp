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

// Event group handle
EventGroupHandle_t wifi_event_group;

#define WAIT_TIME 30 // in seconds

// Define functions
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_init_sta(const char *TAG);
void tcp_client_task(void *pvParameters);

// Logging tag for ESP_LOG* macros in this file
static const char *TAG = "ESP32_STA_CLIENT";

void run_client_app(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_sta(TAG);
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}

// CLIENT FUNCTIONS
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // try to connect to the configured AP
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect(); // reconnect on unexpected disconnect
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        printf("[CLIENT] Got IP address: " IPSTR "\\n", IP2STR(&event->ip_info.ip));
        printf("[CLIENT] Ready to connect to server\\n");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT); // indicate to tasks waiting on the EventGroup that we're connected
    }
}

void wifi_init_sta(const char *TAG)
{
    // Create an event group to synchronize WiFi connection state with other tasks
    wifi_event_group = xEventGroupCreate();

    // Initialize the default TCP/IP network interface for station mode
    esp_netif_create_default_wifi_sta();

    // Initialize the WiFi driver with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register a single event handler to handle both WIFI events and IP events
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    // Configure station credentials (SSID and password are defined in the project)
    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    // Set WiFi to station mode, apply configuration and start the driver
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA init finished.");
}

// TODO: Edit Client tasks to fit this projects specifications
void tcp_client_task(void *pvParameters)
{
    const char *TAG = "ESP32_STA_CLIENT";

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    while (1)
    {
        // printf("[CLIENT] Creating socket for connection attempt...\n");
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0)
        {
            printf("[CLIENT] ERROR: Failed to create socket (errno=%d)\n", errno);
            ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        // printf("[CLIENT] Socket created (fd=%d)\n", sock);

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        // printf("[CLIENT] Attempting to connect to %s:%d...\n", HOST_IP, PORT);

        int connect_result = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (connect_result == 0)
        {
            // printf("[CLIENT] Successfully connected to server\n");
            ESP_LOGI(TAG, "Connected to server");

            while (1)
            {
                // Prepare a single float: remaining_balance
                float remaining_balance = 67.67; // placeholder value; replace with real remaining balance
                printf("[CLIENT] Preparing to send remaining balance: %f\n", remaining_balance);

                // Serialize the single float into network-order 32-bit word
                // printf("[CLIENT] Serializing float to network byte order...\n");
                uint8_t out_buffer[sizeof(uint32_t)];
                uint32_t hostbits;
                memcpy(&hostbits, &remaining_balance, sizeof(uint32_t));
                uint32_t netbits = htonl(hostbits);
                memcpy(out_buffer, &netbits, sizeof(uint32_t));
                // printf("[CLIENT] Serialization complete, buffer ready\n");

                // Send all bytes (handle partial sends)
                // printf("[CLIENT] Starting to send data...\n");
                size_t to_send = sizeof(out_buffer);
                size_t sent = 0;
                while (sent < to_send)
                {
                    printf("[CLIENT] Sending %u bytes (already sent %u)\n",
                           (unsigned)(to_send - sent), (unsigned)sent);
                    int s = send(sock, out_buffer + sent, to_send - sent, 0);
                    if (s < 0)
                    {
                        printf("[CLIENT] Send failed (result=%d)\n", s);
                        ESP_LOGE(TAG, "Send failed");
                        break;
                    }
                    printf("[CLIENT] Sent %d bytes in this send() call\n", s);
                    sent += (size_t)s;
                }
                // printf("[CLIENT] Data send complete: %u bytes total\n", (unsigned)sent);
                ESP_LOGI(TAG, "Sent remaining balance: %f (%u bytes)", remaining_balance, (unsigned)to_send);

                // Wait for ACK from server
                printf("[CLIENT] Waiting for ACK from server...\n");
                char rx_buffer[64];
                int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
                if (len > 0)
                {
                    rx_buffer[len] = 0;
                    printf("[CLIENT] Received ACK: %s\n", rx_buffer);
                    ESP_LOGI(TAG, "Received: %s", rx_buffer);
                }
                else
                {
                    printf("[CLIENT] Failed to receive ACK (len=%d)\n", len);
                }

                // Wait WAIT_TIME seconds before sending the next update. Use vTaskDelay
                // so we don't block the FreeRTOS scheduler (WAIT_TIME is in seconds).
                vTaskDelay((TickType_t)(WAIT_TIME * 1000 / portTICK_PERIOD_MS));
            }
        }
        else
        {
            // printf("[CLIENT] Connection failed (result=%d, errno=%d), retrying in 5 seconds...\n", connect_result, errno);
            ESP_LOGE(TAG, "Connection failed: errno %d", errno);
        }
        close(sock);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// ESP-IDF entry point
extern "C" void app_main(void)
{
    run_client_app();
}