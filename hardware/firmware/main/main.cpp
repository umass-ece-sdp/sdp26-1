#include "wifi_client.h"
#include "glove.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

static const char* TAG = "Main";

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi client
    wifi_client_init();

    // Initialize Glove
    glove_init();

    // Main loop
    while (1) {
        // Main processing here
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay
    }
}