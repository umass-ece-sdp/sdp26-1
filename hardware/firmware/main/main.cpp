#include "wifi_client.h"
#include "glove.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

static const char* TAG = "Main";

extern "C" void app_main(void)
{
    run_client_app();
}