idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif nvs_flash esp_event lwip freertos log
)
