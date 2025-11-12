#include <Arduino.h>
#include "wifi_client.h"
#include "glove.h"

void setup() {
    Serial.begin(115200);
    delay(1000);

    update_color(COLOR_BLUE);

    Serial.println("\n[MAIN] Starting ESP32 Client...");

    // Initialize glove switches
    setup_glove();
    
    // Initialize WiFi
    setup_wifi();
}

void loop() {
    // Read glove inputs
    char gloveData[16];
    read_glove_inputs(gloveData, sizeof(gloveData));
    
    // Display glove data
    Serial.print("[MAIN] Glove data: ");
    Serial.println(gloveData);
    
    // Try to connect and send data
    connect_and_send();
    
    // If connection fails or disconnects, wait before retrying
    delay(100); // Small delay for switch debouncing
}