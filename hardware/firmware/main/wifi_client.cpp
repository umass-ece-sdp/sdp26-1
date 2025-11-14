#include "wifi_client.h"

WiFiClient client;

void setup_wifi() {
    printf("[WIFI] Connecting to %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        printf("[WIFI] Connecting...\n");
        blink_light(COLOR_PINK, 500);
        delay(50);
    }

    Serial.println("\n[WIFI] Connected to WiFi network");
    Serial.print("[WIFI] IP Address: ");
    Serial.println(WiFi.localIP());
}

void connect_and_send() {
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[CLIENT] WiFi disconnected, reconnecting...");
        blink_light(COLOR_PINK, 500);
        setup_wifi();
        return;
    }
    
    // Try to connect to server
    Serial.print("[CLIENT] Connecting to server ");
    Serial.print(HOST_IP);
    Serial.print(":");
    Serial.println(PORT);
    
    if (!client.connect(HOST_IP, PORT)) {
        Serial.println("[CLIENT] Connection to server failed!");
        blink_light(COLOR_YELLOW, 500);
        delay(5000);
        return;
    }
    
    Serial.println("[CLIENT] Connected to server!");
    
    // Send data loop
    while (client.connected()) {
        // Prepare a 4-character string to send
        const char *message = "1001";
        
        Serial.print("[CLIENT] Sending message: ");
        Serial.println(message);
        
        // Send the message
        client.print(message);
        
        // Wait for ACK from server
        unsigned long timeout = millis() + 5000; // 5 second timeout
        while (client.available() == 0 && millis() < timeout) {
            delay(10);
        }
        
        if (client.available()) {
            String response = client.readStringUntil('\n');
            Serial.print("[CLIENT] Received: ");
            Serial.println(response);
        } else {
            Serial.println("[CLIENT] Server response timeout");
            blink_light(COLOR_YELLOW, 500);
        }
        
        // Wait before sending next message
        delay(WAIT_TIME);
    }
    
    Serial.println("[CLIENT] Disconnected from server");
    client.stop();
}