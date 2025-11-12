#include "wifi_client.h"

WiFiClient client;

void setup_wifi() {
    Serial.println("[CLIENT] Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        blink_light(COLOR_PINK, 500);
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[CLIENT] WiFi connected!");
        Serial.print("[CLIENT] IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("[CLIENT] Gateway: ");
        Serial.println(WiFi.gatewayIP());
        update_color(COLOR_PINK);
    } else {
        Serial.println("\n[CLIENT] WiFi connection failed!");
    }
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
            update_color(COLOR_YELLOW);
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