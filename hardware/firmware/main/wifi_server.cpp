#include "wifi_server.h"

void setup_ap()
{
    Serial.println("[AP] Configuring WiFi...");
    
    // Disable persistent WiFi config to ensure clean state
    WiFi.persistent(false);
    
    // Turn off WiFi initially
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    Serial.println("[AP] Setting WiFi mode to AP...");
    WiFi.mode(WIFI_AP);
    delay(100);
    
    // Set WiFi power to maximum for better range
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    
    Serial.println("[AP] Creating soft AP...");
    // Create soft AP with explicit channel (1 is standard), 0 visible, max 4 clients
    bool apStarted = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
    
    if (apStarted)
    {
        // Configure IP address (gateway at 192.168.4.1)
        IPAddress apIP(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
        IPAddress gateway(AP_IP_1, AP_IP_2, AP_IP_3, AP_IP_4);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(apIP, gateway, subnet);
        
        Serial.println("[AP] ========================================");
        Serial.println("[AP] Access Point created successfully!");
        Serial.print("[AP] SSID: ");
        Serial.println(AP_SSID);
        Serial.print("[AP] Password: ");
        Serial.println(AP_PASS);
        Serial.print("[AP] IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial.print("[AP] MAC Address: ");
        Serial.println(WiFi.softAPmacAddress());
        Serial.println("[AP] ========================================");
        Serial.print("[AP] TCP Server listening on ");
        Serial.print(AP_IP_1);
        Serial.print(".");
        Serial.print(AP_IP_2);
        Serial.print(".");
        Serial.print(AP_IP_3);
        Serial.print(".");
        Serial.print(AP_IP_4);
        Serial.print(":");
        Serial.println(PORT);
        Serial.println("[AP] Waiting for base station client to connect...");
        delay(500);
    }
    else
    {
        Serial.println("[AP] ERROR: Failed to start Access Point!");
        Serial.println("[AP] Retrying in 2 seconds...");
        delay(2000);
        // Recursive retry once
        WiFi.mode(WIFI_OFF);
        delay(100);
        setup_ap();
    }
}

void accept_and_serve(WiFiServer &server, WiFiClient &client, const Packet &packet)
{
    // Check if a new client is trying to connect
    if (server.hasClient())
    {
        // Accept the connection
        if (!client.connected())
        {
            client = server.available();
            client.setNoDelay(true);
            Serial.print("[SERVER] Base station connected from: ");
            Serial.println(client.remoteIP());
        }
    }
    
    // If client is connected, send the packet and wait for ACK
    if (client.connected())
    {
        // Send the sensor packet (24 bytes)
        size_t bytes_sent = client.write((uint8_t *)&packet, sizeof(packet));
        
        if (bytes_sent == sizeof(packet))
        {
            Serial.print("[SERVER] Sent packet (");
            Serial.print(sizeof(packet));
            Serial.println(" bytes)");
            
            // Wait for ACK from client with timeout
            unsigned long timeout = millis() + 5000;
            while (client.available() == 0 && millis() < timeout)
            {
                delay(10);
            }
            
            if (client.available())
            {
                // Read ACK
                String response = client.readStringUntil('\n');
                // Serial.print("[SERVER] Received: ");
                // Serial.println(response);
            }
            else
            {
                Serial.println("[SERVER] No ACK received, waiting for client...");
                // Don't disconnect, just wait for next packet
            }
        }
        else
        {
            Serial.println("[SERVER] Failed to send packet, client disconnected");
            client.stop();
        }
    }
}
