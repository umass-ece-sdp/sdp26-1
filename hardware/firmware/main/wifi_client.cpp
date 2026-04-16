#include "wifi_client.h"

void setup_wifi()
{
    Serial.println("[CLIENT] Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\n[CLIENT] WiFi connected!");
        Serial.print("[CLIENT] IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("[CLIENT] Gateway: ");
        Serial.println(WiFi.gatewayIP());
    }
    else
    {
        Serial.println("\n[CLIENT] WiFi connection failed!");
    }
}

void connect_and_send(WiFiClient &client, const Packet &packet)
{
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("[CLIENT] WiFi disconnected, reconnecting...");
        setup_wifi();
        return;
    }

    // Try to connect to server if not already connected
    if (!client.connected())
    {
        Serial.print("[CLIENT] Connecting to server ");
        Serial.print(HOST_IP);
        Serial.print(":");
        Serial.println(PORT);

        if (!client.connect(HOST_IP, PORT))
        {
            Serial.println("[CLIENT] Connection to server failed!");
            // delay(5000);
            delay(500);
            return;
        }
        client.setNoDelay(true);

        Serial.println("[CLIENT] Connected to server!");
    }

    // Send the message
    Serial.println("[CLIENT] Sending message");
    client.write((uint8_t *)&packet, sizeof(packet));

    // Wait for ACK from server
    unsigned long timeout = millis() + 5000; // 5 second timeout
    while (client.available() == 0 && millis() < timeout)
    {
        delay(10);
    }

    if (client.available())
    {
        String response = client.readStringUntil('\n');
        Serial.print("[CLIENT] Received: ");
        Serial.println(response);
    }
    else
    {
        Serial.println("[CLIENT] Server response timeout");
        // Disconnect on timeout
        client.stop();
    }
}