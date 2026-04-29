#include "wifi_server.h"

void setup_ap()
{
    Serial.println("[AP] Configuring WiFi...");

    WiFi.persistent(false);
    WiFi.mode(WIFI_OFF);
    delay(100);

    Serial.println("[AP] Setting WiFi mode to AP...");
    WiFi.mode(WIFI_AP);
    delay(100);

    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    Serial.println("[AP] Creating soft AP...");
    bool apStarted = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);

    if (apStarted)
    {
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
        WiFi.mode(WIFI_OFF);
        delay(100);
        setup_ap();
    }
}

void accept_and_serve(WiFiServer &server, WiFiClient &client, const Packet &packet)
{
    // Accept new client if one is waiting and we don't already have one
    if (server.hasClient())
    {
        if (!client.connected())
        {
            client = server.available();
            client.setNoDelay(true);
            Serial.print("[SERVER] Base station connected from: ");
            Serial.println(client.remoteIP());
        }
        else
        {
            // Reject — we already have a client connected
            WiFiClient extra = server.available();
            extra.stop();
        }
    }

    if (!client.connected())
    {
        return;
    }

    // Send the sensor packet (24 bytes)
    size_t bytes_sent = client.write((uint8_t *)&packet, sizeof(packet));

    if (bytes_sent != sizeof(packet))
    {
        Serial.println("[SERVER] Failed to send packet, client disconnected");
        client.stop();
        return;
    }

    // Best-effort ACK drain. The original code blocked for up to 5 seconds
    // waiting on client.available(), which would freeze the entire glove
    // loop if a single ACK was lost over WiFi. We instead drain whatever
    // is already buffered (non-blocking) and move on. The loop's natural
    // delay(WAIT_TIME) gives the next ACK time to arrive before the next
    // send, which is plenty for flow control without a hard handshake.
    while (client.available() > 0)
    {
        client.read();
    }
}
