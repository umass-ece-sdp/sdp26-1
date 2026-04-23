#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "glove.h"

// WiFi AP configuration (ESP32 will create this AP for the base station to connect to)
#define AP_SSID "FALCON-Glove"
#define AP_PASS "team1-falcon"
// The base station connects to this AP and should be configured with a static IP at 192.168.4.100
#define HOST_IP "192.168.4.100"  // Fixed base station IP on ESP32 AP network
#define PORT 5000
#define WAIT_TIME 90 // in milliseconds

// Function prototypes
void setup_ap();
void connect_and_send(WiFiClient &client, const Packet &packet);