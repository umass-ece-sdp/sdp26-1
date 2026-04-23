#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include "glove.h"

// WiFi AP configuration (Glove creates this AP for base station to connect to)
#define AP_SSID "FALCON-Glove"
#define AP_PASS "team1-falcon"
#define AP_IP_1 192
#define AP_IP_2 168
#define AP_IP_3 4
#define AP_IP_4 1
#define PORT 5000
#define WAIT_TIME 90  // milliseconds between sensor readings

// Function prototypes
void setup_ap();
void accept_and_serve(WiFiServer &server, WiFiClient &client, const Packet &packet);
