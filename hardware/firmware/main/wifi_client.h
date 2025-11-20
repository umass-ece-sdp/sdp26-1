#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <glove.h>

// WiFi configuration
#define WIFI_SSID "jetson_nano_wifi"
#define WIFI_PASS "team1-falcon"
#define HOST_IP "192.168.20.1"
#define PORT 5000
#define WAIT_TIME 1000 // in milliseconds

// Function prototypes
void setup_wifi();
// void connect_and_send();