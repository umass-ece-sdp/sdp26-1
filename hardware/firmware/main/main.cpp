#include <Arduino.h>
#include "wifi_client.h"
#include "glove.h"

// Data Rate for IMU (Hz, uncomment desired rate)
// const int IMU_dataRate = 400;
// const int IMU_dataRate = 200;
// const int IMU_dataRate = 100;
const int IMU_dataRate = 50;
// const int IMU_dataRate = 25;
// const int IMU_dataRate = 10;
// const int IMU_dataRate = 1;

// Performance Mode for IMU (uncomment for mode)
// const int IMU_perfMode = 0;// 0 for low-power - 8bit
const int IMU_perfMode = 1; // 1 for normal - 10bit
// const int IMU_perfMode = 2;// 2 for high-resolution - 12bit

// Range for IMU (G's, uncomment for desired range)
// const int IMU_range = 2;
const int IMU_range = 4;
// const int IMU_range = 8;
// const int IMU_range = 16;

// IMU sensor
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
sensors_event_t accel;
bool lisOK = false; // ensure IMU stays connected

// Assign repeated variables to improve efficiency
float finger_readings[4];
float IMU_readings[3];
float UWB_distance = 0.0f;  // Eventually have real readings here
Packet packet;
WiFiClient client;

void setup()
{
    Serial.begin(115200);
    delay(3000); // delay to allow serial monitor to connect for debugging

    Serial.println("\n[MAIN] Starting ESP32 Client...");

    setup_glove();
    setup_wifi();
    setup_IMU(lis, lisOK, IMU_perfMode, IMU_range, IMU_dataRate);
}

void loop()
{
    // Read data from the finger sensors and the IMU
    read_fingers(finger_readings);
    if (lisOK)
    {
        read_IMU(lis, accel, IMU_readings);
    }

    // Store data in a packet to send to Base
    store_data(packet, finger_readings, IMU_readings, UWB_distance);

    // // Try to connect and send data
    connect_and_send(client, packet);

    // Wait before reading and sending next update
    delay(1000);
}