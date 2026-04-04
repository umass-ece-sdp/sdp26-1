#include <Arduino.h>
#include "wifi_client.h"
#include "glove.h"

// Accelerometer range for IMU (G's, uncomment for desired range)
// const int IMU_accelRange = 2;
const int IMU_accelRange = 4;
// const int IMU_accelRange = 8;
// const int IMU_accelRange = 16;

// Gyroscope range for IMU (deg/s, uncomment desired range)
// const int IMU_gyroRange = 250;
const int IMU_gyroRange = 500;
// const int IMU_gyroRange = 1000;
// const int IMU_gyroRange = 2000;

// DLPF bandwidth for IMU (Hz, uncomment desired bandwidth)
// const int IMU_filterBandwidth = 260;
// const int IMU_filterBandwidth = 184;
// const int IMU_filterBandwidth = 94;
// const int IMU_filterBandwidth = 44;
const int IMU_filterBandwidth = 21;
// const int IMU_filterBandwidth = 10;
// const int IMU_filterBandwidth = 5;

// IMU sensor
Adafruit_MPU6050 mpu;
sensors_event_t accel;
sensors_event_t gyro;
bool mpuOK = false; // ensure IMU stays connected

// Assign repeated variables to improve efficiency
float finger_readings[4];
float accel_readings[3];
float gyro_readings[3];
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
    setup_IMU(mpu, mpuOK, IMU_accelRange, IMU_gyroRange, IMU_filterBandwidth);
}

void loop()
{
    // Read data from the finger sensors and the IMU
    read_fingers(finger_readings);
    if (mpuOK)
    {
        read_IMU(mpu, accel, gyro, accel_readings, gyro_readings);
    }

    // Store data in a packet to send to Base
    store_data(packet, finger_readings, accel_readings, gyro_readings, UWB_distance);

    // // Try to connect and send data
    connect_and_send(client, packet);

    // Wait before reading and sending next update
    delay(1000);
}