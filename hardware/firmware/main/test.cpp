// =============================================================================
// test.cpp - Hardware Bring-Up Test
// Target : ESP32-S3 (PlatformIO / Arduino framework)
// Tests  : 4× rubber stretch sensors (ADC),
//          MPU-6050 6-DoF IMU (I2C, accelerometer + gyroscope)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "glove.h"

// ── Globals ──────────────────────────────────────────────────────────────────

Adafruit_LIS3DH lis;
sensors_event_t accel;
float finger_reading[4];
float speed;
bool imuOK = false;
float accel_reading[3] = {0.0f, 0.0f, 0.0f};
float gravity_est[3] = {0.0f, 0.0f, 0.0f};
float accel_bias[3] = {0.0f, 0.0f, 0.0f};
float linear_accel[3] = {0.0f, 0.0f, 0.0f};
float velocity[3] = {0.0f, 0.0f, 0.0f};
uint32_t lastIMUus = 0;
bool imuFilterReady = false;

// =============================================================================
void setup()
{
    Serial.begin(115200);
    delay(3000); // wait for serial monitor to connect (esp. USB CDC)

    Serial.println("\n========================================");
    Serial.println("  Hardware Bring-Up Test");
    Serial.println("========================================\n");

    // ── Stretch sensor pins ───────────────────────────────────────────────────
    setup_glove();

    // ── LIS3DH (I2C) ────────────────────────────────────────────────────────
    setup_IMU(lis, imuOK, 4, 50);

    // ── UWB Module ────────────────────────────────────────────────────────────
    setup_UWB();

    Serial.println("\n--- Starting sensor loop ---\n");
    delay(200);
}

// =============================================================================
void loop()
{
    // ── Stretch Sensors ───────────────────────────────────────────────────────
    Serial.println("=== Stretch Sensors ===");
    read_fingers(finger_reading);
    for (int i = 0; i < 4; i++)
    {
        Serial.printf("  Sensor %d  |  voltage: %.3f V\n",
                      i + 1, finger_reading[i]);
    }

    // ── LIS3DH IMU ──────────────────────────────────────────────────────────
    if (imuOK)
    {
        read_IMU(lis, accel, accel_reading);
        filter_IMU(accel_reading, lastIMUus, imuFilterReady, gravity_est, accel_bias, velocity, linear_accel);
        calc_speed(speed, velocity, linear_accel);

        Serial.println("\n=== LIS3DH IMU ===");
        Serial.printf("  Accel  X: %8.4f  Y: %8.4f  Z: %8.4f  m/s^2\n",
                      accel_reading[0],
                      accel_reading[1],
                      accel_reading[2]);

        Serial.printf("  Linear accel X: %8.4f  Y: %8.4f  Z: %8.4f  m/s^2\n",
                      linear_accel[0],
                      linear_accel[1],
                      linear_accel[2]);
        Serial.printf("  Speed (magnitude): %.4f m/s\n", speed);
    }
    else
    {
        Serial.println("\n=== LIS3DH IMU === [NOT CONNECTED]");
    }

    // ── UWB Module ────────────────────────────────────────────────────────────
    // Serial.println("\n=== UWB Module ===");
    // Poll the anchor for the distance to TAG12345 (the ID you found on your Tag)
    // float distance = get_UWB_distance(Serial1, "TAG12345");

    // if (distance >= 0.0f)
    // {
    //     Serial.printf("  Distance to TAG12345: %.2f meters\n", distance);
    // }
    // else
    // {
    //     Serial.println("  Distance: [TIMEOUT or NO TAG RESPONSE]");
    // }

    Serial.println("\n----------------------------------------\n");

    delay(50);
}
