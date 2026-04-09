// =============================================================================
// test.cpp - Hardware Bring-Up Test
// Target : ESP32-S3 (PlatformIO / Arduino framework)
// Tests  : 4× rubber stretch sensors (ADC),
//          MPU-6050 6-DoF IMU (I2C, accelerometer + gyroscope)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "glove.h"

// ── Pin Definitions ───────────────────────────────────────────────────────────

// #define STRETCH_PIN_1 4  // ADC input — stretch sensor 1
// #define STRETCH_PIN_2 6  // ADC input — stretch sensor 2
// #define STRETCH_PIN_3 15 // ADC input — stretch sensor 3
// #define STRETCH_PIN_4 17 // ADC input — stretch sensor 4

// #define STRETCH_PIN_4 4  // ADC input — stretch sensor 1
// #define STRETCH_PIN_3 6  // ADC input — stretch sensor 2
// #define STRETCH_PIN_2 15 // ADC input — stretch sensor 3
// #define STRETCH_PIN_1 17 // ADC input — stretch sensor 4

// #define IMU_SDA_PIN 38 // I2C SDA — MPU-6050
// #define IMU_SCL_PIN 37 // I2C SCL — MPU-6050

// ── Constants ────────────────────────────────────────────────────────────────

static constexpr float ADC_RESOLUTION = 4096.0f; // 12-bit ADC (0–4095)
static constexpr float ADC_VREF_V = 3.3f;        // ESP32-S3 ADC reference (V)

// ── Globals ──────────────────────────────────────────────────────────────────

Adafruit_LIS3DH lis;
bool imuOK = false;

// =============================================================================
void setup()
{
    Serial.begin(115200);
    delay(3000); // wait for serial monitor to connect (esp. USB CDC)

    Serial.println("\n========================================");
    Serial.println("  Hardware Bring-Up Test");
    Serial.println("========================================\n");

    // ── Stretch sensor pins ───────────────────────────────────────────────────
    pinMode(STRETCH_PIN_1, INPUT);
    pinMode(STRETCH_PIN_2, INPUT);
    pinMode(STRETCH_PIN_3, INPUT);
    pinMode(STRETCH_PIN_4, INPUT);
    pinMode(EVENT_LISTENER_PIN, INPUT);
    Serial.println("[INIT] Stretch sensor ADC pins configured.");

    // ── LIS3DH (I2C) ────────────────────────────────────────────────────────
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    if (!lis.begin(0x18))
    {
        Serial.println("[ERROR] LIS3DH not found - check wiring and I2C address. IMU reads will be skipped.");
    }
    else
    {
        imuOK = true;
        Serial.println("[INIT] LIS3DH found.");

        lis.setRange(LIS3DH_RANGE_4_G);
        lis.setDataRate(LIS3DH_DATARATE_50_HZ);

        Serial.println("[INIT] LIS3DH configured (+/- 4 G, 50 Hz).");
    }

    // ── UWB Module ────────────────────────────────────────────────────────────
    Serial1.begin(115200, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
    Serial.println("[INIT] UWB Module initialized on Serial1.");

    Serial.println("\n--- Starting sensor loop ---\n");
    delay(200);
}

// =============================================================================
void loop()
{
    // ── Stretch Sensors ───────────────────────────────────────────────────────
    int rawS[4];
    float voltS[4];
    int stretchPins[4] = {STRETCH_PIN_1, STRETCH_PIN_2,
                          STRETCH_PIN_3, STRETCH_PIN_4};

    for (int i = 0; i < 4; i++)
    {
        rawS[i] = analogRead(stretchPins[i]);
        voltS[i] = (rawS[i] / ADC_RESOLUTION) * ADC_VREF_V;
    }

    Serial.println("=== Stretch Sensors ===");
    for (int i = 0; i < 4; i++)
    {
        Serial.printf("  Sensor %d  |  raw: %4d  |  voltage: %.3f V\n",
                      i + 1, rawS[i], voltS[i]);
    }

    // ── LIS3DH IMU ──────────────────────────────────────────────────────────
    if (imuOK)
    {
        sensors_event_t accel;
        lis.getEvent(&accel);
        Serial.println("\n=== LIS3DH IMU ===");
        Serial.printf("  Accel  X: %8.4f  Y: %8.4f  Z: %8.4f  m/s²\n",
                      accel.acceleration.x,
                      accel.acceleration.y,
                      accel.acceleration.z);
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

    delay(500);
}
