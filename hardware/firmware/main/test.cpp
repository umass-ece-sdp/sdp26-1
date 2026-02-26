// =============================================================================
// test.cpp — Hardware Bring-Up Test
// Target : ESP32-S3 (PlatformIO / Arduino framework)
// Tests  : 4× rubber stretch sensors (ADC),
//          MPU-6050 6-DoF IMU (I2C, accelerometer + gyroscope)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── Pin Definitions ───────────────────────────────────────────────────────────

#define STRETCH_PIN_1 4  // ADC input — stretch sensor 1
#define STRETCH_PIN_2 6  // ADC input — stretch sensor 2
#define STRETCH_PIN_3 15 // ADC input — stretch sensor 3
#define STRETCH_PIN_4 17 // ADC input — stretch sensor 4

#define IMU_SDA_PIN 38 // I2C SDA — MPU-6050
#define IMU_SCL_PIN 37 // I2C SCL — MPU-6050

// ── Constants ────────────────────────────────────────────────────────────────

static constexpr float ADC_RESOLUTION = 4096.0f; // 12-bit ADC (0–4095)
static constexpr float ADC_VREF_V = 3.3f;        // ESP32-S3 ADC reference (V)

// ── Globals ──────────────────────────────────────────────────────────────────

Adafruit_MPU6050 mpu;
bool mpuOK = false;

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
    Serial.println("[INIT] Stretch sensor ADC pins configured.");

    // ── MPU-6050 (I2C) ────────────────────────────────────────────────────────
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    if (!mpu.begin())
    {
        Serial.println("[ERROR] MPU-6050 not found — check wiring and I2C address. IMU reads will be skipped.");
    }
    else
    {
        mpuOK = true;
        Serial.println("[INIT] MPU-6050 found.");

        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

        Serial.println("[INIT] MPU-6050 configured (±4 G, ±500 deg/s, 21 Hz DLPF).");
    }

    Serial.println("\n--- Starting sensor loop ---\n");
    delay(200);
}

// =============================================================================
void loop()
{
    // ── Stretch Sensors ───────────────────────────────────────────────────────
    int   rawS[4];
    float voltS[4];
    int   stretchPins[4] = { STRETCH_PIN_1, STRETCH_PIN_2,
                             STRETCH_PIN_3, STRETCH_PIN_4 };

    for (int i = 0; i < 4; i++) {
        rawS[i]  = analogRead(stretchPins[i]);
        voltS[i] = (rawS[i] / ADC_RESOLUTION) * ADC_VREF_V;
    }

    Serial.println("=== Stretch Sensors ===");
    for (int i = 0; i < 4; i++) {
        Serial.printf("  Sensor %d  |  raw: %4d  |  voltage: %.3f V\n",
                      i + 1, rawS[i], voltS[i]);
    }

    // ── MPU-6050 IMU ──────────────────────────────────────────────────────────
    if (mpuOK)
    {
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);
        Serial.println("\n=== MPU-6050 IMU ===");
        Serial.printf("  Accel  X: %8.4f  Y: %8.4f  Z: %8.4f  m/s²\n",
                      accel.acceleration.x,
                      accel.acceleration.y,
                      accel.acceleration.z);
        Serial.printf("  Gyro   X: %8.4f  Y: %8.4f  Z: %8.4f  rad/s\n",
                      gyro.gyro.x,
                      gyro.gyro.y,
                      gyro.gyro.z);
        Serial.printf("  Temp:  %.2f °C\n", temp.temperature);
    }
    else
    {
        Serial.println("\n=== MPU-6050 IMU === [NOT CONNECTED]");
    }

    Serial.println("\n----------------------------------------\n");

    delay(500);
}
