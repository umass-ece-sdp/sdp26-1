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

// ── Constants ────────────────────────────────────────────────────────────────

static constexpr float ADC_RESOLUTION = 4096.0f;     // 12-bit ADC (0–4095)
static constexpr float ADC_VREF_V = 3.3f;            // ESP32-S3 ADC reference (V)
static constexpr float GRAVITY_ALPHA = 0.96f;        // LPF for gravity estimate
static constexpr float BIAS_ALPHA = 0.02f;           // Bias update rate at rest
static constexpr float ACCEL_DEADBAND = 0.08f;       // Ignore tiny accel noise (m/s^2)
static constexpr float STATIONARY_THRESH = 0.18f;    // Near-rest accel magnitude (m/s^2)
static constexpr float VELOCITY_DAMPING = 1.8f;      // Leak factor (1/s) to limit drift
static constexpr float VELOCITY_ZERO_THRESH = 0.04f; // Snap near-zero speed to zero (m/s)

// ── Globals ──────────────────────────────────────────────────────────────────

Adafruit_LIS3DH lis;
bool imuOK = false;
float gravity_est[3] = {0.0f, 0.0f, 0.0f};
float accel_bias[3] = {0.0f, 0.0f, 0.0f};
float velocity[3] = {0.0f, 0.0f, 0.0f};
uint32_t lastIMUus = 0;
bool imuFilterReady = false;

static float magnitude3(const float x, const float y, const float z)
{
    return sqrtf((x * x) + (y * y) + (z * z));
}

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
    const bool listenerPressed = (digitalRead(EVENT_LISTENER_PIN) == HIGH);

    for (int i = 0; i < 4; i++)
    {
        rawS[i] = analogRead(stretchPins[i]);
        voltS[i] = (rawS[i] / ADC_RESOLUTION) * ADC_VREF_V;
    }

    if (!listenerPressed)
    {
        for (int i = 0; i < 4; i++)
        {
            voltS[i] = 0.0f;
        }
    }

    Serial.println("=== Stretch Sensors ===");
    Serial.printf("  Listener button: %s\n", listenerPressed ? "PRESSED" : "NOT PRESSED (GATED)");
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

        const uint32_t nowUs = micros();
        float dt = 0.02f;

        if (!imuFilterReady)
        {
            gravity_est[0] = accel.acceleration.x;
            gravity_est[1] = accel.acceleration.y;
            gravity_est[2] = accel.acceleration.z;
            lastIMUus = nowUs;
            imuFilterReady = true;
        }
        else
        {
            dt = (nowUs - lastIMUus) * 1e-6f;
            lastIMUus = nowUs;
            if (dt <= 0.0f || dt > 0.2f)
            {
                dt = 0.02f;
            }
        }

        gravity_est[0] = (GRAVITY_ALPHA * gravity_est[0]) + ((1.0f - GRAVITY_ALPHA) * accel.acceleration.x);
        gravity_est[1] = (GRAVITY_ALPHA * gravity_est[1]) + ((1.0f - GRAVITY_ALPHA) * accel.acceleration.y);
        gravity_est[2] = (GRAVITY_ALPHA * gravity_est[2]) + ((1.0f - GRAVITY_ALPHA) * accel.acceleration.z);

        float linear_accel[3] = {
            accel.acceleration.x - gravity_est[0],
            accel.acceleration.y - gravity_est[1],
            accel.acceleration.z - gravity_est[2]};

        const float linMag = magnitude3(linear_accel[0], linear_accel[1], linear_accel[2]);
        if (linMag < STATIONARY_THRESH)
        {
            for (int i = 0; i < 3; i++)
            {
                accel_bias[i] = ((1.0f - BIAS_ALPHA) * accel_bias[i]) + (BIAS_ALPHA * linear_accel[i]);
            }
        }

        for (int i = 0; i < 3; i++)
        {
            linear_accel[i] -= accel_bias[i];
            if (fabsf(linear_accel[i]) < ACCEL_DEADBAND)
            {
                linear_accel[i] = 0.0f;
            }
        }

        const float damping = expf(-VELOCITY_DAMPING * dt);
        for (int i = 0; i < 3; i++)
        {
            velocity[i] += linear_accel[i] * dt;
            velocity[i] *= damping;
        }

        float speed = magnitude3(velocity[0], velocity[1], velocity[2]);
        if ((magnitude3(linear_accel[0], linear_accel[1], linear_accel[2]) < STATIONARY_THRESH) &&
            (speed < VELOCITY_ZERO_THRESH))
        {
            velocity[0] = 0.0f;
            velocity[1] = 0.0f;
            velocity[2] = 0.0f;
            speed = 0.0f;
        }

        Serial.println("\n=== LIS3DH IMU ===");
        Serial.printf("  Accel  X: %8.4f  Y: %8.4f  Z: %8.4f  m/s²\n",
                      accel.acceleration.x,
                      accel.acceleration.y,
                      accel.acceleration.z);

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
