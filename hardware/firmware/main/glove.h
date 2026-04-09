#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Packet to send to Base
struct Packet
{
	float finger1; // Finger 1 input from the stretch sensors
	float finger2; // Finger 2 input from the stretch sensors
	float finger3; // Finger 3 input from the stretch sensors
	float finger4; // Finger 4 input from the stretch sensors
	float speed;   // Calculated user speed
	float dist;	   // Distance measured between glove and drone
};

// Pin Definitions
#define STRETCH_PIN_1 4
#define STRETCH_PIN_2 6
#define STRETCH_PIN_3 15
#define STRETCH_PIN_4 17
#define IMU_SDA_PIN 38
#define IMU_SCL_PIN 37
#define UWB_RX_PIN 44 // UART RX — UWB Module
#define UWB_TX_PIN 43 // UART TX — UWB Module
#define EVENT_LISTENER_PIN 18

// Constants
#define ADC_RESOLUTION 4096.0f	   // 12-bit ADC (0–4095)
#define ADC_VREF_V 3.3f			   // ESP32-S3 ADC reference (V)
#define GRAVITY_ALPHA 0.96f		   // LPF for gravity estimate
#define BIAS_ALPHA 0.02f		   // Bias update rate at rest
#define ACCEL_DEADBAND 0.08f	   // Ignore tiny accel noise (m/s^2)
#define STATIONARY_THRESH 0.18f	   // Near-rest accel magnitude (m/s^2)
#define VELOCITY_DAMPING 1.8f	   // Leak factor (1/s) to limit drift
#define VELOCITY_ZERO_THRESH 0.04f // Snap near-zero speed to zero (m/s)

// Functions
void setup_glove();
void setup_IMU(Adafruit_LIS3DH &imu, bool &imuOK, const int &accelRange, const int &dataRate);
void setup_UWB();
bool read_listener();
void read_fingers(float (&reading)[4]);
void read_IMU(Adafruit_LIS3DH &imu, sensors_event_t &accel, float (&accel_reading)[3]);
float get_UWB_distance(HardwareSerial &uwbSerial, const char *tagAddress);
void store_data(Packet &packet, const float (&finger_readings)[4], const float &speed, const float &UWB_distance);
static float magnitude(const float x, const float y, const float z);
void filter_IMU(const float (&accel_reading)[3], uint32_t &lastIMUus, bool &imuFilterReady, float (&gravity_est)[3], float (&accel_bias)[3], float (&velocity)[3], float (&linear_accel)[3]);
void calc_speed(float &speed, float (&velocity)[3], const float (&linear_accel)[3]);
