#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Packet to send to Base
struct Packet
{
	float finger1; // Finger x input from the stretch sensors
	float finger2;
	float finger3;
	float finger4;
	float accel_x; // Accelerometer x coordinate (m/s²)
	float accel_y;
	float accel_z;
	float dist; // distance measured between glove and drone
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

// Function prototypes
void setup_glove();
void setup_IMU(Adafruit_LIS3DH &imu, bool &imuOK, const int &accelRange, const int &dataRate);
void read_fingers(float (&reading)[4]);
void read_IMU(Adafruit_LIS3DH &imu, sensors_event_t &accel, float (&accel_reading)[3]);
float get_UWB_distance(HardwareSerial &uwbSerial, const char* tagAddress);
void store_data(Packet &packet, const float (&finger_readings)[4], const float (&accel_readings)[3], const float &UWB_distance);
