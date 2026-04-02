#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
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
	float gyro_x;  // Gyroscope x coordinate (rad/s)
	float gyro_y;
	float gyro_z;
	float dist; // distance measured between glove and drone
};

// Pin Definitions
#define FINGER_PIN_1 4
#define FINGER_PIN_2 6
#define FINGER_PIN_3 15
#define FINGER_PIN_4 17
#define IMU_SDA_PIN 38
#define IMU_SCL_PIN 37
#define UWB_RX_PIN 44 // UART RX — UWB Module
#define UWB_TX_PIN 43 // UART TX — UWB Module

// Function prototypes
void setup_glove();
void setup_IMU(Adafruit_MPU6050 &mpu, bool &mpuOK, const int &accelRange, const int &gyroRange, const int &filterBandwidth);
void read_fingers(float (&reading)[4]);
void read_IMU(Adafruit_MPU6050 &mpu, sensors_event_t &accel, sensors_event_t &gyro, float (&accel_reading)[3], float (&gyro_reading)[3]);
float get_UWB_distance(HardwareSerial &uwbSerial, const char* tagAddress);
void store_data(Packet &packet, const float (&finger_readings)[4], const float (&accel_readings)[3], const float (&gyro_readings)[3], const float &UWB_distance);
