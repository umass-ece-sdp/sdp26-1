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
	float accel_x; // Accelerometer x coordinate
	float accel_y;
	float accel_z;
	float dist; // distance measured between glove and drone
};

// Pin Definitions
#define FINGER_PIN_1 4
#define FINGER_PIN_2 6
#define FINGER_PIN_3 15
#define FINGER_PIN_4 17
#define IMU_SDA_PIN 38
#define IMU_SCL_PIN 37

// Function prototypes
void setup_glove();
void setup_IMU(Adafruit_LIS3DH &lis, bool &lisOK, const int &perfMode, const int &range, const int &dataRate);
void read_fingers(float (&reading)[4]);
void read_IMU(Adafruit_LIS3DH &lis, sensors_event_t &accel, float (&reading)[3]);
void store_data(Packet &packet, const float (&finger_readings)[4], const float (&IMU_readings)[3], const float &UWB_distance);
