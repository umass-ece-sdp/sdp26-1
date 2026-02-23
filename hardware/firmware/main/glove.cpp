#include "glove.h"

// Constants
static constexpr float ADC_RESOLUTION = 4096.0f; // 12-bit ADC (0–4095)
static constexpr float ADC_VREF_V = 3.3f;		 // ESP32-S3 ADC reference (V)

void setup_glove()
{
	Serial.println("[GLOVE] Initializing pins...");
	pinMode(FINGER_PIN_1, INPUT);
	pinMode(FINGER_PIN_2, INPUT);
	pinMode(FINGER_PIN_3, INPUT);
	pinMode(FINGER_PIN_4, INPUT);
	Serial.println("[GLOVE] Finger pins initialized");
	pinMode(IMU_SCL_PIN, INPUT);
	pinMode(IMU_SDA_PIN, INPUT);
	Serial.println("[GLOVE] IMU pins initialized");
}

void setup_IMU(Adafruit_LIS3DH &lis, bool &lisOK, const int &perfMode, const int &range, const int &dataRate)
{
	Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
	if (!lis.begin(0x18))
	{
		Serial.println("[IMU] LIS3DH not found - check wiring and I2C address. IMU reads will be skipped.");
		lisOK = false;
	}
	else
	{
		lisOK = true;
		Serial.println("[IMU] LIS3DH successfully connected.");
	}

	// Set IMU performance mode
	switch (perfMode)
	{
	case 0:
		lis.setPerformanceMode(LIS3DH_MODE_LOW_POWER);
		Serial.println("[IMU] LIS3DH mode set to LOW-POWER");
		break;
	case 1:
		lis.setPerformanceMode(LIS3DH_MODE_NORMAL);
		Serial.println("[IMU] LIS3DH mode set to NORMAL");
		break;
	case 2:
		lis.setPerformanceMode(LIS3DH_MODE_HIGH_RESOLUTION);
		Serial.println("[IMU] LIS3DH mode set to HIGH-RESOLUTION");
		break;
	}

	// Set IMU range
	switch (range)
	{
	case 2:
		lis.setRange(LIS3DH_RANGE_2_G);
		break;
	case 4:
		lis.setRange(LIS3DH_RANGE_4_G);
		break;
	case 8:
		lis.setRange(LIS3DH_RANGE_8_G);
		break;
	case 16:
		lis.setRange(LIS3DH_RANGE_16_G);
		break;
	}
	Serial.printf("[IMU] LIS3DH range set to %d G\n", range);

	// Set IMU data rate
	switch (dataRate)
	{
	case 400:
		lis.setDataRate(LIS3DH_DATARATE_400_HZ);
		break;
	case 200:
		lis.setDataRate(LIS3DH_DATARATE_200_HZ);
		break;
	case 100:
		lis.setDataRate(LIS3DH_DATARATE_100_HZ);
		break;
	case 50:
		lis.setDataRate(LIS3DH_DATARATE_50_HZ);
		break;
	case 25:
		lis.setDataRate(LIS3DH_DATARATE_25_HZ);
		break;
	case 10:
		lis.setDataRate(LIS3DH_DATARATE_10_HZ);
		break;
	case 1:
		lis.setDataRate(LIS3DH_DATARATE_1_HZ);
		break;
	}
	Serial.printf("[IMU] LIS3DH data rate set to %d Hz\n", dataRate);
}

void read_fingers(float (&reading)[4])
{
	reading[0] = (analogRead(FINGER_PIN_1) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[1] = (analogRead(FINGER_PIN_2) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[2] = (analogRead(FINGER_PIN_3) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[3] = (analogRead(FINGER_PIN_4) / ADC_RESOLUTION) * ADC_VREF_V;
}

void read_IMU(Adafruit_LIS3DH &lis, sensors_event_t &accel, float (&reading)[3])
{
	lis.getEvent(&accel);
	reading[0] = accel.acceleration.x;
	reading[1] = accel.acceleration.y;
	reading[2] = accel.acceleration.z;
}

void store_data(Packet &packet, const float (&finger_readings)[4], const float (&IMU_readings)[3])
{
	packet.finger1 = finger_readings[0];
	packet.finger2 = finger_readings[1];
	packet.finger3 = finger_readings[2];
	packet.finger4 = finger_readings[3];
	packet.accel_x = IMU_readings[0];
	packet.accel_y = IMU_readings[1];
	packet.accel_z = IMU_readings[2];
}