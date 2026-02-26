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

void setup_IMU(Adafruit_MPU6050 &mpu, bool &mpuOK, const int &accelRange, const int &gyroRange, const int &filterBandwidth)
{
	Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
	if (!mpu.begin())
	{
		Serial.println("[IMU] MPU-6050 not found - check wiring and I2C address. IMU reads will be skipped.");
		mpuOK = false;
		return;
	}

	mpuOK = true;
	Serial.println("[IMU] MPU-6050 successfully connected.");

	// Set accelerometer range
	switch (accelRange)
	{
	case 2:
		mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
		break;
	case 4:
		mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
		break;
	case 8:
		mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
		break;
	case 16:
		mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
		break;
	}
	Serial.printf("[IMU] MPU-6050 accelerometer range set to ±%d G\n", accelRange);

	// Set gyroscope range
	switch (gyroRange)
	{
	case 250:
		mpu.setGyroRange(MPU6050_RANGE_250_DEG);
		break;
	case 500:
		mpu.setGyroRange(MPU6050_RANGE_500_DEG);
		break;
	case 1000:
		mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
		break;
	case 2000:
		mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
		break;
	}
	Serial.printf("[IMU] MPU-6050 gyroscope range set to ±%d deg/s\n", gyroRange);

	// Set DLPF (digital low-pass filter) bandwidth
	switch (filterBandwidth)
	{
	case 260:
		mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
		break;
	case 184:
		mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
		break;
	case 94:
		mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
		break;
	case 44:
		mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
		break;
	case 21:
		mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
		break;
	case 10:
		mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
		break;
	case 5:
		mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
		break;
	}
	Serial.printf("[IMU] MPU-6050 filter bandwidth set to %d Hz\n", filterBandwidth);
}

void read_fingers(float (&reading)[4])
{
	reading[0] = (analogRead(FINGER_PIN_1) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[1] = (analogRead(FINGER_PIN_2) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[2] = (analogRead(FINGER_PIN_3) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[3] = (analogRead(FINGER_PIN_4) / ADC_RESOLUTION) * ADC_VREF_V;
}

void read_IMU(Adafruit_MPU6050 &mpu, sensors_event_t &accel, sensors_event_t &gyro, float (&accel_reading)[3], float (&gyro_reading)[3])
{
	sensors_event_t temp;
	mpu.getEvent(&accel, &gyro, &temp);
	accel_reading[0] = accel.acceleration.x;
	accel_reading[1] = accel.acceleration.y;
	accel_reading[2] = accel.acceleration.z;
	gyro_reading[0] = gyro.gyro.x;
	gyro_reading[1] = gyro.gyro.y;
	gyro_reading[2] = gyro.gyro.z;
}

void store_data(Packet &packet, const float (&finger_readings)[4], const float (&accel_readings)[3], const float (&gyro_readings)[3], const float &UWB_distance)
{
	packet.finger1 = finger_readings[0];
	packet.finger2 = finger_readings[1];
	packet.finger3 = finger_readings[2];
	packet.finger4 = finger_readings[3];
	packet.accel_x = accel_readings[0];
	packet.accel_y = accel_readings[1];
	packet.accel_z = accel_readings[2];
	packet.gyro_x  = gyro_readings[0];
	packet.gyro_y  = gyro_readings[1];
	packet.gyro_z  = gyro_readings[2];
	packet.dist = UWB_distance;
}