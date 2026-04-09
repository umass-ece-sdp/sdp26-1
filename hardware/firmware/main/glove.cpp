#include "glove.h"

// Constants
static constexpr float ADC_RESOLUTION = 4096.0f; // 12-bit ADC (0–4095)
static constexpr float ADC_VREF_V = 3.3f;		 // ESP32-S3 ADC reference (V)

void setup_glove()
{
	Serial.println("[GLOVE] Initializing pins...");
	pinMode(STRETCH_PIN_1, INPUT);
	pinMode(STRETCH_PIN_2, INPUT);
	pinMode(STRETCH_PIN_3, INPUT);
	pinMode(STRETCH_PIN_4, INPUT);
	Serial.println("[GLOVE] Finger pins initialized");
	pinMode(IMU_SCL_PIN, INPUT);
	pinMode(IMU_SDA_PIN, INPUT);
	Serial.println("[GLOVE] IMU pins initialized");
}

void setup_IMU(Adafruit_LIS3DH &imu, bool &imuOK, const int &accelRange, const int &dataRate)
{
	Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
	if (!imu.begin(0x18)) // LIS3DH I2C address is usually 0x18 or 0x19
	{
		Serial.println("[IMU] LIS3DH not found - check wiring and I2C address. IMU reads will be skipped.");
		imuOK = false;
		return;
	}

	imuOK = true;
	Serial.println("[IMU] LIS3DH successfully connected.");

	// Set accelerometer range
	switch (accelRange)
	{
	case 2:
		imu.setRange(LIS3DH_RANGE_2_G);
		break;
	case 4:
		imu.setRange(LIS3DH_RANGE_4_G);
		break;
	case 8:
		imu.setRange(LIS3DH_RANGE_8_G);
		break;
	case 16:
		imu.setRange(LIS3DH_RANGE_16_G);
		break;
	}
	Serial.printf("[IMU] LIS3DH accelerometer range set to ±%d G\n", accelRange);

	// Set data rate
	switch (dataRate)
	{
	case 1:
		imu.setDataRate(LIS3DH_DATARATE_1_HZ);
		break;
	case 10:
		imu.setDataRate(LIS3DH_DATARATE_10_HZ);
		break;
	case 25:
		imu.setDataRate(LIS3DH_DATARATE_25_HZ);
		break;
	case 50:
		imu.setDataRate(LIS3DH_DATARATE_50_HZ);
		break;
	case 100:
		imu.setDataRate(LIS3DH_DATARATE_100_HZ);
		break;
	case 200:
		imu.setDataRate(LIS3DH_DATARATE_200_HZ);
		break;
	case 400:
		imu.setDataRate(LIS3DH_DATARATE_400_HZ);
		break;
	}
	Serial.printf("[IMU] LIS3DH data rate set to %d Hz\n", dataRate);
}

void read_fingers(float (&reading)[4])
{
	reading[0] = (analogRead(STRETCH_PIN_1) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[1] = (analogRead(STRETCH_PIN_2) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[2] = (analogRead(STRETCH_PIN_3) / ADC_RESOLUTION) * ADC_VREF_V;
	reading[3] = (analogRead(STRETCH_PIN_4) / ADC_RESOLUTION) * ADC_VREF_V;
}

void read_IMU(Adafruit_LIS3DH &imu, sensors_event_t &accel, float (&accel_reading)[3])
{
	imu.getEvent(&accel);
	accel_reading[0] = accel.acceleration.x;
	accel_reading[1] = accel.acceleration.y;
	accel_reading[2] = accel.acceleration.z;
}

float get_UWB_distance(HardwareSerial &uwbSerial, const char *targetTag)
{
	// Clear out any old garbage in the Serial buffer
	while (uwbSerial.available())
	{
		uwbSerial.read();
	}

	// Send the "AT+ANCHOR_SEND" command
	uwbSerial.print("AT+ANCHOR_SEND=");
	uwbSerial.print(targetTag);
	uwbSerial.print(",4,Ping\r\n");

	// Wait for the response
	unsigned long start_time = millis();
	while (millis() - start_time < 500) // 500ms timeout
	{
		if (uwbSerial.available())
		{
			String response = uwbSerial.readStringUntil('\n');
			response.trim();

			// Expected response: +RCV=TAG12345,4,Ping,-80,2.34
			if (response.startsWith("+RCV="))
			{
				int lastComma = response.lastIndexOf(',');
				if (lastComma != -1)
				{
					// Parse the distance (last component) as a float
					String distanceStr = response.substring(lastComma + 1);
					return distanceStr.toFloat();
				}
			}
		}
	}

	// Return a negative value to indicate a timeout or failed reading
	return -1.0f;
}

void store_data(Packet &packet, const float (&finger_readings)[4], const float (&accel_readings)[3], const float &UWB_distance)
{
	packet.finger1 = finger_readings[0];
	packet.finger2 = finger_readings[1];
	packet.finger3 = finger_readings[2];
	packet.finger4 = finger_readings[3];
	packet.accel_x = accel_readings[0];
	packet.accel_y = accel_readings[1];
	packet.accel_z = accel_readings[2];
	packet.dist = UWB_distance;
}