#include "glove.h"

void setup_glove()
{
	Serial.println("[GLOVE] Initializing pins...");
	pinMode(STRETCH_PIN_1, INPUT);
	pinMode(STRETCH_PIN_2, INPUT);
	pinMode(STRETCH_PIN_3, INPUT);
	pinMode(STRETCH_PIN_4, INPUT);
	Serial.println("[GLOVE] Stretch pins initialized");
	pinMode(IMU_SCL_PIN, INPUT);
	pinMode(IMU_SDA_PIN, INPUT);
	Serial.println("[GLOVE] IMU pins initialized");
	pinMode(EVENT_LISTENER_PIN, INPUT);
	Serial.println("[GLOVE] Event Listener pin initialized");
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

void setup_UWB()
{
	Serial1.begin(115200, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
	Serial.println("[INIT] UWB Module initialized on Serial1.");
}

void read_listener(bool &listen)
{
	if (digitalRead(EVENT_LISTENER_PIN) == HIGH)
	{
		listen = !listen;
	}
}

void read_fingers(float (&reading)[4], bool &listen)
{
	if (listen)
	{
		reading[0] = (analogRead(STRETCH_PIN_1) / ADC_RESOLUTION) * ADC_VREF_V;
		reading[1] = (analogRead(STRETCH_PIN_2) / ADC_RESOLUTION) * ADC_VREF_V;
		reading[2] = (analogRead(STRETCH_PIN_3) / ADC_RESOLUTION) * ADC_VREF_V;
		reading[3] = (analogRead(STRETCH_PIN_4) / ADC_RESOLUTION) * ADC_VREF_V;
	}
	else
	{
		reading[0] = -1.0f;
		reading[1] = -1.0f;
		reading[2] = -1.0f;
		reading[3] = -1.0f;
	}
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

void store_data(Packet &packet, const float (&finger_readings)[4], const float &speed, const float &UWB_distance)
{
	packet.finger1 = finger_readings[0];
	packet.finger2 = finger_readings[1];
	packet.finger3 = finger_readings[2];
	packet.finger4 = finger_readings[3];
	packet.speed = speed;
	packet.dist = UWB_distance;
}

float magnitude(const float x, const float y, const float z)
{
	return sqrtf((x * x) + (y * y) + (z * z));
}

void filter_IMU(const float (&accel_reading)[3], uint32_t &lastIMUus, bool &imuFilterReady, float (&gravity_est)[3], float (&accel_bias)[3], float (&velocity)[3], float (&linear_accel)[3])
{
	// Prepare filter
	const uint32_t nowUs = micros();
	float dt = 0.02f; // Default to 50Hz (20ms); will be corrected after first sample
	if (!imuFilterReady)
	{
		gravity_est[0] = accel_reading[0];
		gravity_est[1] = accel_reading[1];
		gravity_est[2] = accel_reading[2];
		lastIMUus = nowUs;
		imuFilterReady = true;
		return; // Skip velocity update on first sample
	}
	else
	{
		dt = (nowUs - lastIMUus) * 1e-6f;
		// Clamp dt to reasonable range: [5ms, 100ms] for 50Hz nominal rate
		if (dt < 0.005f || dt > 0.1f)
		{
			dt = 0.02f;
		}
	}
	lastIMUus = nowUs;

	// Estimate gravity
	gravity_est[0] = (GRAVITY_ALPHA * gravity_est[0]) + ((1.0f - GRAVITY_ALPHA) * accel_reading[0]);
	gravity_est[1] = (GRAVITY_ALPHA * gravity_est[1]) + ((1.0f - GRAVITY_ALPHA) * accel_reading[1]);
	gravity_est[2] = (GRAVITY_ALPHA * gravity_est[2]) + ((1.0f - GRAVITY_ALPHA) * accel_reading[2]);

	// Normalize accel reading with gravity estimation
	linear_accel[0] = accel_reading[0] - gravity_est[0];
	linear_accel[1] = accel_reading[1] - gravity_est[1];
	linear_accel[2] = accel_reading[2] - gravity_est[2];

	// Calculate bias + apply filter
	const float linMag = magnitude(linear_accel[0], linear_accel[1], linear_accel[2]);
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
}

void calc_speed(float &speed, float (&velocity)[3], const float (&linear_accel)[3])
{
	// Calculate speed
	speed = magnitude(velocity[0], velocity[1], velocity[2]);
	if ((magnitude(linear_accel[0], linear_accel[1], linear_accel[2]) < STATIONARY_THRESH) && (speed < VELOCITY_ZERO_THRESH))
	{
		velocity[0] = 0.0f;
		velocity[1] = 0.0f;
		velocity[2] = 0.0f;
		speed = 0.0f;
	}
}
