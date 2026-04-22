#include "glove.h"
#include "wifi_client.h"

// Globals
Adafruit_LIS3DH lis;
sensors_event_t accel;
WiFiClient client;
float finger_reading[4];
float speed;
float distance;
bool listen = false;
bool imuOK = false;
float accel_reading[3] = {0.0f, 0.0f, 0.0f};
float gravity_est[3] = {0.0f, 0.0f, 0.0f};
float accel_bias[3] = {0.0f, 0.0f, 0.0f};
float linear_accel[3] = {0.0f, 0.0f, 0.0f};
float velocity[3] = {0.0f, 0.0f, 0.0f};
uint32_t lastIMUus = 0;
bool imuFilterReady = false;
Packet packet;

void setup()
{
    Serial.begin(115200);
    delay(500);

    // Initialize pins for finger sensors, IMU, UWB, setup WiFi
    setup_glove();
    setup_IMU(lis, imuOK, 4, 50);
    // setup_UWB();
    setup_wifi();

    Serial.println("[Glove] Initialization complete");

    delay(200);
}

void loop()
{
    // Read stretch sensors
	read_listener(listen);
    read_fingers(finger_reading, listen);

    // Read IMU data
    if (imuOK)
    {
        read_IMU(lis, accel, accel_reading);
        filter_IMU(accel_reading, lastIMUus, imuFilterReady, gravity_est, accel_bias, velocity, linear_accel);
        calc_speed(speed, velocity, linear_accel);
    }

    // Read UWB
    // distance = get_UWB_distance(Serial1, "TAG12345");
    // if (distance >= 0.0f)
    // {
    //     Serial.printf("  Distance to TAG12345: %.2f meters\n", distance);
    // }

    // Package data and send it to the glove
    store_data(packet, finger_reading, speed, distance);
    connect_and_send(client, packet);

    delay(WAIT_TIME);
}
