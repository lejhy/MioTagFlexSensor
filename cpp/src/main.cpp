#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
Madgwick filter;
unsigned long micros_per_reading, micros_previous;
float accl_scale, gyro_scale;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Started");
    int imu = IMU.begin();
    if (!imu) {
    Serial.println("Failed to initialize IMU!");
    while (1);
    }
    filter.begin(119);
    micros_per_reading = 1000000/119;
    micros_previous = micros();
}

void loop() {
    float ax, ay, az; // acclerometer reading is already in G / sec
    float gx, gy, gz; // gyroscope reading is already in degrees / sec
    float mx, my, mz; // magnetic field in micro teslas uT
    float roll, pitch, heading;
    unsigned long micros_now;
    micros_now = micros();
    if (micros_now - micros_previous >= micros_per_reading) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);

        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz); // for all 3
        // filter.updateIMU(gx, gy, gz, ax, ay, az);//only for accl, gyro
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
        Serial.println(roll);
        // Serial.print("Orientation: ");
        // Serial.print(heading);
        // Serial.print("| Pitch: ");
        // Serial.print(pitch);
        // Serial.print("| Roll: ");
        // Serial.println(roll);

        micros_previous = micros_previous + micros_per_reading;
    }
}
