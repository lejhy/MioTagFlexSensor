#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <ArduinoBLE.h>

Madgwick filter;
const unsigned long imu_frequency = 119;
const unsigned long micros_per_reading = 1000000 / imu_frequency;
unsigned long micros_previous;
unsigned long micros_now;

//defines the two services used
BLEService fingerService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEService IMUService("19B10001-E8F2-537E-4F6C-D104768A1214");

// defines all the characteristics used
BLEIntCharacteristic Pinkie("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Index("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Middle("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Ring("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Thumb("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Roll("19B10007-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Pitch("19B10008-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Yaw("19B10009-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// sets up LED pin used to indicate a connection to a central
const int ledPin = LED_BUILTIN;

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    // setup BLE and IMU
    if (!BLE.begin()) {
      while (1);
    }
    if (!IMU.begin()) {
      while (1);
    }
    // set advertised device name as MIOTAG
    BLE.setDeviceName("MIOTAG");
    BLE.setLocalName("MIOTAG");
    // adds the characteristics to the appropriate services
    fingerService.addCharacteristic(Pinkie);
    fingerService.addCharacteristic(Index);
    fingerService.addCharacteristic(Middle);
    fingerService.addCharacteristic(Ring);
    fingerService.addCharacteristic(Thumb);
    IMUService.addCharacteristic(Pitch);
    IMUService.addCharacteristic(Roll);
    IMUService.addCharacteristic(Yaw);
    // adds the services to the profile
    BLE.addService(fingerService);
    BLE.addService(IMUService);
    // advertises the device after everything has been set up
    BLE.advertise();
    // start filter
    filter.begin(imu_frequency);
    micros_previous = micros();
}

void loop() {
    // listen for BLE peripherals to connect:
    BLEDevice central = BLE.central();
    // if a central is connected to peripheral:
    if (central) {
      // switch the LED on
      digitalWrite(ledPin,HIGH);
      // while the central is still connected to peripheral:
      while (central.connected()) {
        // IMU
        float ax, ay, az; // acclerometer reading is already in G / sec
        float gx, gy, gz; // gyroscope reading is already in degrees / sec
        float mx, my, mz; // magnetic field in micro teslas uT

        micros_now = micros();
        if (micros_now - micros_previous >= micros_per_reading) {
            IMU.readAcceleration(ax, ay, az);
            IMU.readGyroscope(gx, gy, gz);
            IMU.readMagneticField(mx, my, mz);

            filter.update(gx, gy, gz, ax, ay, az, mx, my, mz); // for all 3

            Roll.writeValue(filter.getRoll());
            Pitch.writeValue(filter.getPitch());
            Yaw.writeValue(filter.getYaw());

            micros_previous = micros_previous + micros_per_reading;
        }
      }
      // when the central disconnects, switch the LED off
      digitalWrite(ledPin,LOW);
    }
}
