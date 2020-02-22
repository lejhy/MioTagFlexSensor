#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <ArduinoBLE.h>

union Package {
  int16_t values[10];
  byte bytes[20]; // This is the maximum supported by React Native
};

Madgwick filter;
unsigned long micros_previous_reading;

//defines the services used
BLEService service("19B10000-E8F2-537E-4F6C-D104768A1214");

// defines all the characteristics used
BLECharacteristic characteristicIMU("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify, 20);
BLECharacteristic characteristicFingers("19B10002-E8F2-537E-4F6C-D104768A1214", BLENotify, 20);

// sets up LED pin used to indicate a connection to a central
const int ledPin = LED_BUILTIN;

void setup() {
  Serial.begin(9600);
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
  service.addCharacteristic(characteristicIMU);
  service.addCharacteristic(characteristicFingers);
  // adds the services to the profile
  BLE.addService(service);
  // advertises the device after everything has been set up
  BLE.advertise();
  // start timer
  micros_previous_reading = micros();
}

// IMU
float ax, ay, az; // acclerometer reading is already in G / sec
float gx, gy, gz; // gyroscope reading is already in degrees / sec
float mx, my, mz; // magnetic field in micro teslas uT

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
    // switch the LED on
    digitalWrite(ledPin, HIGH);
    // while the central is still connected to peripheral:
    while (central.connected()) {

      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        Package package;
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);
        
        long micros_reading = micros();
        // directly set the frequency instead of calling begin()
        filter.invSampleFreq = (micros_reading - micros_previous_reading) * 0.000001f;
        micros_previous_reading = micros_reading;
        filter.update(gx, gy, gz, ax, ay, az, -mx, my, mz); // for all 3

        package.values[0] = ax*100;
        package.values[1] = ay*100;
        package.values[2] = az*100;
        package.values[3] = gx;
        package.values[4] = gy;
        package.values[5] = gz;
        package.values[6] = filter.getRoll();
        package.values[7] = filter.getPitch();
        package.values[8] = filter.getYaw();

        Serial.println(filter.invSampleFreq);
        Serial.print("ROLL: ");
        Serial.print(package.values[6]);
        Serial.print(" | PITCH: ");
        Serial.print(package.values[7]);
        Serial.print(" | YAW: ");
        Serial.println(package.values[8]);

        characteristicIMU.writeValue(package.bytes, 18);
      }
    }
    // when the central disconnects, switch the LED off
    digitalWrite(ledPin, LOW);
  }
}
