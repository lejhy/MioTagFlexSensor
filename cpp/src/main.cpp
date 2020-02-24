#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <ArduinoBLE.h>

struct ImuAndFingers {
  int16_t imu[6];
  u_int8_t fingers[5];
};

union ImuAndFingersPackage {
  ImuAndFingers values;
  byte bytes[17];
};

union QuaternionsPackage {
  float values[4];
  byte bytes[16];
};

Madgwick filter;
unsigned long micros_previous_reading;

//defines the services used
BLEService service("19B10000-E8F2-537E-4F6C-D104768A1214");

// defines all the characteristics used
BLECharacteristic characteristicImuAndFingers("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify, 20);
BLECharacteristic characteristicQuaternions("19B10002-E8F2-537E-4F6C-D104768A1214", BLENotify, 20);

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
  service.addCharacteristic(characteristicImuAndFingers);
  service.addCharacteristic(characteristicQuaternions);
  // adds the services to the profile
  BLE.addService(service);
  // advertises the device after everything has been set up
  BLE.advertise();
  // setup filter algorithm gain
  filter.beta = 0.3;
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
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);
        
        long micros_reading = micros();
        // directly set the frequency instead of calling begin()
        filter.invSampleFreq = (micros_reading - micros_previous_reading) * 0.000001f;
        micros_previous_reading = micros_reading;
        // Note that gyro and accelerometer have different coordinate system
        filter.updateIMU(gx, gy, -gz, -ax, -ay, az);
        // filter.update(gx, gy, -gz, -ax, -ay, az, -1, 0, 0); // Could also set arbitraty earth pole to always point forward

        // Send accelerometer edited to correspond to our little reordering and fixing of the coordinate system
        // X-axis points forward, Y-axis to the right and Z-axis downward
        ImuAndFingersPackage imuAndFingersPackage;
        imuAndFingersPackage.values.imu[0] = ax*100;
        imuAndFingersPackage.values.imu[1] = ay*100;
        imuAndFingersPackage.values.imu[2] = -az*100;

        // Now send the adjusted rotation data (yaw has now been fixed in library to be +-180 like the rest)
        // X-axis points forward, Y-axis to the right and Z-axis downward
        imuAndFingersPackage.values.imu[3] = -filter.getRoll();
        imuAndFingersPackage.values.imu[4] = -filter.getPitch();
        imuAndFingersPackage.values.imu[5] = filter.getYaw();

        // Now send the fingers yo!
        // Values are between 0 and 200 (the analog readings should be 500-700)
        imuAndFingersPackage.values.fingers[0] = analogRead(A0) - 500;
        imuAndFingersPackage.values.fingers[1] = analogRead(A1) - 500;
        imuAndFingersPackage.values.fingers[2] = analogRead(A2) - 500;
        imuAndFingersPackage.values.fingers[3] = analogRead(A3) - 500;
        imuAndFingersPackage.values.fingers[4] = analogRead(A4) - 500;

        // Also send quaternion data in the adjusted order for those that dare listen
        // THREE JS Coordinate System where X-axis points to the right, Y-axis upward and Z-axis backward
        QuaternionsPackage quaternionsPackage;
        quaternionsPackage.values[0] = filter.q1;
        quaternionsPackage.values[1] = filter.q0;
        quaternionsPackage.values[2] = filter.q2;
        quaternionsPackage.values[3] = filter.q3;

        // Finally send it all out
        characteristicImuAndFingers.writeValue(imuAndFingersPackage.bytes, 17);
        characteristicQuaternions.writeValue(quaternionsPackage.bytes, 16);
      }
    }
    // when the central disconnects, switch the LED off
    digitalWrite(ledPin, LOW);
  }
}
