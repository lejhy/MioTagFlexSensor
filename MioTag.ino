// adds appropriate libraries
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

//defines the two services used
BLEService fingerService("19B10000-E8F2-537E-4F6C-D104768A1214"); 
BLEService IMUService("19B10001-E8F2-537E-4F6C-D104768A1214");

// defines all the characteristics used
BLEIntCharacteristic Pinkie("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Index("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Middle("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Ring("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic Thumb("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic AccX("19B10007-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic AccY("19B10008-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic AccZ("19B10009-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic GyroX("19B10010-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic GyroY("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic GyroZ("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic MagX("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic MagY("19B10014-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic MagZ("19B10015-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// sets up LED pin used to indicate a connection to a central
const int ledPin = LED_BUILTIN; 

void setup() {
  // begins the serial monitor
  Serial.begin(9600);
  
  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);                              

  // begin initialization
  if (!BLE.begin()) {

    while (1);                                                                         
  }

  // starts the IMU and checks to make sure it is successful
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // set advertised device name as MIOTAG
  BLE.setDeviceName("MIOTAG");

  // adds the characteristics to the appropriate services
  fingerService.addCharacteristic(Pinkie);
  fingerService.addCharacteristic(Index);
  fingerService.addCharacteristic(Middle);
  fingerService.addCharacteristic(Ring);
  fingerService.addCharacteristic(Thumb);
  IMUService.addCharacteristic(AccX);
  IMUService.addCharacteristic(AccY);
  IMUService.addCharacteristic(AccZ);
  IMUService.addCharacteristic(GyroX);
  IMUService.addCharacteristic(GyroY);
  IMUService.addCharacteristic(GyroZ);
  IMUService.addCharacteristic(MagX);
  IMUService.addCharacteristic(MagY);
  IMUService.addCharacteristic(MagZ);
  
  // adds the services to the profile
  BLE.addService(fingerService);
  BLE.addService(IMUService);

  // advertises the device after everything has been set up 
  BLE.advertise();

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

      // call the other loops to update the noticeboard
      updateIMU();
      updateFinger();
    }

    // when the central disconnects, switch the LED off
    digitalWrite(ledPin,LOW);
  }
}
                                                                                    
void updateIMU() {

  // sets up floats to take 9 axis values
  float x, y, z, x1, y1, z1, x2, y2, z2;

  // if the IMU is available, read the three types
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    IMU.readGyroscope(x1, y1, z1);
    IMU.readMagneticField(x2, y2, z2);

    // since all values are between -1 and 1, 
    // multiply this by 100 and round to the nearest whole number
    x = round(x*100);
    y = round(y*100);
    z = round(z*100);
    x1 = round(x1*100);
    y1 = round(y1*100);
    z1 = round(z1*100);
    x2 = round(x2*100);
    y2 = round(y2*100);
    z2 = round(z2*100);

    // map the values to a more consistent format
    float xvalue = map(x, -100, 100, 1, 100);
    float yvalue = map(y, -100, 100, 1, 100);
    float zvalue = map(z, -100, 100, 1, 100);
    float xvalue1 = map(x1, -100, 100, 1, 100);
    float yvalue1 = map(y1, -100, 100, 1, 100);
    float zvalue1 = map(z1, -100, 100, 1, 100);
    float xvalue2 = map(x2, -100, 100, 1, 100);
    float yvalue2 = map(y2, -100, 100, 1, 100);
    float zvalue2 = map(z2, -100, 100, 1, 100);

    // write new values to the characteristic to be updated
    AccX.writeValue(xvalue);
    AccY.writeValue(yvalue);
    AccZ.writeValue(zvalue);
    GyroX.writeValue(xvalue1);
    GyroY.writeValue(yvalue1);
    GyroZ.writeValue(zvalue1);
    MagX.writeValue(xvalue2);
    MagY.writeValue(yvalue2);
    MagZ.writeValue(zvalue2);
  }
}

void updateFinger() {

  // only set up for middle finger currently
  // the analogue value from the pin is read, mapped and updated
  int middleFinger = 0;
  middleFinger = analogRead(A0);
  middleFinger = map(middleFinger, 500, 700, 1, 100);
  Middle.writeValue(middleFinger);
  
}
