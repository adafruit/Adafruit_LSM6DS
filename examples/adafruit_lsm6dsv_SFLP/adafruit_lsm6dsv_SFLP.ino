// Test the sensor-fusion-low-power (SFLP) available on the LSM6DSV... chips

#include <Adafruit_LSM6DSV320.h>  //All "V" chips are subsets of this one

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

//depending on which chip you have, uncomment only one line below
Adafruit_LSM6DSV320 dsv;
//Adafruit_LSM6DSV80 dsv;
//Adafruit_LSM6DSV32 dsv;
//Adafruit_LSM6DSV16 dsv;
//Adafruit_LSM6DSV16B dsv;
//Adafruit_LSM6DSV dsv;

//Globals - current acceleration, gravity, linear accel (subtract gravity), quaternion
float accelX, accelY, accelZ;
float gravX, gravY, gravZ;
float x, y, z;
float16 qW, qX, qY, qZ;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("Adafruit LSM6DSV... SFLP test!");

    // while (!dsv.begin_I2C()) {
  while (!dsv.begin_I2C(LSM6DSV_I2CADDR_DEFAULT, &Wire1)) {
    // while (!dsv.begin_SPI(LSM_CS)) {
    // while (!dsv.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSV chip");
    delay(1000);
  }

  Serial.println("LSM6DSV... Found!");

  //For I2C, set the maximum fast-mode clock speed. 
  Wire.setClock(400000);

  //dsv.setAccelRange(LSM6DSV_ACCEL_RANGE_16_G);
  //dsv.setGyroRange(LSM6DSV_GYRO_RANGE_1000_DPS );

  dsv.setAccelDataRate(LSM6DSV_RATE_480_HZ);
  dsv.setGyroDataRate(LSM6DSV_RATE_480_HZ);

  //If you know the offsets for the accelerometer, put them here...
  //dsv.accelOffset(true, 0, 1,2,-3); //enable, weight(scale) option 0 or 1, X,Y,Z
  //Gyro bias will be calculated by the algorithm although it may help to give it a starting value
  //That bias would be entered as a float16, but the library doesn't have that function yet.

  dsv.enableSFLP(true);
  dsv.setSFLPDataRate(LSM6DSV_SFLP_RATE_480_HZ);
  //No need to "initialize" the SFLP. It starts working right away although may need 0.8 seconds to stabilize.
  

  Serial.println("Setup done");
}

void loop() {
  //There's no "available" bit to tell us when SFLP has completed, but we set all the data rates the same
  //so all data should be ready simultaneously
  //There is a bit in EMB_FUNC_EXEC_STATUS which will tell us when "all" embedded functions complete
  //but that isn't exposed by the library right now
  if (dsv.accelerationAvailable()) {
    dsv.readAcceleration(accelX, accelY, accelZ);
    dsv.readSFLPGravity(gravX, gravY, gravZ);    
    x = accelX - gravX;
    y = accelY - gravY;
    z = accelZ - gravZ;
    dsv.readSFLPQuaternion(qW, qX, qY, qZ);
  }


  //Display output infrequently
  const unsigned long displayPeriod = 100; //milliseconds
  static unsigned long lastDisplay = 0;

  if (millis() - lastDisplay >= displayPeriod) {
    lastDisplay = millis();

    /* optional temperature readout:
      Serial.print("\t\tTemperature ");
      Serial.print(temp.temperature);
      Serial.println(" deg C");
    */

    /* Display the linear accel results (acceleration is measured in m/s^2, convert to G's) */
    Serial.print("X:");
    Serial.print(x / SENSORS_GRAVITY_EARTH);
    Serial.print("\tY:");
    Serial.print(y / SENSORS_GRAVITY_EARTH);
    Serial.print("\tZ:");
    Serial.print(z / SENSORS_GRAVITY_EARTH);
    Serial.print(" G");


    /* Display the gravity vector  */
    Serial.print("\t\tGravity X:");
    Serial.print(gravX / SENSORS_GRAVITY_EARTH);
    Serial.print("\tY:");
    Serial.print(gravY / SENSORS_GRAVITY_EARTH);
    Serial.print("\tZ:");
    Serial.print(gravZ / SENSORS_GRAVITY_EARTH);
    Serial.print(" G");

#ifdef FLOAT16_LIB_VERSION
    /* Display the quaternion - just to prove it exists, we're not doing anything with it 
       Requires the float16.h library to be installed and utilized in the Adafruit_LSM6DSV.h*/
    Serial.print("\t\tQuaternion W:");
    Serial.print(qW.toFloat());
    Serial.print("\tX:");
    Serial.print(qX.toFloat());
    Serial.print("\tY:");
    Serial.print(qY.toFloat());
    Serial.print("\tZ:");
    Serial.print(qZ.toFloat());
#endif    

    Serial.println("");
  }//end of display

}
