// Basic demo for both accelerometer readings from Adafruit
// LSM6DSV320 sensor

#include <Adafruit_LSM6DSV320.h>

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

Adafruit_LSM6DSV320 dsv320;

//High-G accel always runs at a high rate, we're interested in finding the max and min
float highGMaxX, highGMaxY, highGMaxZ, highGMinX, highGMinY, highGMinZ;
int countUpdates;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("Adafruit LSM6DSV320 test!");

  while (!dsv320.begin_I2C()) {
    // while (!dsv320.begin_SPI(LSM_CS)) {
    // while (!dsv320.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSV320 chip");
    delay(1000);
  }

  Serial.println("LSM6DSV320 Found!");

  //For I2C, set the maximum fast-mode clock speed. 
  //This example gets all data for all sensors so I2C isn't fast enough for more than 960Hz update rate
  Wire.setClock(400000);

  //dsv320.setAccelRange(LSM6DSV_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (dsv320.getAccelRange()) {
    case LSM6DSV_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case LSM6DSV_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DSV_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DSV_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  dsv320.setHighGRange(LSM6DSV320_ACCEL_RANGE_320_G);
  Serial.print("High-G range set to: ");
  switch (dsv320.getHighGRange()) {
    case LSM6DSV320_ACCEL_RANGE_32_G:
      Serial.println("+-32G");
      break;
    case LSM6DSV320_ACCEL_RANGE_64_G:
      Serial.println("+-64G");
      break;
    case LSM6DSV320_ACCEL_RANGE_128_G:
      Serial.println("+-128G");
      break;
    case LSM6DSV320_ACCEL_RANGE_256_G:
      Serial.println("+-256G");
      break;
    case LSM6DSV320_ACCEL_RANGE_320_G:
      Serial.println("+-320G");
      break;
  }

  // dsv320.setGyroRange(LSM6DSV_GYRO_RANGE_1000_DPS );
  Serial.print("Gyro range set to: ");
  switch (dsv320.getGyroRange()) {
    case LSM6DSV_GYRO_RANGE_RESERVED:
      Serial.println("Reserved/invalid");
      break;
    case LSM6DSV_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DSV_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DSV_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DSV_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case LSM6DSV_GYRO_RANGE_4000_DPS:
      Serial.println("4000 degrees/s");
      break;
  }

  dsv320.setAccelDataRate(LSM6DSV_RATE_480_HZ);
  Serial.print("Accelerometer data rate set to: ");
  Serial.print(dsv320.getAccelDataRate(), BIN);
  Serial.print(" = ");
  Serial.print(dsv320.accelerationSampleRate(), 3);
  Serial.print(" HZ, fine adjust: ");
  Serial.println(dsv320.getInternalFreqFine());

  dsv320.setHighGDataRate(LSM6DSV320_HGRATE_1_92K_HZ);
  Serial.print("High-G accelerometer data rate set to: 0b");
  Serial.print(dsv320.getHighGDataRate(), BIN);
  Serial.print(" = ");
  Serial.print(dsv320.highGSampleRate(), 3);
  Serial.print(" HZ, fine adjust: ");
  Serial.println(dsv320.getInternalFreqFine());

  dsv320.setGyroDataRate(LSM6DSV_RATE_480_HZ);
  Serial.print("Gyro data rate set to: ");
  Serial.print(dsv320.getGyroDataRate(), BIN);
  Serial.print(" = ");
  Serial.print(dsv320.gyroscopeSampleRate(), 3);
  Serial.print(" HZ, fine adjust: ");
  Serial.println(dsv320.getInternalFreqFine());

  Serial.println("Setup done");

  //initialize the max/min
  highGMaxX = highGMaxY = highGMaxZ = -32000;
  highGMinX = highGMinY = highGMinZ = 32000;
  countUpdates = 0;
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t highG;

  if (dsv320.highGAvailable()) {
    countUpdates++;
    //  /* Get a new normalized sensor event */
    dsv320.getEvent(&accel, &gyro, &temp, &highG);

    //extract max/min
    if (highG.acceleration.x > highGMaxX) highGMaxX = highG.acceleration.x;
    if (highG.acceleration.y > highGMaxY) highGMaxY = highG.acceleration.y;
    if (highG.acceleration.z > highGMaxZ) highGMaxZ = highG.acceleration.z;
    if (highG.acceleration.x < highGMinX) highGMinX = highG.acceleration.x;
    if (highG.acceleration.y < highGMinY) highGMinY = highG.acceleration.y;
    if (highG.acceleration.z < highGMinZ) highGMinZ = highG.acceleration.z;
    
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

    /* Display the low-G results (acceleration is measured in m/s^2, convert to G's) 
    Serial.print("X:");
    Serial.print(accel.acceleration.x / SENSORS_GRAVITY_EARTH);
    Serial.print("\tY:");
    Serial.print(accel.acceleration.y / SENSORS_GRAVITY_EARTH);
    Serial.print("\tZ:");
    Serial.print(accel.acceleration.z / SENSORS_GRAVITY_EARTH);
*/

    /* Display the high-G max/min results (acceleration is measured in m/s^2, convert to G's) */
    Serial.print("\tMax_X:");
    Serial.print(highGMaxX / SENSORS_GRAVITY_EARTH);
    Serial.print("\tMax_Y:");
    Serial.print(highGMaxY / SENSORS_GRAVITY_EARTH);
    Serial.print("\tMax_Z:");
    Serial.print(highGMaxZ / SENSORS_GRAVITY_EARTH);
    Serial.print("\tMin_X:");
    Serial.print(highGMinX / SENSORS_GRAVITY_EARTH);
    Serial.print("\tMin_Y:");
    Serial.print(highGMinY / SENSORS_GRAVITY_EARTH);
    Serial.print("\tMin_Z:");
    Serial.print(highGMinZ / SENSORS_GRAVITY_EARTH);
    /*
    Serial.print("\tData Rate ");
    Serial.print((float)countUpdates / displayPeriod * 1000);
    Serial.print("Hz");
    */    

    //Reset the max/min
    highGMaxX = highGMaxY = highGMaxZ = -32000;
    highGMinX = highGMinY = highGMinZ = 32000;
    countUpdates = 0;


    /* Display the gyro results (rotation is measured in rad/s) */
    /*
      Serial.print("\t\tGyro X: ");
      Serial.print(gyro.gyro.x);
      Serial.print(" \tY: ");
      Serial.print(gyro.gyro.y);
      Serial.print(" \tZ: ");
      Serial.print(gyro.gyro.z);
      Serial.println(" radians/s ");
    */

    Serial.println("");
  }//end of display

}
