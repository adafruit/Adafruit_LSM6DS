// Basic demo for accelerometer readings from Adafruit sox301

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM6DSOX sox;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX test!");
  
  // Try to initialize!
  if (! sox.begin()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) { delay(10); }
  }
  Serial.println("LSM6DSOX Found!");

  //sox.setAccelRange(LSM6DSOX_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
    case LSM6DSOX_ACCEL_RANGE_2_G: Serial.println("+-2G"); break;
    case LSM6DSOX_ACCEL_RANGE_4_G: Serial.println("+-4G"); break;
    case LSM6DSOX_ACCEL_RANGE_8_G: Serial.println("+-8G"); break;
    case LSM6DSOX_ACCEL_RANGE_16_G: Serial.println("+-16G"); break;
  }

  sox.setGyroRange(LSM6DSOX_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
    case LSM6DSOX_GYRO_RANGE_250_DPS: Serial.println("250 degrees/s"); break;
    case LSM6DSOX_GYRO_RANGE_500_DPS: Serial.println("500 degrees/s"); break;
    case LSM6DSOX_GYRO_RANGE_1000_DPS: Serial.println("1000 degrees/s"); break;
    case LSM6DSOX_GYRO_RANGE_2000_DPS: Serial.println("2000 degrees/s"); break;
  }

  //sox.setAccelDataRate(LSM6DSOX_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
    case LSM6DSOX_RATE_SHUTDOWN: Serial.println("0 Hz"); break;
    case LSM6DSOX_RATE_12_5_HZ: Serial.println("12.5 Hz"); break;
    case LSM6DSOX_RATE_26_HZ: Serial.println("26 Hz"); break;
    case LSM6DSOX_RATE_52_HZ: Serial.println("52 Hz"); break;
    case LSM6DSOX_RATE_104_HZ: Serial.println("104 Hz"); break;
    case LSM6DSOX_RATE_208_HZ: Serial.println("208 Hz"); break;
    case LSM6DSOX_RATE_416_HZ: Serial.println("416 Hz"); break;
    case LSM6DSOX_RATE_833_HZ: Serial.println("833 Hz"); break;
    case LSM6DSOX_RATE_1_66K_HZ: Serial.println("1.66 KHz"); break;
    case LSM6DSOX_RATE_3_33K_HZ: Serial.println("3.33 KHz"); break;
    case LSM6DSOX_RATE_6_66K_HZ: Serial.println("6.66 KHz"); break;
    case LSM6DSOX_RATE_1_6_HZ_LP: Serial.println("1.6 Hz"); break;
  }

  //sox.setGyroDataRate(LSM6DSOX_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
    case LSM6DSOX_RATE_SHUTDOWN: Serial.println("0 Hz"); break;
    case LSM6DSOX_RATE_12_5_HZ: Serial.println("12.5 Hz"); break;
    case LSM6DSOX_RATE_26_HZ: Serial.println("26 Hz"); break;
    case LSM6DSOX_RATE_52_HZ: Serial.println("52 Hz"); break;
    case LSM6DSOX_RATE_104_HZ: Serial.println("104 Hz"); break;
    case LSM6DSOX_RATE_208_HZ: Serial.println("208 Hz"); break;
    case LSM6DSOX_RATE_416_HZ: Serial.println("416 Hz"); break;
    case LSM6DSOX_RATE_833_HZ: Serial.println("833 Hz"); break;
    case LSM6DSOX_RATE_1_66K_HZ: Serial.println("1.66 KHz"); break;
    case LSM6DSOX_RATE_3_33K_HZ: Serial.println("3.33 KHz"); break;
    case LSM6DSOX_RATE_6_66K_HZ: Serial.println("6.66 KHz"); break;
  }
}

void loop() {
//  sox.read();      // get X Y and Z data at once
//  // Then print out the raw data
//  Serial.print("X:  "); Serial.print(sox.x); 
//  Serial.print("  \tY:  "); Serial.print(sox.y); 
//  Serial.print("  \tZ:  "); Serial.print(sox.z); 
//  delay(100); 
//  
//  /* Or....get a new sensor event, normalized */ 
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);
    
//    Serial.print("\t\tTemperature "); Serial.print(temp.temperature);
//    Serial.println(" deg C");
//
//    /* Display the results (acceleration is measured in m/s^2) */
//    Serial.print("\t\tAccel X: "); Serial.print(accel.acceleration.x);
//    Serial.print(" \tY: "); Serial.print(accel.acceleration.y);
//    Serial.print(" \tZ: "); Serial.print(accel.acceleration.z);
//    Serial.println(" m/s^2 ");
//    
//    /* Display the results (acceleration is measured in m/s^2) */
//    Serial.print("\t\tGyro X: "); Serial.print(gyro.gyro.x);
//    Serial.print(" \tY: "); Serial.print(gyro.gyro.y);
//    Serial.print(" \tZ: "); Serial.print(gyro.gyro.z);
//    Serial.println(" degrees/s ");
//    Serial.println();
//   
//    delay(100);
//  Serial.print(temp.temperature);
//  
//  Serial.print(","); Serial.print(accel.acceleration.x);
//  Serial.print(","); Serial.print(accel.acceleration.y);
//  Serial.print(","); Serial.print(accel.acceleration.z);
//
//  Serial.print(",");
  Serial.print(gyro.gyro.x);
  Serial.print(","); Serial.print(gyro.gyro.y);
  Serial.print(","); Serial.print(gyro.gyro.z);
  Serial.println();
//  delay(10);

}
