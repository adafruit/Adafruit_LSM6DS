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

//  //sox.setDataRate(LSM6DSOX_DATARATE_31_25_HZ);
//  Serial.print("Data rate set to: ");
//  switch (sox.getDataRate()) {
//    case LSM6DSOX_DATARATE_1_HZ: Serial.println("1 Hz"); break;
//    case LSM6DSOX_DATARATE_1_95_HZ: Serial.println("1.95 Hz"); break;
//    case LSM6DSOX_DATARATE_3_9_HZ: Serial.println("3.9 Hz"); break;
//    case LSM6DSOX_DATARATE_7_81_HZ: Serial.println("7.81 Hz"); break;
//    case LSM6DSOX_DATARATE_15_63_HZ: Serial.println("15.63 Hz"); break;
//    case LSM6DSOX_DATARATE_31_25_HZ: Serial.println("31.25 Hz"); break;
//    case LSM6DSOX_DATARATE_62_5_HZ: Serial.println("62.5 Hz"); break;
//    case LSM6DSOX_DATARATE_125_HZ: Serial.println("125 Hz"); break;
//    case LSM6DSOX_DATARATE_250_HZ: Serial.println("250 Hz"); break;
//    case LSM6DSOX_DATARATE_500_HZ: Serial.println("500 Hz"); break;
//    case LSM6DSOX_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
//  }
//

//
//  //sox.setRange(LSM6DSOX_RANGE_2_G);
//  Serial.print("Range set to: ");
//  switch (sox.getRange()) {
//    case LSM6DSOX_RANGE_2_G: Serial.println("+-2G"); break;
//    case LSM6DSOX_RANGE_4_G: Serial.println("+-4G"); break;
//    case LSM6DSOX_RANGE_8_G: Serial.println("+-8G"); break;
//    case LSM6DSOX_RANGE_16_G: Serial.println("+-16G"); break;
//  }

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
    sox.getEvent(&accel);
//  
//  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(accel.acceleration.x);
  Serial.print(" \tY: "); Serial.print(accel.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(accel.acceleration.z); 
  Serial.println(" m/s^2 ");

  Serial.println();
// 
  delay(100); 
}



//
//void Adafruit_LSM6DSOX::setAccelDataRate(LSM6DSOX_data_rate_t data_rate){
//    
//    Adafruit_BusIO_Register ctrl1 =
//      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL1_XL); 
//
//    // ctrl1.write(0b10000);
//
//    Adafruit_BusIO_RegisterBits accel_data_rate =
//      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);
//
//    accel_data_rate.write(data_rate);
//    
//    delay(200);
//}