/*********************************************
   LSM6DS Sensor Hub

   Use ISM330/LSM6 Sensor Hub to integrate a magnetometer on the subsidiary bus

   To keep the example simple, we're not doing a lot of processing, 
   but this would normally be used for an AHRS algorithm to fuse multiple sensors.

   Tested with LIS3MDL and LIS2MDL. The "3" is no longer available and the "2" isn't
   the best sensor to choose for a compass. It works, but it's not great.

   There's two strategies to do the initial configuration on the magnetometer:
   1. Passthrough mode - ask the LSM6 to join the two I2C buses and just run the normal
      .begin() function provided by the sensor library.
   2. Use the .sHubDataWrite(uint8_t data) to write a single byte of data to the target sensor.
      This can only write to the sensor in the "0" slot so if you have to configure more than one
      you need to do them one-at-a-time with .sHubConfigWriteOnce() .sHubSensor(0, addr, regAddr) 
      This is going to be relatively complex with only sending 1 byte at a time, so writing to a
      single configuration register will take many round-trips to the LSM6.
   *This example only shows the passthrough method*

   The theory of operation of the sensor hub is you give it a sensor I2C address, the register
   address you wish to read and how many bytes to read. The results are stored in the 18 sensor hub
   registers on the LSM6. The results may also be directed to the Fifo, which helps when you want to
   leave the LSM6 running unattended for a while (delays doing other processing) and pick up 
   all the sensor readings later, with the order of the readings preserved.

   The sensor hub read operation can be triggered from a transition on external interrupt pin INT2 or it may be
   triggered by the accel/gyro completion flag. A few of the LSM6 chips will apply a decimation
   to that (sensor hub every 1, 2, 4 or 8 triggers) but most specify a data rate. 
   These data rates are the lowest four or seven data rates for your LSM6. The data rate is common across
   all 4 sensors in the hub. 

   90% of the time you're probably reading three XYZ values from the hubbed sensor, so 6 bytes per sensor.
   But you may need to read 1 byte or some other number. This example will explore where the results
   get placed by requesting temperature from the magnetometer (2 bytes) as sensor 0 then 6 bytes of results
   as sensor 1. (Yes, you could use one read to do this but the point is to show what happens with 2 sensors.)

   Circuit: Any ISM330 or LSM6 chip connected to I2C, except LSM6DOP and LSM6DS33 which don't have sensor hubs.
   Connect an LISM2MDL or LISM3MDL to the SDx and SCx pins. OPTIONAL: DRDY connected to INT2
   Pullup resistors are optional on SDx and SCx - turn on pullups in code below.
   This is the "Mode 2" connection shown in the LSM6... datasheet.
*/
#define myWire Wire  //for processors like Teensy, there may be more than one I2C bus, select it here

const bool debug = false; //turn this on for detailed display of time taken to do each task

//Uncomment only 1 line below, depending on which chip you have 
//#include <Adafruit_LSM6DS.h>
//#include <Adafruit_LSM6DS3.h>
//#include <Adafruit_LSM6DS3TRC.h>
//#include <Adafruit_LSM6DSL.h>
//#include <Adafruit_LSM6DSOX.h>
//#include <Adafruit_LSM6DSO32.h>
//#include <Adafruit_ISM330DHCX.h>
//#include <Adafruit_LSM6DSV.h>
#include <Adafruit_LSM6DSV320.h>
  
//Uncomment only 1 line below, depending on which chip you have 
//Adafruit_LSM6DS myISM;      //handles almost all "basic" variants such as LSM6DSTX and many more
//Adafruit_LSM6DS3 myISM;
//Adafruit_LSM6DS3TRC myISM;
//Adafruit_LSM6DSL myISM;     //also LSM6DSM (not tested)
//Adafruit_LSM6DSOX myISM;
//Adafruit_LSM6DSO32 myISM;     //This is one of my favourites as it handles +/-32G
//Adafruit_ISM330DHCX myISM;
//Adafruit_LSM6DSV myISM;     //"V" chips have significantly more capabilities, such as SFLP
//Adafruit_LSM6DSV16B myISM;
//Adafruit_LSM6DSV32 myISM;
//Adafruit_LSM6DSV80 myISM;
Adafruit_LSM6DSV320 myISM;

//Your magnetometer library
//#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LIS2MDL.h>

#ifdef LIS3MDL_I2CADDR_DEFAULT    //Detect which type of magnetometer you have #included
#define _ADDRESS_MAG LIS3MDL_I2CADDR_DEFAULT
#define MAG_REG_OUT_X_L 0x28      ///< Register address for X axis lower byte
#define MAG_REG_TEMP_OUT_L 0x2E   ///< Register address for temperature lower byte
Adafruit_LIS3MDL mag;
#else
#define MAG_REG_OUT_X_L 0x68      ///< Register address for X axis lower byte
#define MAG_REG_TEMP_OUT_L 0x6E   ///< Register address for temperature lower byte
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
#endif

//sensor data - raw 16-bit values
int16_t gyroX, gyroY, gyroZ;
int16_t accelX, accelY, accelZ;
int16_t magX, magY, magZ; 
int16_t magTemperature;


void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1);
  Serial.println("Starting Sensor Hub example");

  //set up the IMU here - change address as required by your addres pins (for chips that have address pins)
  //while (!myISM.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &myWire)) {
  //while (!myISM.begin_I2C(0x6B, &myWire)) { //example alternate address
  while (!myISM.begin_I2C(LSM6DSV_I2CADDR_DEFAULT, &myWire)) {
    Serial.println("Failed to find LSM6 chip");
    delay(1000);
  }
  Serial.println("LSM6 Starting!");

  //Here you would normally set up the data rate and range for the gyro and accelerometer
  //Since this example is generic for many chips, we'll leave the chip-specific library defaults

  //Set passthrough mode so the magnetometer library can talk and do the configuration setup
  myISM.sHubConfigPassThrough(true);
  //If you have no pullup resistors for the sensor hub I2C bus, set pullup to true below
  myISM.sHubConfigPullUp(false);

  /* Initialise the magnetometer */
  while(!mag.begin_I2C(_ADDRESS_MAG, &myWire)) {
    Serial.println("Ooops, no magnetometer detected ... Check your wiring!");
    delay(1000);
  }  

  //The mag.begin() should have set up the magnetometer in continuous-running mode
  //This is the default for the two chips tested when writing this example
  //The standard Adafruit library for the LIS3MDL doesn't provide a method
  //to turn on the temperature sensor, so you will always get zero for this.

  //In LIS2MDL, the data-ready pin is not enabled by default
#ifdef LIS2MDL_MAG_H
  mag.enableDataReady(true);
#endif

  //To show the magnetometer is working, you can use its library functions
  //to get data from it here, while still in passthrough mode

  //leave passthrough mode
  myISM.sHubConfigPassThrough(false);


  //Now set up the two sensor hub sensors...
  //For LSM6 chips in the "DSL group", they have a slightly different interpretation of the data-rate field  
  //Rate is a decimation factor, dividing the trigger count by 2^n 
  //Can have different decimation rates for each sensor.
  //For "V group" chips, rate 0 is the slowest rate (1.875Hz) but for all others it is the fastest rate (104Hz)
  myISM.sHubSensor(0, _ADDRESS_MAG, true, MAG_REG_TEMP_OUT_L, 2, false, 1); 
  myISM.sHubSensor(1, _ADDRESS_MAG, true, MAG_REG_OUT_X_L, 6, false, 1);   

  //Tell it how many sensors we want to read (1-4)
  myISM.sHubConfigCountSensors(2);

  //INT2 will switch to an input later, when both MASTER_ON and START_CONFIG are 1. 
  //Otherwise it's pulled up high all the time, unless you make it an open drain.
  myISM.configIntOpenDrain(true);
  //Set the polarity of the INT2 input: only required on "V" chips - others will ignore this as they always think INT2 input is active high. 
  myISM.configInt2InHigh(true);
  //Select the start signal for the sensor hub to be the data rate you selected for sensor0
  myISM.sHubConfigStartConfig(false); 
  
  //Allow the sensor hub to act as a master for its I2C bus (turn this off and wait 0.3ms before using passthrough again)
  myISM.sHubConfigMasterOn(true);

  /************************************************************************/
  /*  If you have DRDY of the magnetometer connected to INT2 then you can perform the steps below to 
   *  activate reading based on that input instead of the data rate selected.
   *  [Has been tested with LIS3MDL at 155Hz]
   *  
   *  The reason for these steps to occur after turning the master "on" is that 
   *  the INT2 pin only detects transitions from low to high (high to low is possible for "V" chips.)
   *  By the time the code reaches here, the magnetometer has been configured in
   *  continuous mode for long enough that DRDY is already high. The INT2 trigger
   *  won't be triggered until we read one reading from the magnetometer so 
   *  that it will set DRDY low.
   *  
   *  While INT2 is low, we can turn on the StartConfig option to read the 
   *  next transition to high. Then it will continue running by itself.
   */  
  //while(!myISM.sHubConcluded()) yield();//Wait for the sensor hub to perform one automated read, so the DRDY on the magnetometer will go low
  //myISM.sHubConfigStartConfig(true); //turn this on so that INT2 will see the next transition from low to high
  /************************************************************************/
  
  myWire.setClock(1000000);  //run I2C in fast mode plus (if all other devices on the bus can handle it)
  //Note the LIS_MDL magnetometers are examples of devices which can't operate in fast mode plus
  //so this is one way to speed up by letting the LSM6 do the slow work.
  //The LSM6 datasheets don't specify what speed they use on their sensor hub bus. 
  //Tested an LSM6DSO32 and found about 110KHz on the SCX clock pin.
  //Tested an LSM6DSV320 and found about 245KHz on the SCX clock pin.
}

void loop() {
  static unsigned long lastOutput = millis();
  const unsigned long outputInterval = 20; //ms = we're running an "output" at 50Hz
  if (millis() - lastOutput >= outputInterval) {
    
    //Don't try to read if the LSM6 is in the middle of a read operation itself
    if(myISM.sHubConcluded()) {
      lastOutput += outputInterval;
      
      if(debug) Serial.print(millis());
      if(debug) Serial.print(" output ");

      //expecting 8 bytes holding 4 16-bit numbers, sensor hub can hold a maximum of 18 bytes
      uint8_t data[18];
      if(myISM.sHubData(data, 8)) {
        //Expect to always get 0 from thermometer as it isn't enabled
        int16_t thermometer = (int16_t)data[1] << 8 + data[0];
        float tempC = (float)thermometer / 8.0;
        Serial.print(" temp(C):");
        Serial.print(tempC);

        int16_t magX = (int16_t)data[3] << 8 | data[2];
        int16_t magY = (int16_t)data[5] << 8 | data[4];
        int16_t magZ = (int16_t)data[7] << 8 | data[6];
        Serial.print(" magX:");
        Serial.print(magX);
        Serial.print(" magY:");
        Serial.print(magY);
        Serial.print(" magZ:");
        Serial.print(magZ);
        
      } else {
        //read failed - ignore
      }

    Serial.println();
    } //else wasn't ready to read sensor hub, try again immediately on next loop

  } //else wasn't time to output
}
