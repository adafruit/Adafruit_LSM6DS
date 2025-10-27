/*********************************************
   LSM6DS Fifo

   Use ISM330/LSM6 Fifo function to compensate for big delays due to other processing
   You can launch long-running processes or sleep and the LSM6 just accumulates data until
   you're ready to read it later.

   To keep the example simple, we're not doing a lot of processing, just delays,
   but this would normally be used for an AHRS algorithm to fuse multiple sensors.

   The LSM6 will be configured to record the gyro and accel at high speed
   Temperature may be recorded at a slow speed
   Fifo is in Continuous mode
   Watermark is set to 1 and that's routed to INT1 pin so any data in the Fifo will assert the interrupt

   Timestamp in Fifo has fixed resolution of 25us or 21.7us depending on the chip (plus or minus FREQ_FINE adjustment)
   The timestamp has 32 bits, so it's only occupying the first two words in the Fifo.
   The last 16 bits of the timestamp contains the Fifo (batch) data rates selected for the sensors plus other flags.
   The exact mapping of these is available in https://www.st.com/resource/en/application_note/an5922-lsm6dsv-6axis-imu-with-embedded-sensor-fusion-i3c-oiseis-for-smart-applications-stmicroelectronics.pdf

   Because the Fifo can contain any type of data, you need to read the tag to understand what the
   data is and how to convert from the raw 16-bit numbers to a useful unit of measure. The
   conversion functions inside the class are unable to help with this, so you need to do your
   own conversions when using the Fifo.

   Circuit: Any ISM330 or LSM6 chip connected to I2C, except LSM6DS3... or LSM6DSL as their Fifo's aren't implemented.
*/
#define myWire Wire1  //for processors like Teensy, there may be more than one I2C bus, select it here

const bool debug = true; //turn this on for detailed display of time taken to do each task

//#include <float16.h> //only include this if you have a LSM6DSV... and want to use the SFLP quaternion

//Uncomment only 1 line below, depending on which chip you have 
//The libraries not listed here have a different kind of fifo that's not currently supported
//#include <Adafruit_LSM6DS.h>
//#include <Adafruit_LSM6DSOX.h>
//#include <Adafruit_LSM6DSO32.h>
//#include <Adafruit_ISM330DHCX.h>
//#include <Adafruit_LSM6DSV.h>
#include <Adafruit_LSM6DSV320.h>
  
const int chunkSize = 10; //maximum Fifo entries to get at one time before beginning processing
int16_t fifoData[chunkSize][4]; //each Fifo entry is 1 tag plus 3 16-bit ints.
int fifoLevel = 0; //number of items currently in the array


//Uncomment only 1 line below, depending on which chip you have 
//Adafruit_LSM6DS myISM;      //handles almost all "basic" variants such as LSM6DSTX, LSM6DSOP and many more
//Adafruit_LSM6DSOX myISM;
//Adafruit_LSM6DSO32 myISM;     //This is one of my favourites as it handles +/-32G
//Adafruit_ISM330DHCX myISM;
//Adafruit_LSM6DSV myISM;     //"V" chips have significantly more capabilities, such as SFLP
//Adafruit_LSM6DSV16B myISM;
//Adafruit_LSM6DSV32 myISM;
//Adafruit_LSM6DSV80 myISM;
Adafruit_LSM6DSV320 myISM;

//Scale for converting raw gyro+accel data into useful units like deg/sec or G's
//Scale will be different depending on which chip you're using and what range you specify in setup
//(see other examples for how to do this.)
float gyroScale = 70.0 / 1000.0; //datasheet sensitivity is specified in milli-degrees-per-second
float accelScale = 0.122 / 1000.0; //datasheet sensitivity is specified in milli-gees

//sensor data
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
uint32_t timestamp = 0;
bool newGyro = false; //raw gyro reading available
bool newAccel = false; //raw accel reading available
float updateRate, deltaTime;

#ifdef _ADAFRUIT_LSM6DSV_H
   //variables specific to the "V" series chips - SFLP and high-G accelerometer
   float16 tempW, tempX, tempY, tempZ;
   float sflpRotationW, sflpRotationX, sflpRotationY, sflpRotationZ;
   bool newQuat0, newQuat1, newQuat;
   float sflpGravityX, sflpGravityY, sflpGravityZ;
   float highGX, highGY, highGZ;
   float highGScale = 10.417 / 1000.0;
   bool newHighG, newGravity;
#endif        

void inhaleFifo() {
  //bring in up to 10 entries at once as fast as possible from the device 
  //Assumes the fifoData[][] array has been cleared [it is not a circular buffer]
  //expecting big delays that might have 20 or more in the fifo but we'll only process a chunk
  //so we can allow the rest of the main loop to have a chance to update its output
  
  //First, how many are in the fifo?
  int countFifo = myISM.getFifoAvailable();
  if (countFifo <= 0) return; //nothing to do

  fifoLevel = 0;  //if there was data in the array not processed then we'll lose it
  int numToGet = min(countFifo, chunkSize); //get the smallest of the actual contents or space in the array

  unsigned long start = micros();
  if(debug) Serial.print(start / 1000.0, 3);
  if(debug) Serial.print(" inhale ");
  if(debug) Serial.print(numToGet);

  while (numToGet > 0) {
    fifoData[fifoLevel][0] = 0; //ensure the tag is zeroed in case no data returned
    myISM.getFifo((uint8_t*)fifoData[fifoLevel] + 1); //fill data into the right 7 bytes of the 8-byte-wide array
    //Serial.print(":"); Serial.print(fifoData[fifoLevel][0], HEX);
    if (0 == fifoData[fifoLevel][0]) break; //special zero tag means Fifo empty, stop reading
    numToGet--;
    fifoLevel++;
    if (fifoLevel >= chunkSize) break; //check we didn't exceed the array size
  }


  if(debug) Serial.print(" dur:");
  if(debug) Serial.print((micros() - start) / 1000.0, 3);
  if(debug) Serial.println();
}

void processAvailData() {
  //feed the fifoData[] in and convert units

  if (fifoLevel > 0) {
    unsigned long start = micros();
    if(debug) Serial.print(start / 1000.0, 3);
    if(debug) Serial.print(" process ");
    if(debug) Serial.print(fifoLevel);

    for (int fifoPtr = 0; fifoPtr < fifoLevel; fifoPtr++) {
      //process one fifo entry
      lsm6_fifo_tag_t tag = myISM.parseFifoTag(fifoData[fifoPtr][0] >> 8);
      if(debug) Serial.print(" 0x");
      if(debug && tag < 0x10) Serial.print("0");
      if(debug) Serial.print((uint8_t)tag, HEX);
    
      switch (tag) {
        
        case LSM6_FIFO_TAG_GYRO:
          if(debug) Serial.print(".gyro");
          newGyro = true;
          gyroX = fifoData[fifoPtr][1] * gyroScale;
          gyroY = fifoData[fifoPtr][2] * gyroScale;
          gyroZ = fifoData[fifoPtr][3] * gyroScale;
          break;
          
        case LSM6_FIFO_TAG_ACCEL:
          if(debug) Serial.print(".accel");
          newAccel = true;
          accelX = fifoData[fifoPtr][1] * accelScale;
          accelY = fifoData[fifoPtr][2] * accelScale;
          accelZ = fifoData[fifoPtr][3] * accelScale;
          break;
          
        case LSM6_FIFO_TAG_TEMP:
          if(debug) Serial.print(".temperature");
          break;

        case LSM6_FIFO_TAG_TIME:
          if(debug) Serial.print(".time ");
          timestamp = fifoData[fifoPtr][1] + ((uint32_t)fifoData[fifoPtr][2] << 16);
          //Multiply this by 25ns or 21.76ns to get actual time of the timestamp
          if(debug) Serial.print(timestamp);
          break;

#ifdef _ADAFRUIT_LSM6DSV_H
        //cases specific to the "V" series chips          
        case LSM6_FIFO_TAG_HIGHG:
          if(debug) Serial.print(".high-g");
          newHighG = true;
          highGX = fifoData[fifoPtr][1] * highGScale;
          highGY = fifoData[fifoPtr][2] * highGScale;
          highGZ = fifoData[fifoPtr][3] * highGScale;
          break;
          
        case LSM6_FIFO_TAG_SFLP_ROTATION:
          //This needs 4 16-bit floats, coming in two at a time, tagged by the first 16 bit (as an integer)
          if(debug) Serial.print(".rotation");
          if(debug) Serial.print(fifoData[fifoPtr][1]);
          switch(fifoData[fifoPtr][1]) {
            case 0:
              newQuat0 = true;
              newQuat1 = false; //if we were out of sync and missed a 0, we don't want to be using old 1 data
#ifdef FLOAT16_LIB_VERSION
              //we have 16-bit float support
              tempW.setBinary(fifoData[fifoPtr][2]);
              tempX.setBinary(fifoData[fifoPtr][3]);
#endif              
              break;
            case 1:
              newQuat1 = true;
#ifdef FLOAT16_LIB_VERSION
              tempY.setBinary(fifoData[fifoPtr][2]);
              tempZ.setBinary(fifoData[fifoPtr][3]);
#endif              
              break;
            default:
              if(debug) Serial.print("=invalid for quaternion");
              break;
          }          
          if(newQuat0 && newQuat1) {
#ifdef FLOAT16_LIB_VERSION
            sflpRotationW = tempW.toFloat();
            sflpRotationX = tempX.toFloat();          
            sflpRotationY = tempY.toFloat();
            sflpRotationZ = tempZ.toFloat();
#endif              
            newQuat = true; //let the wider world know it's ready to go
            newQuat0 = false; //ready for next
            newQuat1 = false;
          }
          break;
          
        case LSM6_FIFO_TAG_SFLP_BIAS:
          if(debug) Serial.print(".gyro bias");
          break;
          
        case LSM6_FIFO_TAG_SFLP_GRAVITY:
          if(debug) Serial.print(".gravity");
          newGravity = true;
          // scale to G - note the SFLP is fixed at the "2G" (0.061mG/bit) scale for all variants, all the time
          sflpGravityX = fifoData[fifoPtr][1] * 0.061 / 1000;
          sflpGravityY = fifoData[fifoPtr][2] * 0.061 / 1000;
          sflpGravityZ = fifoData[fifoPtr][3] * 0.061 / 1000;
          break;
#endif         
 
        default:
          if(debug) Serial.print("~not handled");
      }
    }

    fifoLevel = 0; //we've processed everything in the buffer
    if(debug) Serial.print(" dur:");
    if(debug) Serial.print((micros() - start) / 1000.0, 3);
    if(debug) Serial.println();
  } //else fifoLevel was zero, nothing to do
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1);
  Serial.println("Starting Fifo example");

  //set up the IMU here - change address as required by your addres pins (for chips that have address pins)
  //while (!myISM.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &myWire)) {
  //while (!myISM.begin_I2C(0x6B, &myWire)) { //example alternate address
  while (!myISM.begin_I2C(LSM6DSV_I2CADDR_DEFAULT, &myWire)) {
    Serial.println("Failed to find LSM6 chip");
    delay(1000);
  }
  Serial.println("LSM6 Starting!");
  myWire.setClock(400000);  //run I2C in fast mode (most I2C devices allow this)
  //myWire.setClock(1000000);  //run I2C in fast mode + (if all other devices on the bus can handle it)

  //Here you would normally set up the data rate and range for each sensor
  //Since this example is generic for many chips, we'll leave the chip-specific library defaults
  //After setting ranges you should update accelScale, gyroScale etc.

  myISM.configFifoInt1(true, false, false, false); //threshold interrupt
  myISM.setFifoMode(LSM6_FIFO_MODE_CONTINUOUS);
  myISM.setFifoWatermark(1); //trigger threshold interrupt if there is even 1 item in the Fifo

  
  //Turn on which sensors are going to appear in the Fifo. 
  //Defaults to off, so omit lines below to leave those sensors out of the Fifo
  myISM.setFifoTSDec(LSM6_FIFO_DEC_TS_8);  //decimation rate based on highest rate out of accel or gyro

#ifdef _ADAFRUIT_LSM6DS_H
  //setups specific to the "non-V" series chips
  myISM.setFifoAccelRate(LSM6DS_RATE_104_HZ); //all standard rates allowed here
  myISM.setFifoGyroRate(LSM6DS_RATE_104_HZ);
  myISM.setFifoTempRate(LSM6DS_RATE_12_5_HZ); //record temperature to Fifo, max 52Hz
#endif
 
#ifdef _ADAFRUIT_LSM6DSV_H
  //setups specific to the "V" series chips
  myISM.setFifoAccelRate(LSM6DSV_RATE_120_HZ); //all standard rates allowed here
  myISM.setFifoGyroRate(LSM6DSV_RATE_120_HZ);
  myISM.setFifoTempRate(LSM6DSV_RATE_15_HZ); //record temperature to Fifo, max 60Hz
  myISM.setFifoHighGRate(true); //Only on/off allowed

  //test SFLP output into the Fifo
  myISM.enableSFLP(true);
  myISM.setSFLPDataRate(LSM6DSV_SFLP_RATE_120_HZ);
  myISM.setFifoEmbFunc(true, true, false); //game quaternion, gravity, gyro bias: routed to Fifo
#endif

  updateRate = myISM.accelerationSampleRate();
  deltaTime = 1 / updateRate;

  Serial.print("Started with update rate ");
  Serial.println(updateRate);

}

void loop() {
  inhaleFifo();
  processAvailData(); //feed the fifoData in

  static unsigned long lastOutput = 0;
  const unsigned long outputInterval = 20; //ms = we're running an "output" at 50Hz
  static bool firstRun = true;
  if (millis() - lastOutput >= outputInterval) {
    if (firstRun) lastOutput = outputInterval * (millis() / outputInterval); else lastOutput += outputInterval;
    firstRun = false;

    if(debug) Serial.print(millis());
    if(debug) Serial.print(" output");

    if(newAccel && newGyro) {
      /* basic acceleration output */
      Serial.print(" x:");
      Serial.print(accelX);
      Serial.print(" y:");
      Serial.print(accelY);
      Serial.print(" z:");
      Serial.print(accelZ);
      newAccel = newGyro = false; //tell the processor we've utilized its output
    } else {
      if(debug) Serial.print(" Catch-up missed output interval (no fresh data)");
    }

#ifdef _ADAFRUIT_LSM6DSV_H    
    /* V-series chips with SFLP: see if we got the gravity vector. */
    if(newGravity) {
      Serial.print(" sflpGravX:");
      Serial.print(sflpGravityX);
      Serial.print(" sflpGravY:");
      Serial.print(sflpGravityY);
      Serial.print(" sflpGravZ:");
      Serial.print(sflpGravityZ);
      newGravity = false;
    }
#endif

    Serial.println();
    
    delay(1); //simulate the time-consuming effect of doing other output processing
  }

  //"random" delay - pretend that we've gotten busy doing some other task and let the Fifo accumulate
  static unsigned long lastDelay = millis();
  const unsigned long delayInterval = 500;
  if (millis() - lastDelay > delayInterval) {
    lastDelay = millis();
    if(debug) Serial.print(millis());
    if(debug) Serial.print(" delay ");
    delay(80);
    if(debug) Serial.print(" end:");
    if(debug) Serial.print(millis());
    if(debug) Serial.println();
  }
}
