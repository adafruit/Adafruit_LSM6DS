/*********************************************
   LSM6DS Fifo used with Sensor Hub

   Use ISM330/LSM6 Fifo function to compensate for big delays due to other processing
   You can launch long-running processes or sleep and the LSM6 just accumulates data until
   you're ready to read it later.

   By using the sensor hub, other sensors can be recorded into the Fifo so that they
   come out in the correct order that they were actually read.

   To keep the example simple, we're not doing a lot of processing, just delays,
   but this would normally be used for an AHRS algorithm with a magnetometer compass.

   The LSM6 will be configured to record the gyro and accel at high speed
   Fifo is in Continuous mode
   Watermark is set to 1 and that's routed to INT1 pin so any data in the Fifo will assert the interrupt
   Sensor hub will record at its own rate specified for sensor zero.
   See the sensor hub example to use INT2 to trigger sensor hub readings.

   Because the Fifo can contain any type of data, you need to read the tag to understand what the
   data is and how to convert from the raw 16-bit numbers to a useful unit of measure. The
   conversion functions inside the class are unable to help with this, so you need to do your
   own conversions when using the Fifo.

   The datasheets for the individual sensors don't say, but ST application note AN5192 page 103
   says that reading the Fifo registers has a "wraparound" so that you can read continuously
   from the Fifo registers without having to re-start the read operation every 7 bytes.
   AN5922 page 113 calls this feature "rounding".

   The wraparound is optional here as it may not be appropriate for your use of the Fifo
   and may not work on every LSM6 chip, since it's basically an undocumented feature.
   Wraparound could be used to read the Fifo without even asking how many entries it has,
   as it can read until it sees the special zero tag. This is not implemented here.

   The I2C functions which sit at the bottom of the stack have a limit of (usually) 32 bytes per 
   read operation. While the middle functions are capable of chaining together multiple 
   reads with the I2C repeated-start mechanism, my tests on a Teensy 4.0 always lost
   one byte after the first 32. It seems like endTransmission(false) sends a clock pulse 
   when it should not send anything. So, in the interest of reliability, I'll keep my
   wraparound reads smaller than BUFFER_LENGTH defined in Wire.h, which is 32 bytes for 
   most Arduino variants.

   Circuit: Any ISM330 or LSM6 chip connected to I2C, except LSM6DS3... or LSM6DSL as their Fifo's aren't implemented.
            except LSM6DOP and LSM6DS33 which don't have sensor hubs.
            LIS3MDL or LIS2MDL magnetometer connected to the secondary I2C bus on the LSM6.
            Pullup resistors are optional on second I2C - turn on pullups in code below.
*/
#define myWire Wire  //for processors like Teensy, there may be more than one I2C bus, select it here
//#define useFifoWraparound //Optional, not sure if it works on all chips as it's not documented in any datasheet

const bool debug = true; //turn this on for detailed display of time taken to do each task

//Uncomment only 1 line below, depending on which chip you have
//The libraries not listed here have a different kind of fifo that's not currently supported
//#include <Adafruit_LSM6DS.h>
//#include <Adafruit_LSM6DSOX.h>
//#include <Adafruit_LSM6DSO32.h>
//#include <Adafruit_ISM330DHCX.h>
//#include <Adafruit_LSM6DSV.h>
#include <Adafruit_LSM6DSV320.h>

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


//Your magnetometer library
//#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LIS2MDL.h>

#ifdef LIS3MDL_I2CADDR_DEFAULT
#define _ADDRESS_MAG LIS3MDL_I2CADDR_DEFAULT
#define MAG_REG_OUT_X_L 0x28      ///< Register address for X axis lower byte
Adafruit_LIS3MDL mag;
float magScale = 1.0 / 6842; //LIS3MDL 4 gauss scale
#else
#define MAG_REG_OUT_X_L 0x68      ///< Register address for X axis lower byte
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
float magScale = 1.5 / 1000.0; //LIS2MDL fixed scale
#endif

const int fifoWidth = 7; //each Fifo entry is 1 tag plus 3 16-bit ints = 7 bytes
#ifdef useFifoWraparound
//maximum Fifo entries to get at one time before beginning processing
//based on the low-level I2C buffer which is usually only 32 bytes
const int chunkSize = 32 / fifoWidth; 
#else 
const int chunkSize = 20; //not limited by low-level I2C. You may prefer non-wraparound because of the bigger chunks.
#endif                                      
uint8_t fifoData[chunkSize * fifoWidth]; 
int fifoLevel = 0; //number of items currently in the fifoData array

//Scale for converting raw data into useful units like deg/sec or G's
//Scale will be different depending on which chip you're using and what range you specify in setup
//(see other examples for how to do this.)
float gyroScale = 70.0 / 1000.0; //datasheet sensitivity is specified in milli-degrees-per-second
float accelScale = 0.122 / 1000.0; //datasheet sensitivity is specified in milli-gees

//sensor data
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float magX, magY, magZ;
bool newGyro = false; //raw gyro reading available
bool newAccel = false; //raw accel reading available
bool newMag = false; //raw magnetometer reading available
float updateRate, deltaTime;


void inhaleFifo() {
  //bring in up to 10 entries at once as fast as possible from the device
  //Assumes the fifoData[] array has been cleared [it is not a circular buffer]
  //expecting big delays that might have 20 or more in the fifo but we'll only process a chunk
  //so we can allow the rest of the main loop to have a chance to update its output

  //Without wraparound, using 1MHz fast mode plus, it takes 100us to read from one Fifo entry
  //If reading 4 entries, it takes 394us.
  //With wraparound, 4 entries takes 297us.
  //I'd really like to be reading 20 entries at once but the I2C functions are skipping data somehouw.

  //First, how many are in the fifo?
  int countFifo = myISM.getFifoAvailable();

  fifoLevel = 0;  //if there was data in the array not processed then we'll lose it

  if (countFifo <= 0) return; //nothing to do, 

  int numToGet = min(countFifo, chunkSize); //get the smallest of the actual contents or space in the array

  unsigned long start = micros();
  if (debug) Serial.print(start / 1000.0, 3);
  if (debug) Serial.print(" inhale ");
  if (debug) Serial.print(numToGet);

#ifdef useFifoWraparound

  //Wraparound function allows us to get kilobytes of data with a single (blocking) read operation  
  //However the Teensy 4 seems to skip byte 33, so we just grab smaller chunks than 32
  fifoLevel = myISM.getFifo(fifoData, numToGet);

#else

  //"conventional" reading of the 7 Fifo registers, requesting the tag register address each time
  while (numToGet > 0) {
    fifoData[fifoLevel * fifoWidth] = 0; //ensure the tag is zeroed in case no data returned
    myISM.getFifo(&fifoData[fifoLevel * fifoWidth]);
    if (0 == fifoData[fifoLevel * fifoWidth]) break; //special zero tag means Fifo empty, stop reading
    numToGet--;
    fifoLevel++;
    if (fifoLevel >= chunkSize) break; //check we didn't exceed the array size
  }
  
#endif

  if (debug) Serial.print(" dur:");
  if (debug) Serial.print((micros() - start) / 1000.0, 3);
  if (debug) Serial.println();
}

void processAvailData() {
  //feed the fifoData[] in and convert units

  if (fifoLevel > 0) {
    unsigned long start = micros();
    if (debug) Serial.print(start / 1000.0, 3);
    if (debug) Serial.print(" process ");
    if (debug) Serial.print(fifoLevel);

    for (int fifoPtr = 0; fifoPtr < fifoLevel; fifoPtr++) {
      //process one fifo entry
      lsm6_fifo_tag_t tag = myISM.parseFifoTag(fifoData[fifoPtr * fifoWidth]);
      if (debug) Serial.print(" 0x");
      if (debug && tag < 0x10) Serial.print("0");
      if (debug) Serial.print((uint8_t)tag, HEX);
      int16_t rawX, rawY, rawZ;

      switch (tag) {

        case LSM6_FIFO_TAG_GYRO:
          if (debug) Serial.print(".gyro");
          newGyro = true;
          rawX = fifoData[fifoPtr * fifoWidth + 2] << 8 | fifoData[fifoPtr * fifoWidth + 1];
          rawY = fifoData[fifoPtr * fifoWidth + 4] << 8 | fifoData[fifoPtr * fifoWidth + 3];
          rawZ = fifoData[fifoPtr * fifoWidth + 6] << 8 | fifoData[fifoPtr * fifoWidth + 5];
          gyroX = rawX * gyroScale;
          gyroY = rawY * gyroScale;
          gyroZ = rawZ * gyroScale;
          break;

        case LSM6_FIFO_TAG_ACCEL:
          if (debug) Serial.print(".accel");
          newAccel = true;
          rawX = fifoData[fifoPtr * fifoWidth + 2] << 8 | fifoData[fifoPtr * fifoWidth + 1];
          rawY = fifoData[fifoPtr * fifoWidth + 4] << 8 | fifoData[fifoPtr * fifoWidth + 3];
          rawZ = fifoData[fifoPtr * fifoWidth + 6] << 8 | fifoData[fifoPtr * fifoWidth + 5];
          accelX = rawX * accelScale;
          accelY = rawY * accelScale;
          accelZ = rawZ * accelScale;
          break;

        case LSM6_FIFO_TAG_SENSOR_0:
          if (debug) Serial.print(".SensorHub0");
          newMag = true;
          rawX = fifoData[fifoPtr * fifoWidth + 2] << 8 | fifoData[fifoPtr * fifoWidth + 1];
          rawY = fifoData[fifoPtr * fifoWidth + 4] << 8 | fifoData[fifoPtr * fifoWidth + 3];
          rawZ = fifoData[fifoPtr * fifoWidth + 6] << 8 | fifoData[fifoPtr * fifoWidth + 5];
          magX = rawX * magScale;
          magY = rawY * magScale;
          magZ = rawZ * magScale;
          break;

        case LSM6_FIFO_TAG_SENSOR_NACK:
          if (debug) Serial.print(".SensorNACK");
          if (debug) Serial.print(fifoData[fifoPtr * fifoWidth + 1]);
          break;

        case 0x1D: //LSM6_FIFO_TAG_HIGHG defined only for "V" chips which have a high-G accelerometer
          if (debug) Serial.print(".HighG");         
          break;

        default:
          if (debug) Serial.print("~not handled");
      }
    }

    fifoLevel = 0; //we've processed everything in the buffer
    if (debug) Serial.print(" dur:");
    if (debug) Serial.print((micros() - start) / 1000.0, 3);
    if (debug) Serial.println();
  } //else fifoLevel was zero, nothing to do
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(1);
  Serial.println("Starting Fifo + Sensor Hub example");

  //set up the IMU here - change address as required by your addres pins (for chips that have address pins)
  //while (!myISM.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &myWire)) {
  //while (!myISM.begin_I2C(0x6B, &myWire)) { //example alternate address
  while (!myISM.begin_I2C(LSM6DSV_I2CADDR_DEFAULT, &myWire)) {
    Serial.println("Failed to find LSM6 chip");
    delay(1000);
  }
  Serial.println("LSM6 Starting!");

  //Here you would normally set up the data rate and range for each sensor
  //Since this example is generic for many chips, we'll leave the chip-specific library defaults
  //After setting ranges you should update accelScale, gyroScale etc.

  myISM.configFifoInt1(true, false, false, false); //threshold interrupt
  myISM.setFifoMode(LSM6_FIFO_MODE_CONTINUOUS);
  myISM.setFifoWatermark(1); //trigger threshold interrupt if there is even 1 item in the Fifo

  //Turn on which sensors are going to appear in the Fifo.
  //Defaults to off, so omit lines below to leave those sensors out of the Fifo
  //Sensor hub is congifured separately, for each of its sensors.

#ifdef _ADAFRUIT_LSM6DS_H
  //setups specific to the "non-V" series chips
  myISM.setFifoAccelRate(LSM6DS_RATE_104_HZ); //all standard rates allowed here
  myISM.setFifoGyroRate(LSM6DS_RATE_104_HZ);
#endif

#ifdef _ADAFRUIT_LSM6DSV_H
  //setups specific to the "V" series chips
  myISM.setFifoAccelRate(LSM6DSV_RATE_120_HZ); //all standard rates allowed here
  myISM.setFifoGyroRate(LSM6DSV_RATE_120_HZ);
  myISM.setFifoHighGRate(true); //Only on/off allowed; high-G rate doesn't go slower than 480Hz
#endif

  updateRate = myISM.accelerationSampleRate();
  deltaTime = 1 / updateRate;


  //Now set up the sensor hub...
  //If you have no pullup resistors for the sensor hub I2C bus, set pullup to true below
  myISM.sHubConfigPullUp(false);

  //Set passthrough mode so the magnetometer library can talk and do the configuration setup
  myISM.sHubConfigPassThrough(true);

  /* Initialise the magnetometer */
  while (!mag.begin_I2C(_ADDRESS_MAG, &myWire)) {
    Serial.println("Ooops, no magnetometer detected ... Check your wiring!");
    delay(1000);
  }

  //The mag.begin() should have set up the magnetometer in continuous-running mode
  //This is the default for the two chips tested when writing this example

  Serial.println("Magnetometer configured");
  //leave passthrough mode
  myISM.sHubConfigPassThrough(false);


  //Now set up the sensor hub sensor...
  //For LSM6 chips in the "DSL group", they have a slightly different interpretation of the data-rate field
  //Rate is a decimation factor, dividing the trigger count by 2^n
  //Can have different decimation rates for each sensor.
  //For "V group" chips, rate 0 is the slowest rate (1.875Hz) but for all others it is the fastest rate (104Hz)
  //Pick rate=1 for something in the middle.
  myISM.sHubSensor(0, _ADDRESS_MAG, true, MAG_REG_OUT_X_L, 6, true, 1);

  //Tell it how many sensors we want to read (1-4)
  myISM.sHubConfigCountSensors(1);

  //Allow the sensor hub to act as a master for its I2C bus
  myISM.sHubConfigMasterOn(true);

  //If you want to use the INT2 trigger for the sensor hub, see the dedicated sensor hub example

  myWire.setClock(1000000);  //run I2C in fast mode plus (if all other devices on the bus can handle it)

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
    bool newLine = false;

    if (debug) Serial.print(millis());
    if (debug) Serial.print(" output");

    if (newAccel && newGyro) {
      /* basic acceleration output */
      Serial.print(" x:");
      Serial.print(accelX);
      Serial.print(" y:");
      Serial.print(accelY);
      Serial.print(" z:");
      Serial.print(accelZ);
      newAccel = newGyro = false; //tell the processor we've utilized its output
      newLine = true;
    } else {
      if (debug) {
        Serial.println(" Catch-up missed output interval (no fresh data)");
        newLine = true;
      }
    }

    //Did we get a new magnetometer reading?
    if (newMag) {
      Serial.print(" magX:");
      Serial.print(magX);
      Serial.print(" magY:");
      Serial.print(magY);
      Serial.print(" magZ:");
      Serial.print(magZ);
      newMag = false;
      newLine = true;
    }


    if (newLine) Serial.println();

    delay(1); //simulate the time-consuming effect of doing other output processing
  }

  //"random" delay - pretend that we've gotten busy doing some other task and let the Fifo accumulate
  static unsigned long lastDelay = millis();
  const unsigned long delayInterval = 500;
  if (millis() - lastDelay > delayInterval) {
    lastDelay = millis();
    if (debug) Serial.print(millis());
    if (debug) Serial.print(" DELAY ");
    delay(80);
    if (debug) Serial.print(" end:");
    if (debug) Serial.print(millis());
    if (debug) Serial.println();
  }
}
