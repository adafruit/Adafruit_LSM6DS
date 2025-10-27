/*!
 *  @file Adafruit_LSM6DSL.h
 *
 * 	I2C Driver for the Adafruit LSM6DSL 6-DoF Accelerometer and Gyroscope
 *library (may also work for LSM6DSM)
 *
 * 	This is a library for the Adafruit LSM6DSL breakout:
 * 	https://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSL_H
#define _ADAFRUIT_LSM6DSL_H

#include "Adafruit_LSM6DS.h"

#define LSM6DSL_CHIP_ID 0x6A ///< LSM6DSL default device id from WHOAMI

#define LSM6DSL_SENSOR_HUB_1   0x2E //sensor hub first data byte (in the MAIN REGISTER BANK)
#define LSM6DSL_SENSOR_HUB_13  0x4D //there's a gap after 12, 13 starts at this address, goes up to 18
#define LSM6DSL_MASTER_CONFIG  0x1A //sensor hub I2C master
#define LSM6DSL_SLV0_ADD       0x02 //sensor hub slave chip address (in the embedded functions register bank A)
#define LSM6DSL_SLV0_SUBADD    0x03 //sensor hub slave register address
#define LSM6DSL_SLV0_CONFIG    0x04 //sensor hub slave config
#define LSM6DSL_SLV1_ADD       0x05 //sensor hub slave chip address
#define LSM6DSL_SLV1_SUBADD    0x06 //sensor hub slave register address
#define LSM6DSL_SLV1_CONFIG    0x07 //sensor hub slave config
#define LSM6DSL_SLV2_ADD       0x08 //sensor hub slave chip address
#define LSM6DSL_SLV2_SUBADD    0x09 //sensor hub slave register address
#define LSM6DSL_SLV2_CONFIG    0x0A //sensor hub slave config
#define LSM6DSL_SLV3_ADD       0x0B //sensor hub slave chip address
#define LSM6DSL_SLV3_SUBADD    0x0C //sensor hub slave register address
#define LSM6DSL_SLV3_CONFIG    0x0D //sensor hub slave config
#define LSM6DSL_DATAWRITE_SLV0 0x0E //sensor hub data byte to write to slave 0
#define LSM6DSL_FUNC_SRC1      0x53 //sensor hub end-op plus other flags
#define LSM6DSL_FUNC_SRC2      0x54 //sensor hub NAK status

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DSL (may also work for LSM6DSM)
 */
class Adafruit_LSM6DSL : public Adafruit_LSM6DS {
public:
  Adafruit_LSM6DSL();
  ~Adafruit_LSM6DSL(){};

  void enableI2CMasterPullups(bool enable_pullups);

  //Sensor hub works a little differently on this chip versus the others
  bool sHubConcluded(); //func_src1
  bool sHubWriteOnceDone() {return true;}; //not found in the datasheet
  uint8_t sHubSlaveNAK(); //func_src2
  void sHubConfigResetMaster(bool resetMaster = false) {}; //not found in the datasheet 
  void sHubConfigWriteOnce(bool writeOnce = false); //write-once for sensor0 is in the slave1_config byte
  void sHubConfigStartConfig(bool startConfig = false);  //true for INT2 pin, false for accel/gyro trigger
  void sHubConfigPassThrough(bool passThrough = false);
  void sHubConfigPullUp(bool pullUp = false);
  void sHubConfigMasterOn(bool masterOn = false);
  void sHubConfigCountSensors(int countSensors = 0); //in slave0_config
  int sHubGetCountSensors(); 
  void sHubSensor(int sensorNumber, uint8_t addr, bool readBit, uint8_t subAddr, bool numReadOperations, bool enableFifo = false, uint8_t sHubODR = 0);
  //sensor number 0-3. Output data rate (ODR) is a decimator 1/2/4/8. enableFifo is ignored
  void sHubDataWrite(uint8_t data); //single byte of data to write to sensor 0 (usually used for write-once to send one byte to the sensor)
  bool sHubData(uint8_t data[], int numBytes); //read bytes from the sensor hub registers (max 18) Special chips jump address from 12 to 13

private:
  bool _init(int32_t sensor_id);

  //Fifo works completely differently in this chip compared to the others in the LSM6DS family
  //I don't have one to test with so I will de-implement the Fifo by making all related functions private
  using Adafruit_LSM6DS::parseFifoTag;
  using Adafruit_LSM6DS::parseFifoTimeSlot;
  using Adafruit_LSM6DS::getFifoStatus;
  using Adafruit_LSM6DS::getFifoAvailable;
  using Adafruit_LSM6DS::getFifo;
  using Adafruit_LSM6DS::configFifoInt1;
  using Adafruit_LSM6DS::configFifoInt2;
  using Adafruit_LSM6DS::setFifoAccelRate;
  using Adafruit_LSM6DS::setFifoGyroRate;
  using Adafruit_LSM6DS::setFifoTempRate;
  using Adafruit_LSM6DS::setFifoTSDec;
  using Adafruit_LSM6DS::setFifoMode;
  using Adafruit_LSM6DS::setFifoWatermark;
  using Adafruit_LSM6DS::setFifoCounterBDR;
  using Adafruit_LSM6DS::setFifoEmbFunc;

};

#endif
