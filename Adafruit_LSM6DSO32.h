/*!
 *  @file Adafruit_LSM6DSO32.h
 *
 * 	I2C Driver for the Adafruit LSM6DSO32 6-DoF Accelerometer and Gyroscope
 *library
 *
 * 	This is a library for the Adafruit LSM6DSO32 breakout:
 * 	https://www.adafruit.com/products/PID_HERE
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSO32_H
#define _ADAFRUIT_LSM6DSO32_H

#include "Adafruit_LSM6DS.h"
#include "Adafruit_LSM6DSOX.h"
#define LSM6DSO32_CHIP_ID 0x6C ///< LSM6DSO32 default device id from WHOAMI

/** The accelerometer data range */
//The data ranges on the CTRL1_XL register are always numbered 0, 1, 2, 3 for all of the LSM6DS devices
//However in the +/-32G devices these now represent double the value in the other devices.
//Adafruit created a new enum but that can't be converted back and forth, to store the value in the 
//base class's variable. So let's use #define's to just rename those numbers to the numbers we want.
//They are written in this odd order because that's the numerical order in the datasheet. 0b01 is the highest range
//With this change, we don't need to redefine the setAccelRange() function as the base class can handle 0, 1, 2, 3
#define  LSM6DSO32_ACCEL_RANGE_4_G LSM6DS_ACCEL_RANGE_2_G
#define  LSM6DSO32_ACCEL_RANGE_32_G LSM6DS_ACCEL_RANGE_16_G
#define  LSM6DSO32_ACCEL_RANGE_8_G LSM6DS_ACCEL_RANGE_4_G
#define  LSM6DSO32_ACCEL_RANGE_16_G LSM6DS_ACCEL_RANGE_8_G

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DSO32 
 */
class Adafruit_LSM6DSO32 : public Adafruit_LSM6DSOX {
public:
  Adafruit_LSM6DSO32();

  void _read(void);
  void _readFast(void);

private:
  bool _init(int32_t sensor_id);
};

#endif
