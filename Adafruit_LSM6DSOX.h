/*!
 *  @file Adafruit_LSM6DSOX.h
 *
 * 	I2C Driver for the Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope library
 *
 * 	This is a library for the Adafruit LSM6DSOX breakout:
 * 	https://www.adafruit.com/products/PID_HERE
 * 
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSOX_H
#define _ADAFRUIT_LSM6DSOX_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>


#define LSM6DSOX_I2CADDR_DEFAULT 0x6a ///< LSM6DSOX default i2c address
#define LSM6DSOX_CHIP_ID 0x6C ///< LSM6DSOX default device id from WHOAMI


#define LSM6DSOX_CTRL1_XL           0x10 ///< Acceleration configuration register
#define LSM6DSOX_WHOAMI              0xF ///< Chip ID register

/**
 * @brief Example enum values
 *
 * Allowed values for `setProximityLEDCurrent`.
 */
typedef enum led_current {
  LSM6DSOX_EXAMPLE_50MA,
  LSM6DSOX_EXAMPLE_75MA,
} LSM6DSOX_example_t;


/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DSOX I2C Digital Potentiometer
 */
class Adafruit_LSM6DSOX {
public:

  Adafruit_LSM6DSOX();  
  bool begin(uint8_t i2c_address=LSM6DSOX_I2CADDR_DEFAULT, TwoWire *wire = &Wire);

  LSM6DSOX_example_t getEXAMPLE(void);
  void setEXAMPLE(LSM6DSOX_example_t example_value);

private:
  bool _init(void);

  Adafruit_I2CDevice *i2c_dev;
};

#endif

