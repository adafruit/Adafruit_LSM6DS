
/*!
 *  @file Adafruit_LSM6DSOX.cpp
 *
 *  @mainpage Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope library
 * 
 * 	This is a library for the Adafruit LSM6DSOX breakout:
 * 	https://www.adafruit.com/product/PID_HERE
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *  
 *  This library depends on the Adafruit Unified Sensor library

 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 * 
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_LSM6DSOX.h"

/*!
 *    @brief  Instantiates a new LSM6DSOX class
 */
Adafruit_LSM6DSOX::Adafruit_LSM6DSOX(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_LSM6DSOX::begin(uint8_t i2c_address, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init();
}

boolean Adafruit_LSM6DSOX::_init(void) {
  Adafruit_BusIO_Register chip_id = 
    Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_WHOAMI, 2);

  // make sure we're talking to the right chip
  if (chip_id.read() != LSM6DSOX_CHIP_ID) {
    return false;
  }
/*
        self._bdu = True
        sleep(0.010)
        self._data_rate = 3
        sleep(0.010)
        self._gyro_data_rate = 3
        sleep(0.10)
        self._cached_accel_range = AccelRange.RANGE_4G
        self._gyro_range = GyroRange.RANGE_250_DPS                                                    #set range also
        sleep(0.010)
        self._cached_gyro_range = GyroRange.RANGE_250_DPS
        self._gyro_range = GyroRange.RANGE_250_DPS
        sleep(0.010)
        self._if_inc = True
        sleep(0.20)
*/
  // set accel data rate == enable

  return true;
}

/*********** typdef enum getter with bitfield *********************/

/**************************************************************************/
/*!
    @brief Gets EXAMPLE VALUE.
    @returns The EXAMPLE VALUE.
*/
LSM6DSOX_example_t Adafruit_LSM6DSOX::getEXAMPLE(void){
    
    Adafruit_BusIO_Register example_register =
      // Adafruit_I2CDevice pointer, address, number of bytes
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL1_XL, 2); 
    Adafruit_BusIO_RegisterBits example_bitfield =
      // register pointer, number of bits, shift
      Adafruit_BusIO_RegisterBits(&example_register, 3, 1);
    return (LSM6DSOX_example_t)example_bitfield.read();
}

/*********** typdef enum setter with bitfield  *********************/

/**************************************************************************/
/*!
    @brief Sets EXAMPLE VALUE.
    @param  example_value
            The EXAMPLE used to EXAMPLE. Must be a
            `LSM6DSOX_example_t`.
*/
void Adafruit_LSM6DSOX::setEXAMPLE(LSM6DSOX_example_t example_value){
    
    Adafruit_BusIO_Register example_register =
      // Adafruit_I2CDevice pointer, address, number of bytes
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL1_XL, 2); 
    Adafruit_BusIO_RegisterBits example_bitfield =
      // register pointer, number of bits, shift
      Adafruit_BusIO_RegisterBits(&example_register, 3, 1);
    example_bitfield.write(example_value);
}