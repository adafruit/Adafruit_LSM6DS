
/*!
 *  @file Adafruit_LSM6DS33.cpp Adafruit LSM6DS33 6-DoF Accelerometer
 *  and Gyroscope library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit LSM6DS33 6-DoF Accelerometer
 *      and Gyroscope library
 *
 * 	This is a library for the Adafruit LSM6DS33 breakout:
 * 	https://www.adafruit.com/product/4480
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

#include "Adafruit_LSM6DS33.h"

/*!
 *    @brief  Instantiates a new LSM6DS33 class
 */
Adafruit_LSM6DS33::Adafruit_LSM6DS33(void) {}

bool Adafruit_LSM6DS33::_init(int32_t sensor_id) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_WHOAMI);

  // make sure we're talking to the right chip
  if (chip_id.read() != LSM6DS33_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;

  reset();

  // enable accelerometer and gyro by setting the data rate to non-zero
  setAccelDataRate(LSM6DS_RATE_104_HZ);
  setGyroDataRate(LSM6DS_RATE_104_HZ);

  delay(10);

  return true;
}
