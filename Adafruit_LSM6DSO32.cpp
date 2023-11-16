
/*!
 *  @file Adafruit_LSM6DSO32.cpp
 *  Adafruit LSM6DSO32 6-DoF Accelerometer and Gyroscope library
 *
 *  Bryan Siepert for Adafruit Industries
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_LSM6DSO32.h"
#include "Adafruit_LSM6DSOX.h"

/*!
 *    @brief  Instantiates a new LSM6DSO32 class
 */
Adafruit_LSM6DSO32::Adafruit_LSM6DSO32(void) {}

bool Adafruit_LSM6DSO32::_init(int32_t sensor_id) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_WHOAMI);

  // make sure we're talking to the right chip
  if (chip_id.read() != LSM6DSO32_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;

  reset();

  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL3_C);
  Adafruit_BusIO_RegisterBits bdu = Adafruit_BusIO_RegisterBits(&ctrl3, 1, 6);
  bdu.write(true);

  // Disable I3C
  Adafruit_BusIO_Register ctrl_9 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL9_XL);
  Adafruit_BusIO_RegisterBits i3c_disable_bit =
      Adafruit_BusIO_RegisterBits(&ctrl_9, 1, 1);

  i3c_disable_bit.write(true);

  // call base class _init()
  Adafruit_LSM6DS::_init(sensor_id);

  return true;
}

/**************************************************************************/
/*!
 *  @brief convert raw accelerometer data
 */
void Adafruit_LSM6DSO32::convertRawAccelerometerValues() {
  float accel_scale = 1; // range is in milli-g per bit!
  switch ((lsm6dso32_accel_range_t)accelRangeBuffered) {
  case LSM6DSO32_ACCEL_RANGE_32_G:
    accel_scale = 0.976;
    break;
  case LSM6DSO32_ACCEL_RANGE_16_G:
    accel_scale = 0.488;
    break;
  case LSM6DSO32_ACCEL_RANGE_8_G:
    accel_scale = 0.244;
    break;
  case LSM6DSO32_ACCEL_RANGE_4_G:
    accel_scale = 0.122;
    break;
  }

  accX = rawAccX * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  accY = rawAccY * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  accZ = rawAccZ * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer measurement range.
    @returns The the accelerometer measurement range.
*/
lsm6dso32_accel_range_t Adafruit_LSM6DSO32::getAccelRange(void) {
  return (lsm6dso32_accel_range_t)Adafruit_LSM6DS::getAccelRange();
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range.
    @param new_range The `lsm6dso32_accel_range_t` range to set.
*/
void Adafruit_LSM6DSO32::setAccelRange(lsm6dso32_accel_range_t new_range) {
  Adafruit_LSM6DS::setAccelRange((lsm6ds_accel_range_t)new_range);
}
