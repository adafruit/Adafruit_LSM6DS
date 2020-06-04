
/*!
 *  @file Adafruit_LSM6DS33.cpp Adafruit LSM6DS33 6-DoF Accelerometer
 *  and Gyroscope library
 *
 *  Bryan Siepert for Adafruit Industries
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_LSM6DS33.h"

/*!
 *    @brief  Instantiates a new LSM6DS33 class
 */
Adafruit_LSM6DS33::Adafruit_LSM6DS33(void) {}

bool Adafruit_LSM6DS33::_init(int32_t sensor_id) {
  // make sure we're talking to the right chip
  if (chipID() != LSM6DS33_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;

  reset();
  if (chipID() != LSM6DS33_CHIP_ID) {
    return false;
  }

  // enable accelerometer and gyro by setting the data rate to non-zero
  setAccelDataRate(LSM6DS_RATE_104_HZ);
  setGyroDataRate(LSM6DS_RATE_104_HZ);

  delay(10);

  temp_sensor = new Adafruit_LSM6DS_Temp(this);
  accel_sensor = new Adafruit_LSM6DS_Accelerometer(this);
  gyro_sensor = new Adafruit_LSM6DS_Gyro(this);

  return true;
}
