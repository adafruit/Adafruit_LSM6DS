
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
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_LSM6DSOX::begin(uint8_t i2c_address, TwoWire *wire, int32_t sensor_id) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  Adafruit_BusIO_Register chip_id =
    Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_WHOAMI);

  // make sure we're talking to the right chip
  if (chip_id.read() != LSM6DSOX_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 1;

  reset();
  Adafruit_BusIO_Register ctrl3 =
    Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL3_C);
  Adafruit_BusIO_RegisterBits bdu =
      Adafruit_BusIO_RegisterBits(&ctrl3, 1, 6);

  bdu.write(true);

  // enable accelerometer and gyro by setting the data rate to non-zero (disabled)
  setAccelDataRate(LSM6DSOX_RATE_104_HZ);
  setGyroDataRate(LSM6DSOX_RATE_104_HZ);

  return true;
}

/**************************************************************************/
/*!
    @brief Resets the sensor to its power-on state, clearing all registers and memory
*/
void Adafruit_LSM6DSOX::reset(void) {

    Adafruit_BusIO_Register ctrl3 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL3_C);

    Adafruit_BusIO_RegisterBits sw_reset =
      Adafruit_BusIO_RegisterBits(&ctrl3, 1, 0);

    Adafruit_BusIO_RegisterBits boot =
      Adafruit_BusIO_RegisterBits(&ctrl3, 1, 7);

    sw_reset.write(true);
    while (sw_reset.read()){
      delay(1);
    }

    boot.write(true);
    while (boot.read()){
      delay(1);
    }

}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  accel
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with acceleration event data.

    @param  gyro
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with gyro event data.

    @param  temp
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with temperature event data.

    @return True on successful read
*/
/**************************************************************************/
 bool Adafruit_LSM6DSOX::getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp){
  uint32_t t = millis();
  _read();

  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = t;
  accel->acceleration.x = accX * SENSORS_GRAVITY_STANDARD/1000;
  accel->acceleration.y = accY * SENSORS_GRAVITY_STANDARD/1000;
  accel->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD/1000;


  memset(gyro, 0, sizeof(sensors_event_t));
  gyro->version = 1;
  gyro->sensor_id = _sensorid_gyro;
  gyro->type = SENSOR_TYPE_GYROSCOPE;
  gyro->timestamp = t;
  gyro->gyro.x = gyroX / 1000;
  gyro->gyro.y = gyroY / 1000;
  gyro->gyro.z = gyroZ / 1000;

  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = t;
  temp->temperature = temperature;


  return true;
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer data rate.
    @returns The the accelerometer data rate.
*/
lsm6dsox_data_rate_t Adafruit_LSM6DSOX::getAccelDataRate(void){

    Adafruit_BusIO_Register ctrl1 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL1_XL);

    Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 4, 4);

    return accel_data_rate.read();
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer data rate.
    @param  data_rate
            The the accelerometer data rate. Must be a `lsm6dsox_data_rate_t`.
*/
void Adafruit_LSM6DSOX::setAccelDataRate(lsm6dsox_data_rate_t data_rate){

    Adafruit_BusIO_Register ctrl1 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL1_XL);

    Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 4, 4);

    accel_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer measurement range.
    @returns The the accelerometer measurement range.
*/
lsm6dsox_accel_range_t Adafruit_LSM6DSOX::getAccelRange(void){

    Adafruit_BusIO_Register ctrl1 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL1_XL);

    Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

    return accel_range.read();
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range.
    @param new_range The `lsm6dsox_accel_range_t` range to set.
*/
void Adafruit_LSM6DSOX::setAccelRange(lsm6dsox_accel_range_t new_range){

    Adafruit_BusIO_Register ctrl1 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL1_XL);

    Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

    accel_range.write(new_range);
}

/**************************************************************************/
/*!
    @brief Gets the gyro data rate.
    @returns The the gyro data rate.
*/
lsm6dsox_data_rate_t Adafruit_LSM6DSOX::getGyroDataRate(void){

    Adafruit_BusIO_Register ctrl2 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL2_G);

    Adafruit_BusIO_RegisterBits gyro_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl2, 4, 4);

    return gyro_data_rate.read();
}

/**************************************************************************/
/*!
    @brief Sets the gyro data rate.
    @param  data_rate
            The the gyro data rate. Must be a `lsm6dsox_data_rate_t`.
*/
void Adafruit_LSM6DSOX::setGyroDataRate(lsm6dsox_data_rate_t data_rate){

    Adafruit_BusIO_Register ctrl2 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL2_G);

    Adafruit_BusIO_RegisterBits gyro_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl2, 4, 4);

    gyro_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Gets the gyro range.
    @returns The the gyro range.
*/
lsm6dsox_gyro_range_t Adafruit_LSM6DSOX::getGyroRange(void){

    Adafruit_BusIO_Register ctrl2 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL2_G);

    Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&ctrl2, 2, 2);

    return gyro_range.read();
}

/**************************************************************************/
/*!
    @brief Sets the gyro range.
    @param new_range The `lsm6dsox_gyro_range_t` to set.
*/
void Adafruit_LSM6DSOX::setGyroRange(lsm6dsox_gyro_range_t new_range){

    Adafruit_BusIO_Register ctrl2 =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_CTRL2_G);

    Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&ctrl2, 2, 2);

    gyro_range.write(new_range);
}
/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void Adafruit_LSM6DSOX::_read(void) {
  // get raw readings
  Adafruit_BusIO_Register data_reg =
      Adafruit_BusIO_Register(i2c_dev, LSM6DSOX_OUT_TEMP_L, 14);

  uint8_t buffer[14];
  data_reg.read(buffer, 14);

  temperature = buffer[1] << 8 | buffer[0];

  rawGyroY = buffer[3] << 8 | buffer[2];
  rawGyroZ = buffer[5] << 8 | buffer[4];
  rawGyroX = buffer[7] << 8 | buffer[6];

  rawAccY = buffer[9] << 8 | buffer[8];
  rawAccZ = buffer[11] << 8 | buffer[10];
  rawAccZ = buffer[13] << 8 | buffer[10];

  lsm6dsox_gyro_range_t gyro_range = getGyroRange();
  float gyro_scale = 1;
  if (gyro_range == LSM6DSOX_GYRO_RANGE_2000_DPS)
    gyro_scale = 70.0;
  if (gyro_range == LSM6DSOX_GYRO_RANGE_1000_DPS)
    gyro_scale = 35.0;
  if (gyro_range == LSM6DSOX_GYRO_RANGE_500_DPS)
    gyro_scale = 17.50;
  if (gyro_range == LSM6DSOX_GYRO_RANGE_250_DPS)
    gyro_scale =  8.75;

  gyroX = rawGyroX * gyro_scale;
  gyroY = rawGyroY * gyro_scale;
  gyroZ = rawGyroZ * gyro_scale;
  
  lsm6dsox_accel_range_t accel_range = getAccelRange();
  float accel_scale = 1;
  if (accel_range == LSM6DSOX_ACCEL_RANGE_16_G)
    accel_scale = 0.488;
  if (accel_range == LSM6DSOX_ACCEL_RANGE_8_G)
    accel_scale = 0.244;
  if (accel_range == LSM6DSOX_ACCEL_RANGE_4_G)
    accel_scale = 0.122;
  if (accel_range == LSM6DSOX_ACCEL_RANGE_2_G)
    accel_scale = 0.061;

  accX = rawAccX * accel_scale;
  accY = rawAccY * accel_scale;
  accZ = rawAccZ * accel_scale;

}
