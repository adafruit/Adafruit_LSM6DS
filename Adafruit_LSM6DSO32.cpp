
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
  // Might have been stopped in a strange state, reset the register access to
  // default state
  funcCfgAccess();

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

  // Set the block data update so it doesn't change a value while we're in the
  // middle of reading the low and high bytes
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

/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void Adafruit_LSM6DSO32::_read(void) {
  // get raw readings
  Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_OUT_TEMP_L, 14);

  uint8_t buffer[14];
  data_reg.read(buffer, 14);

  rawTemp = buffer[1] << 8 | buffer[0];
  temperature = (rawTemp / 256.0) + 25.0;

  rawGyroX = buffer[3] << 8 | buffer[2];
  rawGyroY = buffer[5] << 8 | buffer[4];
  rawGyroZ = buffer[7] << 8 | buffer[6];

  rawAccX = buffer[9] << 8 | buffer[8];
  rawAccY = buffer[11] << 8 | buffer[10];
  rawAccZ = buffer[13] << 8 | buffer[12];

  float gyro_scale = 1; // range is in milli-dps per bit!
  switch (gyroRangeBuffered) {
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    gyro_scale = 140.0;
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    gyro_scale = 70.0;
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    gyro_scale = 35.0;
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    gyro_scale = 17.50;
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    gyro_scale = 8.75;
    break;
  case LSM6DS_GYRO_RANGE_125_DPS:
    gyro_scale = 4.375;
    break;
  }

  gyroX = rawGyroX * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyroY = rawGyroY * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyroZ = rawGyroZ * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;

  float accel_scale = 1; // range is in milli-g per bit!
  switch (accelRangeBuffered) {
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

/* Read Fast: only get basic gyro and accel data. Saves 2 bytes over the wire
 * and a floating-point operation*/
/* NOTE THIS IS NOT STANDARD UNITS FOR ADAFRUIT - THESE ARE PREPARED FOR MY
   MAHONEY ALGORITHM (deg/s for gyro) ACCEL AND MAG DON'T NEED UNITS AS THEY ARE
   ONLY DIRECTIONS FOR MAHONEY*/
void Adafruit_LSM6DSO32::_readFast(void) {
  // get raw readings
  const int bytesToRead = (3 + 3) * 2; // 3 gyro, 3 accel, 2 bytes each
  Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_OUTX_L_G, bytesToRead);

  uint8_t buffer[bytesToRead];
  data_reg.read(buffer, bytesToRead);

  rawGyroX = buffer[1] << 8 | buffer[0];
  rawGyroY = buffer[3] << 8 | buffer[2];
  rawGyroZ = buffer[5] << 8 | buffer[4];

  rawAccX = buffer[7] << 8 | buffer[6];
  rawAccY = buffer[9] << 8 | buffer[8];
  rawAccZ = buffer[11] << 8 | buffer[10];

  float gyro_scale = 1; // range is in milli-dps per bit!
  switch (gyroRangeBuffered) {
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    gyro_scale = 140.0;
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    gyro_scale = 70.0;
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    gyro_scale = 35.0;
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    gyro_scale = 17.50;
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    gyro_scale = 8.75;
    break;
  case LSM6DS_GYRO_RANGE_125_DPS:
    gyro_scale = 4.375;
    break;
  }

  gyroX = rawGyroX * gyro_scale / 1000.0;
  gyroY = rawGyroY * gyro_scale / 1000.0;
  gyroZ = rawGyroZ * gyro_scale / 1000.0;

  float accel_scale = 1; // range is in milli-g per bit!
  switch (accelRangeBuffered) {
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
  default:
    Serial.println("unknown scale in accelerometer");
    break;
  }
  accX = rawAccX * accel_scale / 1000;
  accY = rawAccY * accel_scale / 1000;
  accZ = rawAccZ * accel_scale / 1000;
}
