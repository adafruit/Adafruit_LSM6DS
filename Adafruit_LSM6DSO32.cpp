
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

  // enable accelerometer and gyro by setting the data rate to non-zero
  setAccelDataRate(LSM6DS_RATE_104_HZ);
  setGyroDataRate(LSM6DS_RATE_104_HZ);

  delay(10);

  temp_sensor = new Adafruit_LSM6DS_Temp(this);
  accel_sensor = new Adafruit_LSM6DS_Accelerometer(this);
  gyro_sensor = new Adafruit_LSM6DS_Gyro(this);

  return true;
}

// /**************************************************************************/
// /*!
//     @brief Disables and enables the SPI master bus pulllups.
//     @param disable_pullups true to **disable** the I2C pullups, false to enable.
// */
// void Adafruit_LSM6DSO32::disableSPIMasterPullups(bool disable_pullups) {

//   Adafruit_BusIO_Register pin_config = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSO32_PIN_CTRL);

//   Adafruit_BusIO_RegisterBits disable_ois_pu =
//       Adafruit_BusIO_RegisterBits(&pin_config, 1, 7);

//   disable_ois_pu.write(disable_pullups);
// }

// /**************************************************************************/
// /*!
//     @brief Enables and disables the I2C master bus pulllups.
//     @param enable_pullups true to enable the I2C pullups, false to disable.
// */
// void Adafruit_LSM6DSO32::enableI2CMasterPullups(bool enable_pullups) {

//   Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSO32_FUNC_CFG_ACCESS);
//   Adafruit_BusIO_RegisterBits master_cfg_enable_bit =
//       Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 6);

//   Adafruit_BusIO_Register master_config = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSO32_MASTER_CONFIG);
//   Adafruit_BusIO_RegisterBits i2c_master_pu_en =
//       Adafruit_BusIO_RegisterBits(&master_config, 1, 3);

//   master_cfg_enable_bit.write(true);
//   i2c_master_pu_en.write(enable_pullups);
//   master_cfg_enable_bit.write(false);
// }
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

  temperature = buffer[1] << 8 | buffer[0];

  rawGyroX = buffer[3] << 8 | buffer[2];
  rawGyroY = buffer[5] << 8 | buffer[4];
  rawGyroZ = buffer[7] << 8 | buffer[6];

  rawAccX = buffer[9] << 8 | buffer[8];
  rawAccY = buffer[11] << 8 | buffer[10];
  rawAccZ = buffer[13] << 8 | buffer[12];
  // FS = ±125 dps 4.375
  // mdps/LSB
  // FS = ±250 dps 8.75
  // FS = ±500 dps 17.50
  // FS = ±1000 dps 35
  // FS = ±2000 dps 70
  lsm6ds_gyro_range_t gyro_range = getGyroRange();
  float gyro_scale = 1; // range is in milli-dps per bit!
  if (gyro_range == ISM330DHCX_GYRO_RANGE_4000_DPS)
    gyro_scale = 140.0;
  if (gyro_range == LSM6DS_GYRO_RANGE_2000_DPS)
    gyro_scale = 70.0;
  if (gyro_range == LSM6DS_GYRO_RANGE_1000_DPS)
    gyro_scale = 35.0;
  if (gyro_range == LSM6DS_GYRO_RANGE_500_DPS)
    gyro_scale = 17.50;
  if (gyro_range == LSM6DS_GYRO_RANGE_250_DPS)
    gyro_scale = 8.75;
  if (gyro_range == LSM6DS_GYRO_RANGE_125_DPS)
    gyro_scale = 4.375;

  gyroX = rawGyroX * gyro_scale * 1 / 1000.0;
  gyroY = rawGyroY * gyro_scale * 1 / 1000.0;
  gyroZ = rawGyroZ * gyro_scale * 1 / 1000.0;

  lsm6dso32_accel_range_t accel_range = getAccelRange();
  float accel_scale = 1; // range is in milli-g per bit!
  if (accel_range == LSM6DSO32_ACCEL_RANGE_32_G)
    accel_scale = 0.976;
  if (accel_range == LSM6DSO32_ACCEL_RANGE_16_G)
    accel_scale = 0.488;
  if (accel_range == LSM6DSO32_ACCEL_RANGE_8_G)
    accel_scale = 0.244;
  if (accel_range == LSM6DSO32_ACCEL_RANGE_4_G)
    accel_scale = 0.122;
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

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

  return (lsm6dso32_accel_range_t)accel_range.read();
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range.
    @param new_range The `lsm6dso32_accel_range_t` range to set.
*/
void Adafruit_LSM6DSO32::setAccelRange(lsm6dso32_accel_range_t new_range) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

  accel_range.write(new_range);
  delay(20);
}