
/*!
 *  @file Adafruit_LSM6DS.cpp Adafruit LSM6DS 6-DoF Accelerometer
 *  and Gyroscope library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver base for Adafruit LSM6DS 6-DoF Accelerometer
 *      and Gyroscope libraries
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

#include "Adafruit_LSM6DS.h"

/*!
 *    @brief  Instantiates a new LSM6DS class
 */
Adafruit_LSM6DS::Adafruit_LSM6DS(void) {}

uint8_t Adafruit_LSM6DS::chipID(void) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_WHOAMI);

  // make sure we're talking to the right chip
  return chip_id.read();
}

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
boolean Adafruit_LSM6DS::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                                     int32_t sensor_id) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_LSM6DS::begin_SPI(uint8_t cs_pin, SPIClass *theSPI,
                                  int32_t sensor_id) {
  i2c_dev = NULL;

  spi_dev = new Adafruit_SPIDevice(cs_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0,             // data mode
                                   theSPI);
  if (!spi_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes software SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  sck_pin The arduino pin # connected to SPI clock
 *    @param  miso_pin The arduino pin # connected to SPI MISO
 *    @param  mosi_pin The arduino pin # connected to SPI MOSI
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_LSM6DS::begin_SPI(int8_t cs_pin, int8_t sck_pin,
                                  int8_t miso_pin, int8_t mosi_pin,
                                  int32_t sensor_id) {
  i2c_dev = NULL;

  spi_dev = new Adafruit_SPIDevice(cs_pin, sck_pin, miso_pin, mosi_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0);            // data mode
  if (!spi_dev->begin()) {
    return false;
  }
  return _init(sensor_id);
}

/**************************************************************************/
/*!
    @brief Resets the sensor to its power-on state, clearing all registers and
   memory
*/
void Adafruit_LSM6DS::reset(void) {

  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL3_C);

  Adafruit_BusIO_RegisterBits sw_reset =
      Adafruit_BusIO_RegisterBits(&ctrl3, 1, 0);

  Adafruit_BusIO_RegisterBits boot = Adafruit_BusIO_RegisterBits(&ctrl3, 1, 7);

  sw_reset.write(true);

  while (sw_reset.read()) {
    delay(1);
  }

  boot.write(true);
  while (boot.read()) {
    delay(1);
  }

  // Adafruit_BusIO_RegisterBits bdu = Adafruit_BusIO_RegisterBits(&ctrl3, 1,
  // 6);

  // bdu.write(true);
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
bool Adafruit_LSM6DS::getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                                 sensors_event_t *temp) {
  uint32_t t = millis();
  _read();

  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = t;
  accel->acceleration.x = accX * SENSORS_GRAVITY_STANDARD / 1000;
  accel->acceleration.y = accY * SENSORS_GRAVITY_STANDARD / 1000;
  accel->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD / 1000;

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
  temp->temperature = (temperature / 256.0) + 25.0;

  return true;
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer data rate.
    @returns The the accelerometer data rate.
*/
lsm6ds33_data_rate_t Adafruit_LSM6DS::getAccelDataRate(void) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 4, 4);

  return (lsm6ds33_data_rate_t)accel_data_rate.read();
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer data rate.
    @param  data_rate
            The the accelerometer data rate. Must be a `lsm6ds33_data_rate_t`.
*/
void Adafruit_LSM6DS::setAccelDataRate(lsm6ds33_data_rate_t data_rate) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 4, 4);

  accel_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer measurement range.
    @returns The the accelerometer measurement range.
*/
lsm6ds33_accel_range_t Adafruit_LSM6DS::getAccelRange(void) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

  return (lsm6ds33_accel_range_t)accel_range.read();
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range.
    @param new_range The `lsm6ds33_accel_range_t` range to set.
*/
void Adafruit_LSM6DS::setAccelRange(lsm6ds33_accel_range_t new_range) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

  accel_range.write(new_range);
  delay(20);
}

/**************************************************************************/
/*!
    @brief Gets the gyro data rate.
    @returns The the gyro data rate.
*/
lsm6ds33_data_rate_t Adafruit_LSM6DS::getGyroDataRate(void) {

  Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL2_G);

  Adafruit_BusIO_RegisterBits gyro_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl2, 4, 4);

  return (lsm6ds33_data_rate_t)gyro_data_rate.read();
}

/**************************************************************************/
/*!
    @brief Sets the gyro data rate.
    @param  data_rate
            The the gyro data rate. Must be a `lsm6ds33_data_rate_t`.
*/
void Adafruit_LSM6DS::setGyroDataRate(lsm6ds33_data_rate_t data_rate) {

  Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL2_G);

  Adafruit_BusIO_RegisterBits gyro_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl2, 4, 4);

  gyro_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Gets the gyro range.
    @returns The the gyro range.
*/
lsm6ds33_gyro_range_t Adafruit_LSM6DS::getGyroRange(void) {

  Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL2_G);

  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&ctrl2, 3, 1);

  return (lsm6ds33_gyro_range_t)gyro_range.read();
}

/**************************************************************************/
/*!
    @brief Sets the gyro range.
    @param new_range The `lsm6ds33_gyro_range_t` to set.
*/
void Adafruit_LSM6DS::setGyroRange(lsm6ds33_gyro_range_t new_range) {

  Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL2_G);

  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&ctrl2, 3, 1);

  gyro_range.write(new_range);
  delay(20);
}
/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void Adafruit_LSM6DS::_read(void) {
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

  lsm6ds33_gyro_range_t gyro_range = getGyroRange();
  float gyro_scale = 1;
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

  gyroX = rawGyroX * gyro_scale;
  gyroY = rawGyroY * gyro_scale;
  gyroZ = rawGyroZ * gyro_scale;

  lsm6ds33_accel_range_t accel_range = getAccelRange();
  float accel_scale = 1;
  if (accel_range == LSM6DS_ACCEL_RANGE_16_G)
    accel_scale = 0.488;
  if (accel_range == LSM6DS_ACCEL_RANGE_8_G)
    accel_scale = 0.244;
  if (accel_range == LSM6DS_ACCEL_RANGE_4_G)
    accel_scale = 0.122;
  if (accel_range == LSM6DS_ACCEL_RANGE_2_G)
    accel_scale = 0.061;

  accX = rawAccX * accel_scale;
  accY = rawAccY * accel_scale;
  accZ = rawAccZ * accel_scale;
}

/**************************************************************************/
/*!
    @brief Sets the INT1 and INT2 pin activation mode
    @param active_low true to set the pins  as active high, false to set the
   mode to active low
    @param open_drain true to set the pin mode as open-drain, false to set the
   mode to push-pull
*/
void Adafruit_LSM6DS::configIntOutputs(bool active_low, bool open_drain) {

  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL3_C);

  Adafruit_BusIO_RegisterBits ppod_bits =
      Adafruit_BusIO_RegisterBits(&ctrl3, 2, 4);

  ppod_bits.write((active_low << 1) | open_drain);
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 1.
    @param drdy_temp true to output the data ready temperature interrupt
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
*/
void Adafruit_LSM6DS::configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl) {

  Adafruit_BusIO_Register int1_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_INT1_CTRL);

  Adafruit_BusIO_RegisterBits int1_drdy_bits =
      Adafruit_BusIO_RegisterBits(&int1_ctrl, 3, 0);

  int1_drdy_bits.write((drdy_temp << 2) | (drdy_g << 1) | drdy_xl);
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 2.
    @param drdy_temp true to output the data ready temperature interrupt
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
*/
void Adafruit_LSM6DS::configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl) {

  Adafruit_BusIO_Register int2_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_INT2_CTRL);

  Adafruit_BusIO_RegisterBits int2_drdy_bits =
      Adafruit_BusIO_RegisterBits(&int2_ctrl, 3, 0);

  int2_drdy_bits.write((drdy_temp << 2) | (drdy_g << 1) | drdy_xl);
}
