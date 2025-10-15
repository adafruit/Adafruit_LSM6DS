
/*!
 *  @file Adafruit_LSM6DSV.cpp Adafruit LSM6DS 9-DoF Accelerometer
 *  and Gyroscope library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver base for Adafruit LSM6DSV 9-DoF Accelerometer
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
 *  Morgan Sandercock for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_LSM6DSV.h"

static const float _data_rate_arr[] = {
    [LSM6DSV_RATE_SHUTDOWN] = 0.0f,    [LSM6DSV_RATE_1_875_HZ] = 1.875f,
    [LSM6DSV_RATE_7_5_HZ] = 7.5f,      [LSM6DSV_RATE_15_HZ] = 15.0f,
    [LSM6DSV_RATE_30_HZ] = 30.0f,      [LSM6DSV_RATE_60_HZ] = 60.0f,
    [LSM6DSV_RATE_120_HZ] = 120.0f,    [LSM6DSV_RATE_240_HZ] = 240.0f,
    [LSM6DSV_RATE_480_HZ] = 480.0f,    [LSM6DSV_RATE_960_HZ] = 960.0f,
    [LSM6DSV_RATE_1_92K_HZ] = 1920.0f, [LSM6DSV_RATE_3_84K_HZ] = 3840.0f,
    [LSM6DSV_RATE_7_68K_HZ] = 7680.0f
};

/*!
 *    @brief  Instantiates a new LSM6DSV class
 */
Adafruit_LSM6DSV::Adafruit_LSM6DSV(void) {}

/*!
 *    @brief  Cleans up the LSM6DSV
 */
Adafruit_LSM6DSV::~Adafruit_LSM6DSV(void) { 
  delete temp_sensor; 
  delete accel_sensor;
  delete gyro_sensor;
  delete high_g_sensor;
}

/*!  @brief  Unique subclass initializer post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_LSM6DSV::_init(int32_t sensor_id) {

  _sensorid_accel = sensor_id;
  _sensorid_gyro = sensor_id + 1;
  _sensorid_temp = sensor_id + 2;
  _sensorid_high_g = sensor_id + 3;


  reset();


  // Enable accelerometer with 120 Hz data rate, 4G
  setAccelDataRate(LSM6DSV_RATE_120_HZ);
  setAccelRange(LSM6DSV_ACCEL_RANGE_4_G);

  // Enable gyro with 120 Hz data rate, 2000 dps
  setGyroDataRate(LSM6DSV_RATE_120_HZ);
  setGyroRange(LSM6DSV_GYRO_RANGE_2000_DPS);

  //enable high-g outputs
  enableHighG(true);

  delay(10);

  // delete objects if sensor is reinitialized
  delete temp_sensor;
  delete accel_sensor;
  delete gyro_sensor;
  delete high_g_sensor;

  temp_sensor = new Adafruit_LSM6DSV_Temp(this);
  accel_sensor = new Adafruit_LSM6DSV_Accelerometer(this);
  gyro_sensor = new Adafruit_LSM6DSV_Gyro(this);
  high_g_sensor = new Adafruit_LSM6DSV_HighG(this);

  return (chipID() == LSM6DSV_CHIP_ID);
}

/*!
 *    @brief  Read chip identification register
 *    @returns 8 Bit value from WHOAMI register
 */
uint8_t Adafruit_LSM6DSV::chipID(void) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_WHOAMI);
  //Serial.print("Read ID 0x"); Serial.println(chip_id.read(), HEX);

  return chip_id.read();
}

/*!
 *    @brief  Read Status register
 *    @returns 8 Bit value from Status register
 */
uint8_t Adafruit_LSM6DSV::status(void) {
  Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_STATUS_REG);
  return status_reg.read();
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
boolean Adafruit_LSM6DSV::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                                   int32_t sensor_id) {
  delete i2c_dev; // remove old interface


  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    //Serial.println("No I2C device at the given address");
    return false;
  }

  i2c_dev->setSpeed(400000); //crank it up to max "normal I2C" speed 400kHz

  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @param  frequency The SPI bus frequency
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_LSM6DSV::begin_SPI(uint8_t cs_pin, SPIClass *theSPI,
                                int32_t sensor_id, uint32_t frequency) {
  i2c_dev = NULL;

  delete spi_dev; // remove old interface

  spi_dev = new Adafruit_SPIDevice(cs_pin,
                                   frequency,             // frequency
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
 *    @param  frequency The SPI bus frequency
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_LSM6DSV::begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                                int8_t mosi_pin, int32_t sensor_id,
                                uint32_t frequency) {
  i2c_dev = NULL;

  delete spi_dev; // remove old interface

  spi_dev = new Adafruit_SPIDevice(cs_pin, sck_pin, miso_pin, mosi_pin,
                                   frequency,             // frequency
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

   If you aren't talking to a LSM6DS chip then you can get stuck here as maybe the
   bit can be set but then won't un-set itself to show that the reset is done.
   Use a 1-second timeout to get un-stuck. Maybe return an error code?
*/
void Adafruit_LSM6DSV::reset(void) {

  //It is possible for the embedded functions access bit to be set (we interrupted it in the middle of an operation)
  //and this is very persistent. When that is enabled, then you can't access the CFG3 register to do a master reset.
  //So the first step in bringing it back from a random position is to turn that off. 
  //Since we're there already, we'll do a SW_POR (software power-on reset? Global reset?) on the embedded functions too.
  funcCfgAccess(false, false, false, true);
  delay(1); //Wait for the reset? Datasheet doesn't say.


  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL3);

  Adafruit_BusIO_RegisterBits sw_reset =
      Adafruit_BusIO_RegisterBits(&ctrl3, 1, 0);

  sw_reset.write(true);

  unsigned long start = millis();
  while (sw_reset.read() && millis() - start < 1000) {
    delay(1);
  } 

}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the temp sensor component
    @return Adafruit_Sensor pointer to temperature sensor
 */
Adafruit_Sensor *Adafruit_LSM6DSV::getTemperatureSensor(void) {
  return temp_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the accelerometer
    sensor component
    @return Adafruit_Sensor pointer to accelerometer sensor
 */
Adafruit_Sensor *Adafruit_LSM6DSV::getAccelerometerSensor(void) {
  return accel_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the gyro sensor component
    @return Adafruit_Sensor pointer to gyro sensor
 */
Adafruit_Sensor *Adafruit_LSM6DSV::getGyroSensor(void) { 
  return gyro_sensor; 
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the high-g sensor component
    @return Adafruit_Sensor pointer to high-g sensor
 */
Adafruit_Sensor *Adafruit_LSM6DSV::getHighGSensor(void) { 
  return high_g_sensor; 
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
bool Adafruit_LSM6DSV::getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                               sensors_event_t *temp) {
  uint32_t t = millis();
  _read();

  // use helpers to fill in the events
  fillAccelEvent(accel, t);
  fillGyroEvent(gyro, t);
  fillTempEvent(temp, t);
  return true;
}
/**************************************************************************/
/*!
    @brief  Gets the most recent accel+gyro event, Adafruit Unified Sensor format
    @param  accel
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with acceleration event data.

    @param  gyro
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with gyro event data.

    @return True on successful read
*/
/**************************************************************************/
bool Adafruit_LSM6DSV::getEvent(sensors_event_t *accel, sensors_event_t *gyro) {
  uint32_t t = millis();
  _readFast();

  // use helpers to fill in the events
  fillAccelEvent(accel, t);
  fillGyroEvent(gyro, t);
  return true;
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

    @param  highG
            Pointer to an Adafruit Unified sensor_event_t object to be filled
            with high-G acceleration event data.

    @return True on successful read
*/
/**************************************************************************/
bool Adafruit_LSM6DSV::getEvent(sensors_event_t *accel, 
                               sensors_event_t *gyro,
                               sensors_event_t *temp,
                               sensors_event_t *highG) {
  uint32_t t = millis();
  _read();

  // use helpers to fill in the events
  fillAccelEvent(accel, t);
  fillGyroEvent(gyro, t);
  fillTempEvent(temp, t);
  fillHighGEvent(highG, t);
  return true;
}

/**************************************************************************/
//Helpers for events
/**************************************************************************/
void Adafruit_LSM6DSV::fillTempEvent(sensors_event_t *temp, uint32_t timestamp) {
  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = timestamp;
  temp->temperature = temperature;
}

void Adafruit_LSM6DSV::fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp) {
  memset(gyro, 0, sizeof(sensors_event_t));
  gyro->version = 1;
  gyro->sensor_id = _sensorid_gyro;
  gyro->type = SENSOR_TYPE_GYROSCOPE;
  gyro->timestamp = timestamp;
  gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS;
  gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
  gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
}

void Adafruit_LSM6DSV::fillAccelEvent(sensors_event_t *accel, uint32_t timestamp) {
  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = timestamp;
  accel->acceleration.x = accX;
  accel->acceleration.y = accY;
  accel->acceleration.z = accZ;
}

void Adafruit_LSM6DSV::fillHighGEvent(sensors_event_t *accel, uint32_t timestamp) {
  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_high_g;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = timestamp;
  accel->acceleration.x = highGX;
  accel->acceleration.y = highGY;
  accel->acceleration.z = highGZ;
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer data rate.
    @returns The the accelerometer data rate.
*/
lsm6dsv_data_rate_t Adafruit_LSM6DSV::getAccelDataRate(void) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL1);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 4, 0);

  return (lsm6dsv_data_rate_t)accel_data_rate.read();
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer data rate.
    @param  data_rate
            The the accelerometer data rate. Must be a `lsm6dsv_data_rate_t`.
*/
void Adafruit_LSM6DSV::setAccelDataRate(lsm6dsv_data_rate_t data_rate) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL1);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 4, 0);

  accel_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer measurement range.
    @returns The the accelerometer measurement range. (identical for all except DSV32)
*/
lsm6dsv_accel_range_t Adafruit_LSM6DSV::getAccelRange(void) {

  Adafruit_BusIO_Register ctrl8 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL8);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl8, 2, 0);

  accelRangeBuffered = (lsm6dsv_accel_range_t)accel_range.read();

  setAccelScale();

  return accelRangeBuffered;
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range.
    @param new_range The `lsm6dsv_accel_range_t` range to set.
*/
void Adafruit_LSM6DSV::setAccelRange(lsm6dsv_accel_range_t new_range) {

  Adafruit_BusIO_Register ctrl8 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL8);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl8, 2, 0);

  accel_range.write(new_range);

  accelRangeBuffered = new_range;

  setAccelScale();
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer scale from the "buffered" range enum.
*/
void Adafruit_LSM6DSV::setAccelScale() {

  //magic numbers from the datasheet are milli-gee per LSB
  switch (accelRangeBuffered) {
  case LSM6DSV_ACCEL_RANGE_16_G:
    accelScale = 0.488 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  case LSM6DSV_ACCEL_RANGE_8_G:
    accelScale = 0.244 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  case LSM6DSV_ACCEL_RANGE_4_G:
    accelScale = 0.122 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  case LSM6DSV_ACCEL_RANGE_2_G:
    accelScale = 0.061 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  }
}

/**************************************************************************/
/*!
    @brief Gets the gyro data rate.
    @returns The the gyro data rate.
*/
lsm6dsv_data_rate_t Adafruit_LSM6DSV::getGyroDataRate(void) {

  Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL2);

  Adafruit_BusIO_RegisterBits gyro_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl2, 4, 0);

  return (lsm6dsv_data_rate_t)gyro_data_rate.read();
}

/**************************************************************************/
/*!
    @brief Sets the gyro data rate.
    @param  data_rate
            The the gyro data rate. Must be a `lsm6dsv_data_rate_t`.
			1.875Hz is not allowed for the gyro
*/
void Adafruit_LSM6DSV::setGyroDataRate(lsm6dsv_data_rate_t data_rate) {

  Adafruit_BusIO_Register ctrl2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL2);

  Adafruit_BusIO_RegisterBits gyro_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl2, 4, 0);

  gyro_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Gets the gyro range.
    @returns The the gyro range.
*/
lsm6dsv_gyro_range_t Adafruit_LSM6DSV::getGyroRange(void) {

  Adafruit_BusIO_Register ctrl6 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL6);

  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&ctrl6, 3, 0);

  gyroRangeBuffered = (lsm6dsv_gyro_range_t)gyro_range.read();

  setGyroScale();

  return gyroRangeBuffered;
}

/**************************************************************************/
/*!
    @brief Sets the gyro range.
    @param new_range The `lsm6dsv_gyro_range_t` to set.
*/
void Adafruit_LSM6DSV::setGyroRange(lsm6dsv_gyro_range_t new_range) {

  Adafruit_BusIO_Register ctrl6 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL6);

  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&ctrl6, 3, 0);

  gyro_range.write(new_range);

  gyroRangeBuffered = new_range;

  setGyroScale();
}

/**************************************************************************/
/*!
    @brief Sets the gyro scale from the "buffered" range enum.
*/
void Adafruit_LSM6DSV::setGyroScale() {

  //datasheet magic numbers are milli-degrees per second per LSB
  switch (gyroRangeBuffered) {
  case LSM6DSV_GYRO_RANGE_4000_DPS:
    gyroScale = 140.0 / 1000.0;
    break;
  case LSM6DSV_GYRO_RANGE_2000_DPS:
    gyroScale = 70.0 / 1000.0;
    break;
  case LSM6DSV_GYRO_RANGE_1000_DPS:
    gyroScale = 35.0 / 1000.0;
    break;
  case LSM6DSV_GYRO_RANGE_500_DPS:
    gyroScale = 17.50 / 1000.0;
    break;
  case LSM6DSV_GYRO_RANGE_250_DPS:
    gyroScale = 8.75 / 1000.0;
    break;
  case LSM6DSV_GYRO_RANGE_RESERVED: //Should never see this, but need to cover all of the enum
    gyroScale = 8.75 / 1000.0;
    break;
  }

}

/**************************************************************************/
/*
 * Note there are no range or rate settings for the high-g accelerometer
 * in the DSV, DSV16, DSV16B, DSV32. They are fixed to the same rate as
 * the main accelerometer and the range is always the maximum of the 
 * main accelerometer ranges.
*/
/**************************************************************************/

  
/**************************************************************************/
/*!
    @brief Gets the internal frequency fine adjustment (read-only).
    @returns The the adjustment value (steps of 0.13%).
*/
int8_t Adafruit_LSM6DSV::getInternalFreqFine(void) {

  Adafruit_BusIO_Register internal_freq = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_INTERNAL_FREQ);

  return (int8_t)internal_freq.read();

}

/**************************************************************************/
/*!
    @brief Enables the high pass filter and/or slope filter.
    @param filter_enabled Whether to enable the slope filter (see datasheet)
    @param filter The lsm6dsv_filter_t that sets the data rate divisor
*/
/**************************************************************************/
void Adafruit_LSM6DSV::accelHighPassFilter(bool filter_enabled,
                                     lsm6dsv_filter_t filter) {
  Adafruit_BusIO_Register ctrl8 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL8);
  Adafruit_BusIO_RegisterBits HP_LPF2_XL_BW =
      Adafruit_BusIO_RegisterBits(&ctrl8, 3, 5);
  HP_LPF2_XL_BW.write(filter);

  Adafruit_BusIO_Register ctrl9 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL9);
  Adafruit_BusIO_RegisterBits HP_SLOPE_XL_EN =
      Adafruit_BusIO_RegisterBits(&ctrl9, 1, 4);
  HP_SLOPE_XL_EN.write(filter_enabled);
}

/**************************************************************************/
/*!
    @brief Enables the low pass filter (disables high-pass if necessary).
    @param filter_enabled Whether to enable the LP filter (see datasheet)
    @param filter The lsm6dsv_filter_t that sets the data rate divisor
*/
/**************************************************************************/
void Adafruit_LSM6DSV::accelLowPassFilter(bool filter_enabled,
                                     lsm6dsv_filter_t filter) {
  Adafruit_BusIO_Register ctrl8 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL8);
  Adafruit_BusIO_RegisterBits HP_LPF2_XL_BW =
      Adafruit_BusIO_RegisterBits(&ctrl8, 3, 5);
  HP_LPF2_XL_BW.write(filter);

  Adafruit_BusIO_Register ctrl9 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL9);
  Adafruit_BusIO_RegisterBits HP_SLOPE_XL_EN =
      Adafruit_BusIO_RegisterBits(&ctrl9, 1, 4);
  Adafruit_BusIO_RegisterBits LPF2_XL_EN =
      Adafruit_BusIO_RegisterBits(&ctrl9, 1, 3);

  if(filter_enabled) {
	  HP_SLOPE_XL_EN.write(0);
	  LPF2_XL_EN.write(1);
  } else {
	  LPF2_XL_EN.write(0);
  }
}

/**************************************************************************/
/*!
    @brief Enables the gyro low-pass filter LPF1 (ignored in low-power mode or OIS/EIS mode).
    @param filter_enabled Whether to enable the low-pass filter (see datasheet)
    @param filter The 3-bit valuethat sets the data rate divisor (doesn't have an eaily-nameable set of values)
*/
/**************************************************************************/
void Adafruit_LSM6DSV::gyroLowPassFilter(bool filter_enabled,
                                     uint8_t filter) {
  Adafruit_BusIO_Register ctrl6 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL6);
  Adafruit_BusIO_RegisterBits LPF1_G_BW =
      Adafruit_BusIO_RegisterBits(&ctrl6, 3, 4);
  LPF1_G_BW.write(filter);

  Adafruit_BusIO_Register ctrl7 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL7);
  Adafruit_BusIO_RegisterBits LPF1_G_EN =
      Adafruit_BusIO_RegisterBits(&ctrl7, 1, 0);
  LPF1_G_EN.write(filter_enabled);
}

/**************************************************************************/
/*!
    @brief Configures the user offset (zero calibration) for accelerometer.
	       Does not configure USR_OFF_ON_WU to utilize offset within wakeup logic.
		   Note the offset is set in g, not dependent on the accelerometer range selected.
		   Datasheet page 72 seems to be incorrect attributing the weight to the high-G accel offsets
		   as it is contradicted by page 108 which says their weight depends on the high-G scale.
    @param enabled Whether to enable the offset (if you're turning it off, all other parameters are ignored)
    @param weight Multiplier for the offset 0 = 2^-10 g/LSB 1 = 2^-6 g/LSB
    @param offsetX The offset in the X axis (positive or negative)
    @param offsetY The offset in the Y axis (positive or negative)
    @param offsetZ The offset in the Z axis (positive or negative)
*/
/**************************************************************************/
void Adafruit_LSM6DSV::accelOffset(bool enabled, bool weight, int8_t offsetX, int8_t offsetY, int8_t offsetZ){
  Adafruit_BusIO_Register ctrl9 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL9);
  Adafruit_BusIO_RegisterBits USR_OFF_ON_OUT =
      Adafruit_BusIO_RegisterBits(&ctrl9, 1, 0);
  USR_OFF_ON_OUT.write(enabled);

  if(enabled) {
	  Adafruit_BusIO_RegisterBits USR_OFF_W =
		  Adafruit_BusIO_RegisterBits(&ctrl9, 1, 1);
	  USR_OFF_W.write(weight);

	  Adafruit_BusIO_Register X_OFS_USR = Adafruit_BusIO_Register(
		  i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_X_OFS_USR);
	  X_OFS_USR.write(offsetX);
	  Adafruit_BusIO_Register Y_OFS_USR = Adafruit_BusIO_Register(
		  i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_Y_OFS_USR);
	  Y_OFS_USR.write(offsetY);
	  Adafruit_BusIO_Register Z_OFS_USR = Adafruit_BusIO_Register(
		  i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_Z_OFS_USR);
	  Z_OFS_USR.write(offsetZ);
  }
}

/**************************************************************************/
/*!
    @brief Configures the user offset (zero calibration) for high-G accelerometer.
	       Datasheet page 108 says the weight depends on the high-G scale.
		   Up to 256G, each LSB here represents 0.25g and 0.33g for 320G range.
    @param offsetX The offset in the X axis (positive or negative)
    @param offsetY The offset in the Y axis (positive or negative)
    @param offsetZ The offset in the Z axis (positive or negative)
*/
/**************************************************************************/
void Adafruit_LSM6DSV::highGOffset(int8_t offsetX, int8_t offsetY, int8_t offsetZ){

  Adafruit_BusIO_Register XL_HG_X_OFS_USR = Adafruit_BusIO_Register(
	  i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_X_OFS_USR);
  XL_HG_X_OFS_USR.write(offsetX);
  Adafruit_BusIO_Register XL_HG_Y_OFS_USR = Adafruit_BusIO_Register(
	  i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_Y_OFS_USR);
  XL_HG_Y_OFS_USR.write(offsetY);
  Adafruit_BusIO_Register XL_HG_Z_OFS_USR = Adafruit_BusIO_Register(
	  i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_Z_OFS_USR);
  XL_HG_Z_OFS_USR.write(offsetZ);

}

/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously.
 */
/**************************************************************************/
void Adafruit_LSM6DSV::_readFast(void) {
	_read(); //in this lower class, this is just a placeholder. Will be replaced at higher level if required
}
void Adafruit_LSM6DSV::_read(void) {
  // get raw readings
  Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_OUT_TEMP_L, _high_g_enabled?26:14);

  uint8_t buffer[26];
  data_reg.read(buffer, _high_g_enabled?26:14);

  rawTemp = buffer[1] << 8 | buffer[0];

  rawGyroX = buffer[3] << 8 | buffer[2];
  rawGyroY = buffer[5] << 8 | buffer[4];
  rawGyroZ = buffer[7] << 8 | buffer[6];

  rawAccX = buffer[9] << 8 | buffer[8];
  rawAccY = buffer[11] << 8 | buffer[10];
  rawAccZ = buffer[13] << 8 | buffer[12];

  if(_high_g_enabled) {

	  //skip 6 bytes of OIS data that we're not interested in

	  rawHighGX = buffer[21] << 8 | buffer[20];
	  rawHighGY = buffer[23] << 8 | buffer[22];
	  rawHighGZ = buffer[25] << 8 | buffer[24];

	  highGX = rawHighGX * highGScale;
	  highGY = rawHighGY * highGScale;
	  highGZ = rawHighGZ * highGScale;
  }

  temperature = (rawTemp / temperature_sensitivity) + 25.0;

  gyroX = rawGyroX * gyroScale;
  gyroY = rawGyroY * gyroScale;
  gyroZ = rawGyroZ * gyroScale;

  accX = rawAccX * accelScale;
  accY = rawAccY * accelScale;
  accZ = rawAccZ * accelScale;
}

/**************************************************************************/
/*!
    @brief Sets the INT1 and INT2 pin activation mode.
    @param active_low true to set the pins  as active high, false to set the
   mode to active low
    @param open_drain true to set the pin mode as open-drain, false to set the
   mode to push-pull
    @param drive_strength set to 3 (default) for VDDIO < 2V (strongest), 
   set to 1 for VDDIO between 2 and 3 volts
   set to 0  for VDDIO >= 3V (weakest)
*/
void Adafruit_LSM6DSV::configIntOutputs(bool active_low, bool open_drain, uint8_t drive_strength) {

  Adafruit_BusIO_Register ifcfg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_IF_CFG);
  Adafruit_BusIO_RegisterBits ppod_bits =
      Adafruit_BusIO_RegisterBits(&ifcfg, 2, 3);

  ppod_bits.write((active_low << 1) | open_drain);

  Adafruit_BusIO_Register pinctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_PIN_CTRL);
  Adafruit_BusIO_RegisterBits ds_bits =
      Adafruit_BusIO_RegisterBits(&pinctrl, 2, 0);

  ds_bits.write(drive_strength);

}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 1.
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt

	The DSV chips don't have an interrupt for temperature
	and the "dual channel" chips don't have an interrupt for high-g since that's
	the same data as the main accelerometer
*/
void Adafruit_LSM6DSV::configInt1(bool drdy_g, bool drdy_xl) {

  Adafruit_BusIO_Register int1_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_INT1_CTRL);

  Adafruit_BusIO_RegisterBits int1_bits =
      Adafruit_BusIO_RegisterBits(&int1_ctrl, 2, 0);

  int1_bits.write((drdy_g << 1) | drdy_xl);

  //step detect is now in EMB_FUNC_INT1 - haven't enabled pedometer module yet

  //Wakeup INT1 interrupt is on MD1_CFG - routes lots of functions to that pin
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 2.
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
*/
void Adafruit_LSM6DSV::configInt2(bool drdy_g, bool drdy_xl) {

  Adafruit_BusIO_Register int2_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_INT2_CTRL);

  Adafruit_BusIO_RegisterBits int2_drdy_bits =
      Adafruit_BusIO_RegisterBits(&int2_ctrl, 2, 0);

  int2_drdy_bits.write((drdy_g << 1) | drdy_xl);
}

/**************************************************************************/
/*!
    @brief Enables and disables the high-g data outputs
    @param enable
*/
void Adafruit_LSM6DSV::enableHighG(bool enable) {

  Adafruit_BusIO_Register ctrl8 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL8);

  Adafruit_BusIO_RegisterBits XL_DualC_EN =
      Adafruit_BusIO_RegisterBits(&ctrl8, 1, 3);

  XL_DualC_EN.write(enable);

  _high_g_enabled = enable;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the LSM6DS's gyroscope sensor (radians/sec)
*/
/**************************************************************************/
void Adafruit_LSM6DSV_Gyro::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LSM6DSV_G", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _theLSM6DSV->_sensorid_gyro;
  sensor->type = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay = 0;
  sensor->min_value = -32767 * SENSORS_DPS_TO_RADS * _theLSM6DSV->gyroScale;
  sensor->max_value = 32767 * SENSORS_DPS_TO_RADS * _theLSM6DSV->gyroScale;
  sensor->resolution = SENSORS_DPS_TO_RADS * _theLSM6DSV->gyroScale;
}

/**************************************************************************/
/*!
    @brief  Gets the gyroscope as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_LSM6DSV_Gyro::getEvent(sensors_event_t *event) {
  _theLSM6DSV->_read();
  _theLSM6DSV->fillGyroEvent(event, millis());

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the LSM6DS's accelerometer
*/
/**************************************************************************/
void Adafruit_LSM6DSV_Accelerometer::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LSM6DSV_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _theLSM6DSV->_sensorid_accel;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->min_value = -32767 * _theLSM6DSV->accelScale;
  sensor->max_value = 32767 * _theLSM6DSV->accelScale;
  sensor->resolution = _theLSM6DSV->accelScale;
}

/**************************************************************************/
/*!
    @brief  Gets the accelerometer as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_LSM6DSV_Accelerometer::getEvent(sensors_event_t *event) {
  _theLSM6DSV->_read();
  _theLSM6DSV->fillAccelEvent(event, millis());

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the LSM6DSV's high-g accelerometer
*/
/**************************************************************************/
void Adafruit_LSM6DSV_HighG::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "DSV_HighG", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _theLSM6DSV->_sensorid_high_g;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0; //minimum time between events, can we insert the output dta rate here later?
  sensor->min_value = -32767 * _theLSM6DSV->highGScale; 
  sensor->max_value = 32767 * _theLSM6DSV->highGScale; 
  sensor->resolution = _theLSM6DSV->highGScale; 
}

/**************************************************************************/
/*!
    @brief  Gets the high-g accelerometer as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_LSM6DSV_HighG::getEvent(sensors_event_t *event) {
  _theLSM6DSV->_read();
  _theLSM6DSV->fillAccelEvent(event, millis());

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the LSM6DS's tenperature
*/
/**************************************************************************/
void Adafruit_LSM6DSV_Temp::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LSM6DSV_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _theLSM6DSV->_sensorid_temp;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40;
  sensor->max_value = 85;
  sensor->resolution = 1/_theLSM6DSV->temperature_sensitivity;
}

/**************************************************************************/
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_LSM6DSV_Temp::getEvent(sensors_event_t *event) {
  _theLSM6DSV->_read();
  _theLSM6DSV->fillTempEvent(event, millis());

  return true;
}

/**************************************************************************/
/*!
    @brief Gets the temperature data rate.
    @returns The data rate in float
	It's not clear from the datasheet, it says it's fixed at 60Hz
	but then it also says "if gyro is off and accel is in low-power
	then temperature rate (TODR) is equal to the accelerometer"
*/
float Adafruit_LSM6DSV::temperatureSampleRate(void) {
  return 60;
}

/**************************************************************************/
/*!
    @brief Check for available data from thermometer
    @returns 1 if available, 0 if not
*/
int Adafruit_LSM6DSV::temperatureAvailable(void) {
  return (this->status() & 0b100) ? 1 : 0;
}

/**************************************************************************/
/*!
    @brief Read thermometer data, degrees C
    @param tempC reference to temperature
    @returns 1 if success, 0 if not
*/
int Adafruit_LSM6DSV::readTemperature(float &tempC) {
  int16_t data[1];

  Adafruit_BusIO_Register temp_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_OUT_TEMP_L, 2);

  if (!temp_data.read((uint8_t *)data, sizeof(data))) {
    tempC = NAN;
    return 0;
  }

  rawTemp = data[0];

  // scale to metres/sec/sec
  tempC = temperature = data[0] * temperature_sensitivity + 25.0;

  return 1;
}

/**************************************************************************/
/*!
    @brief Enable or disable access to the overlapping registers
	       May only turn on one at a time.
		   Functions using these should set this back to default afterwards
    @param embFunc enable access to the embedded functions
    @param sHub enable access to the sensor hub configuration
	@param FSM enable access ot the finite state machine control registers
	@param globalReset perform a power-on reset
*/
void Adafruit_LSM6DSV::funcCfgAccess(bool embFunc, bool sHub, bool FSM, bool globalReset) {
  Adafruit_BusIO_Register funcCFG = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FUNC_CFG_ACCESS);

  //bit positions
  const int EMB_FUNC_REG_ACCESS = 7;
  const int SHUB_REG_ACCESS = 6;
  const int FSM_WR_CTRL_EN = 3;
  const int SW_POR = 2;

  uint32_t val = 0; //only the first 8 bits of this value will be written
  if(embFunc) bitSet(val, EMB_FUNC_REG_ACCESS);
  if(sHub) bitSet(val, SHUB_REG_ACCESS);
  if(FSM) bitSet(val, FSM_WR_CTRL_EN);
  if(globalReset) bitSet(val, SW_POR);

  funcCFG.write(val);
}

/**************************************************************************/
/*!
    @brief Enable or disable sensor fusion low power (SFLP)
	       You should probably set the accelerometer user offsets for this to work best
    @param enable true/false
*/
void Adafruit_LSM6DSV::enableSFLP(bool enable) {
  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register funcEnA = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_EMB_FUNC_EN_A);

  Adafruit_BusIO_RegisterBits en_bits =
      Adafruit_BusIO_RegisterBits(&funcEnA, 1, 1);

  en_bits.write(enable);

  //last, reset the config functions access to default state
  funcCfgAccess();
}


/**************************************************************************/
/*!
    @brief Gets the SFLP data rate.
    @returns The the SFLP data rate.
*/
lsm6dsv_sflp_data_rate_t Adafruit_LSM6DSV::getSFLPDataRate(void) {
  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_SFLP_ODR);

  Adafruit_BusIO_RegisterBits sflp_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 3, 3);

  uint8_t val = sflp_data_rate.read();

  //reset the config functions access to default state
  funcCfgAccess();

  return (lsm6dsv_sflp_data_rate_t)val;
}

/**************************************************************************/
/*!
    @brief Sets the SFLP data rate.
    @param  data_rate Must be a `lsm6dsv_sflp_data_rate_t`.
*/
void Adafruit_LSM6DSV::setSFLPDataRate(lsm6dsv_sflp_data_rate_t data_rate) {
  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register sflp_odr = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_SFLP_ODR);

  Adafruit_BusIO_RegisterBits sflp_data_rate =
      Adafruit_BusIO_RegisterBits(&sflp_odr, 3, 3);

  sflp_data_rate.write(data_rate);

  //last, reset the config functions access to default state
  funcCfgAccess();
}

/*
Further inputs to the SFLP algorithm:
Looks like you can give it a clue for the starting value of the gyro bias (16-bit floating point)
There's also an "initialization request" bit that may be set - no description on what this does, must be stationary?
It can use accelerometer offsets if you've calibrated the accelerometer to add up to +/-127 to the raw accel readings
before applying the SFLP algorithm. Let's see how it goes without that.

Outputs:
Gyro bias 16 bit integer with 4.375 milli degrees/sec per LSB
Gravity vector X/Y/Z 16 bit with 0.061 mg/LSB
Game rotation vector, as a quaternion W/X/Y/Z, (16-bit floating point) SEEEEEFFFFFFFFFF (sign/exponent/fraction)
use the float16 library to work with that data.

*/

/**************************************************************************/
/*!
    @brief Read SFLP gravity data, meters/sec/sec
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int Adafruit_LSM6DSV::readSFLPGravity(float &x, float &y, float &z) {
  int16_t data[3];

  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register accel_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_SFLP_GRAVX_L, sizeof(data));

  if (!accel_data.read((uint8_t *)data, sizeof(data))) {
    x = y = z = NAN;
	funcCfgAccess();
    return 0;
  }

  // scale to metres/sec/sec - note the SFLP is fixed at the "2G" scale for all variants, all the time
  // and we don't store the calculated value as a side-effect like we do with the main measurements
  x = data[0] * 0.061 * SENSORS_GRAVITY_STANDARD / 1000;
  y = data[1] * 0.061 * SENSORS_GRAVITY_STANDARD / 1000;
  z = data[2] * 0.061 * SENSORS_GRAVITY_STANDARD / 1000;

  funcCfgAccess();
  return 1; //success
}


/**************************************************************************/
/*!
    @brief Read SFLP gyro bias data, degrees/sec
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int Adafruit_LSM6DSV::readSFLPGyroBias(float &x, float &y, float &z) {
  int16_t data[3];

  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register accel_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_SFLP_GBIASX_L, sizeof(data));

  if (!accel_data.read((uint8_t *)data, sizeof(data))) {
    x = y = z = NAN;
	funcCfgAccess();
    return 0;
  }

  // scale to degrees/sec - note the SFLP has a fixed scale for all variants, all the time
  // and we don't store the calculated value as a side-effect like we do with the main measurements
  x = data[0] * 4.375 / 1000;
  y = data[1] * 4.375 / 1000;
  z = data[2] * 4.375 / 1000;

  funcCfgAccess();
  return 1; //success
}


/**************************************************************************/
/*!
    @brief Read SFLP orientation quaternion (float16 or just an int16_t if you don't have <float16.h>)
    @param w reference to w axis
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int Adafruit_LSM6DSV::readSFLPQuaternion(float16 &w, float16 &x, float16 &y, float16 &z) {
  int16_t data[4];

  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register accel_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_SFLP_QUATW_L, sizeof(data));

  if (!accel_data.read((uint8_t *)data, sizeof(data))) {
    w = x = y = z = 0;
	funcCfgAccess();
    return 0;
  }

  // No scale for quaternion, but if we have float16 support, we'll need to use its conversion function
#ifdef FLOAT16_LIB_VERSION
  w.setBinary(data[0]);
  x.setBinary(data[1]);
  y.setBinary(data[2]);
  z.setBinary(data[3]);
#else
  w = data[0];
  x = data[1];
  y = data[2];
  z = data[3];
#endif

  funcCfgAccess();
  return 1; //success
}


/**************************************************************************/
/*!
    @brief Gets the FSM data rate.
    @returns The the FSM output data rate.
*/
lsm6dsv_fsm_data_rate_t Adafruit_LSM6DSV::getFSMDataRate(void) {
  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register fsm_odr = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FSM_ODR);

  Adafruit_BusIO_RegisterBits fsm_data_rate =
      Adafruit_BusIO_RegisterBits(&fsm_odr, 3, 3);

  uint8_t val = fsm_data_rate.read();

  //reset the config functions access to default state
  funcCfgAccess();

  return (lsm6dsv_fsm_data_rate_t)val;
}

/**************************************************************************/
/*!
    @brief Sets the FSM data rate.
    @param  data_rate Must be a `lsm6dsv_fsm_data_rate_t`.
*/
void Adafruit_LSM6DSV::setFSMDataRate(lsm6dsv_fsm_data_rate_t data_rate) {
  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register fsm_odr = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FSM_ODR);

  Adafruit_BusIO_RegisterBits fsm_data_rate =
      Adafruit_BusIO_RegisterBits(&fsm_odr, 3, 3);

  fsm_data_rate.write(data_rate);

  //last, reset the config functions access to default state
  funcCfgAccess();
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer data rate, adjusted by FREQ_FINE.
    @returns The data rate in Hz
*/
float Adafruit_LSM6DSV::accelerationSampleRate(void) {
  int8_t fine = getInternalFreqFine();
  float freqFineAdjust = 1 + 0.0013 * fine;
  return _data_rate_arr[this->getAccelDataRate()] * freqFineAdjust;
}

/**************************************************************************/
/*!
    @brief Check for available data from accelerometer
    @returns 1 if available, 0 if not
*/
int Adafruit_LSM6DSV::accelerationAvailable(void) {
  return (this->status() & 0x01) ? 1 : 0;
}

/**************************************************************************/
/*!
    @brief Read accelerometer data, meters/sec/sec
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int Adafruit_LSM6DSV::readAcceleration(float &x, float &y, float &z) {
  int16_t data[3];

  Adafruit_BusIO_Register accel_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_OUTX_L_A, 6);

  if (!accel_data.read((uint8_t *)data, sizeof(data))) {
    x = y = z = NAN;
    return 0;
  }

  rawAccX = data[0];
  rawAccY = data[1];
  rawAccZ = data[2];

  // scale to metres/sec/sec
  x = accX = data[0] * accelScale;
  y = accY = data[1] * accelScale;
  z = accZ = data[2] * accelScale;

  return 1;
}

/**************************************************************************/
/*!
    @brief Get the gyroscope data rate, adjusted by FREQ_FINE.
    @returns The data rate in Hz
*/
float Adafruit_LSM6DSV::gyroscopeSampleRate(void) {
  int8_t fine = getInternalFreqFine();
  float freqFineAdjust = 1 + 0.0013 * fine;
  return _data_rate_arr[this->getGyroDataRate()] * freqFineAdjust;
}

/**************************************************************************/
/*!
    @brief Check for available data from gyroscope
    @returns 1 if available, 0 if not
*/
int Adafruit_LSM6DSV::gyroscopeAvailable(void) {
  return (this->status() & 0b10) ? 1 : 0;
}

/**************************************************************************/
/*!
    @brief Read gyroscope data in degrees/second
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int Adafruit_LSM6DSV::readGyroscope(float &x, float &y, float &z) {
  int16_t data[3];

  Adafruit_BusIO_Register gyro_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_OUTX_L_G, 6);

  if (!gyro_data.read((uint8_t *)data, sizeof(data))) {
    x = y = z = NAN;
    return 0;
  }

  rawGyroX = data[0];
  rawGyroY = data[1];
  rawGyroZ = data[2];

  // scale to degrees/sec
  x = gyroX = data[0] * gyroScale;
  y = gyroY = data[1] * gyroScale;
  z = gyroZ = data[2] * gyroScale;

  return 1;
}


/**************************************************************************/
/*!
    @brief Gets the high-g accelerometer data rate, adjusted by FREQ_FINE.
    @returns The data rate in float
*/
float Adafruit_LSM6DSV::highGSampleRate(void) {
  int8_t fine = getInternalFreqFine();
  float freqFineAdjust = 1 + 0.0013 * fine;
  return _data_rate_arr[this->getAccelDataRate()] * freqFineAdjust;
}

/**************************************************************************/
/*!
    @brief Check for available data from high-G accelerometer (not useful on chips without independent high-G)
    @returns 1 
*/
int Adafruit_LSM6DSV::highGAvailable(void) {
  //return (this->status() & 0b1000) ? 1 : 0;
  return 1;
}

/**************************************************************************/
/*!
    @brief Read high-g accelerometer data
    @param x reference to x axis
    @param y reference to y axis
    @param z reference to z axis
    @returns 1 if success, 0 if not
*/
int Adafruit_LSM6DSV::readHighG(float &x, float &y, float &z) {
  int16_t data[3];

  if(!_high_g_enabled) {
    x = y = z = NAN;
    return 0;
  }

  Adafruit_BusIO_Register accel_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_OUTX_L_HG, 6);

  if (!accel_data.read((uint8_t *)data, sizeof(data))) {
    x = y = z = NAN;
    return 0;
  }

  rawHighGX = data[0];
  rawHighGY = data[1];
  rawHighGZ = data[2];

  // scale to meters/sec/sec
  x = highGX = data[0] * highGScale;
  y = highGY = data[1] * highGScale;
  z = highGZ = data[2] * highGScale;

  return 1;
}


/****************************************************************
  F I F O
****************************************************************/

/**************************************************************************/
/*!
    @brief Read Fifo status byte
    @returns 8-bits of status: watermark reached, overrun, full
*/
uint8_t Adafruit_LSM6DSV::getFifoStatus() {

  Adafruit_BusIO_Register status2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_STATUS2);

  return status2.read();
}

/**************************************************************************/
/*!
    @brief Fifo data available
    @returns Number of entries available to read
*/
int Adafruit_LSM6DSV::getFifoAvailable() { 

  Adafruit_BusIO_Register status1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_STATUS1);
  Adafruit_BusIO_Register status2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_STATUS2);
  Adafruit_BusIO_RegisterBits highBit =
      Adafruit_BusIO_RegisterBits(&status2, 1, 0);

  int out = status1.read(); //get the lower bits
  if(highBit.read()) out += 256; //8th bit is in status2 register
  
  return out;
}

/**************************************************************************/
/*!
    @brief Read one item from the Fifo
    @param data pointer to buffer at least 7 bytes wide
    @returns true if read
*/
bool Adafruit_LSM6DSV::getFifo(uint8_t *data) {
  Adafruit_BusIO_Register fifo_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_DATA_OUT_TAG, 7);

  return fifo_data.read(data, 7);
}


/**************************************************************************/
/*!
    @brief Configure interrupt pin 1 fifo options
    @param threshold true for interrupt on watermark threshold reached
    @param overflow true for interrupt on overflow of entire Fifo
    @param full true if the next data item is going to fill the Fifo
    @param batch true if the counter_BDR threshold reached
*/
void Adafruit_LSM6DSV::configFifoInt1(bool threshold, bool overflow, bool full, bool batch) {
	//threshold interrupt when there are M sensor readings in the fifo, 
	//batch interrupt when there are N readings of one sensor out of accel/gyro/EIS gyro/High-G accel, not counting the other data in the fifo
  Adafruit_BusIO_Register int1_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_INT1_CTRL);

  Adafruit_BusIO_RegisterBits cnt_bdr =
      Adafruit_BusIO_RegisterBits(&int1_ctrl, 1, 6);
  Adafruit_BusIO_RegisterBits fifo_full =
      Adafruit_BusIO_RegisterBits(&int1_ctrl, 1, 5);
  Adafruit_BusIO_RegisterBits fifo_ovr =
      Adafruit_BusIO_RegisterBits(&int1_ctrl, 1, 4);
  Adafruit_BusIO_RegisterBits fifo_th =
      Adafruit_BusIO_RegisterBits(&int1_ctrl, 1, 3);

  cnt_bdr.write(batch);
  fifo_full.write(full);
  fifo_ovr.write(overflow);
  fifo_th.write(threshold);
}

/**************************************************************************/
/*!
    @brief Configure interrupt pin 2 fifo options
    @param threshold true for interrupt on watermark threshold reached
    @param overflow true for interrupt on overflow of entire Fifo
    @param full true if the next data item is going to fill the Fifo
    @param batch true if the counter_BDR threshold reached
*/
void Adafruit_LSM6DSV::configFifoInt2(bool threshold, bool overflow, bool full, bool batch) {
	//threshold interrupt when there are M sensor readings in the fifo, 
	//batch interrupt when there are N readings of one sensor out of accel/gyro/EIS gyro/High-G accel, not counting the other data in the fifo
  Adafruit_BusIO_Register int2_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_INT2_CTRL);

  Adafruit_BusIO_RegisterBits cnt_bdr =
      Adafruit_BusIO_RegisterBits(&int2_ctrl, 1, 6);
  Adafruit_BusIO_RegisterBits fifo_full =
      Adafruit_BusIO_RegisterBits(&int2_ctrl, 1, 5);
  Adafruit_BusIO_RegisterBits fifo_ovr =
      Adafruit_BusIO_RegisterBits(&int2_ctrl, 1, 4);
  Adafruit_BusIO_RegisterBits fifo_th =
      Adafruit_BusIO_RegisterBits(&int2_ctrl, 1, 3);

  cnt_bdr.write(batch);
  fifo_full.write(full);
  fifo_ovr.write(overflow);
  fifo_th.write(threshold);
}

/**************************************************************************/
/*!
    @brief Set the rate of accelerometer data recorded to the Fifo
    @param  data_rate
            The the accelerometer data rate. Must be a `lsm6dsv_data_rate_t`.
*/
void Adafruit_LSM6DSV::setFifoAccelRate(lsm6dsv_data_rate_t data_rate) {
  Adafruit_BusIO_Register fifo_ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_CTRL3);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&fifo_ctrl3, 4, 0);

  accel_data_rate.write(data_rate);
}


/**************************************************************************/
/*!
    @brief Set the rate of gyro data recorded to the Fifo
    @param  data_rate
            The the gyro data rate. Must be a `lsm6dsv_data_rate_t`.
*/
void Adafruit_LSM6DSV::setFifoGyroRate(lsm6dsv_data_rate_t data_rate) {
  Adafruit_BusIO_Register fifo_ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_CTRL3);

  Adafruit_BusIO_RegisterBits gyro_data_rate =
      Adafruit_BusIO_RegisterBits(&fifo_ctrl3, 4, 4);

  gyro_data_rate.write(data_rate);
}


/**************************************************************************/
/*!
    @brief Set the rate of temperature data recorded to the Fifo
    @param  data_rate
            The the temperature data rate. Must be one of the first 4 `lsm6dsv_data_rate_t` (0 to 60Hz).
*/
void Adafruit_LSM6DSV::setFifoTempRate(lsm6dsv_data_rate_t data_rate) {
  Adafruit_BusIO_Register fifo_ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_CTRL4);

  Adafruit_BusIO_RegisterBits temp_data_rate =
      Adafruit_BusIO_RegisterBits(&fifo_ctrl4, 2, 4);

  temp_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Set the rate of timestamps data recorded to the Fifo
			Side effect: always sets the timestamp register to running state
			(each click is 21.7us plus or minus the freq_fine adjustment)
    @param  dec
            The the decimation factor: 0 (do not record) 1, 8, 32
*/
void Adafruit_LSM6DSV::setFifoTSDec(lsm6_fifo_dec_ts_t dec) {
  Adafruit_BusIO_Register fifo_ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_CTRL4);

  Adafruit_BusIO_RegisterBits dec_ts =
      Adafruit_BusIO_RegisterBits(&fifo_ctrl4, 2, 6);

  dec_ts.write(dec);

  //turn on the timestamp counter 

  Adafruit_BusIO_Register func_en = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FUNCTIONS_ENABLE);

  Adafruit_BusIO_RegisterBits timestamp_en =
      Adafruit_BusIO_RegisterBits(&func_en, 1, 6);

  timestamp_en.write(1);
}

/**************************************************************************/
/*!
    @brief Set the fifo recording of EIS gyro data 
		(not in use as no other EIS functions available)
		(only included for complete coverage of this register)
    @param  en true to enable
*/
void Adafruit_LSM6DSV::setFifoGyroEISRate(bool en) {
  Adafruit_BusIO_Register fifo_ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_CTRL4);

  Adafruit_BusIO_RegisterBits eis =
      Adafruit_BusIO_RegisterBits(&fifo_ctrl4, 1, 3);

  eis.write(en);
}

/**************************************************************************/
/*!
    @brief Set the fifo recording of high-g accelerometer 
		(experimental - not sure if this is the real control for this)
    @param  en true to enable
*/
void Adafruit_LSM6DSV::setFifoHighGRate(bool en) {
  Adafruit_BusIO_Register reg1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_COUNTER_BDR_REG1);

  Adafruit_BusIO_RegisterBits xl_hg_batch_en =
      Adafruit_BusIO_RegisterBits(&reg1, 1, 3);

  xl_hg_batch_en.write(en);
}


/**************************************************************************/
/*!
    @brief Set the fifo mode
    @param  mode
            The the mode 0b000 - 0b111
*/
void Adafruit_LSM6DSV::setFifoMode(lsm6_fifo_mode_t mode) {
  Adafruit_BusIO_Register fifo_ctrl4 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_CTRL4);

  Adafruit_BusIO_RegisterBits fifo_mode =
      Adafruit_BusIO_RegisterBits(&fifo_ctrl4, 3, 0);

  fifo_mode.write(mode);
}

/**************************************************************************/
/*!
    @brief Set the fifo watermark threshold level
    @param  wtm Number of fifo items to trigger watermark interrupt
*/
void Adafruit_LSM6DSV::setFifoWatermark(uint8_t wtm) {
  Adafruit_BusIO_Register fifo_ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_FIFO_CTRL1);

  fifo_ctrl1.write(wtm);
}


/**************************************************************************/
/*!
    @brief Set the batch data rate for the fifo
    @param  trig The triggering sensor to count for the batch
	@param threshold The number of batches to raise the interrupt (max 1023)
*/
void Adafruit_LSM6DSV::setFifoCounterBDR(lsm6_fifo_batch_trig_t trig, uint16_t threshold) {
  Adafruit_BusIO_Register reg1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_COUNTER_BDR_REG1);
  Adafruit_BusIO_Register reg2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_COUNTER_BDR_REG2);

  Adafruit_BusIO_RegisterBits trig_counter_bdr =
      Adafruit_BusIO_RegisterBits(&reg1, 2, 5);

  trig_counter_bdr.write(trig);

  Adafruit_BusIO_RegisterBits high_bits =
      Adafruit_BusIO_RegisterBits(&reg1, 2, 0);

  high_bits.write(threshold >> 8);
  reg2.write(threshold & 0xFF);

}

/**************************************************************************/
/*!
    @brief Configure embedded functions going into Fifo
    @param sflpGame True to record game vector
    @param sflpGravity True to record gravity vector
    @param sflpGBias True to record gyro bias
    @param stepCounter True to record pedometer step counter
	@param mlc True to record Machine Learning Core results
*/
void Adafruit_LSM6DSV::setFifoEmbFunc(bool sflpGame, bool sflpGravity, bool sflpGBias, bool stepCounter, bool mlc) {
  //first tell it that we will be talking to the embedded functions registers
  funcCfgAccess(true);

  Adafruit_BusIO_Register fifo_en_a = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_EMB_FUNC_FIFO_EN_A);

  Adafruit_BusIO_RegisterBits game_en =
      Adafruit_BusIO_RegisterBits(&fifo_en_a, 1, 1);
  Adafruit_BusIO_RegisterBits gravity_en =
      Adafruit_BusIO_RegisterBits(&fifo_en_a, 1, 4);
  Adafruit_BusIO_RegisterBits gbias_en =
      Adafruit_BusIO_RegisterBits(&fifo_en_a, 1, 5);
  Adafruit_BusIO_RegisterBits step_en =
      Adafruit_BusIO_RegisterBits(&fifo_en_a, 1, 6);
  Adafruit_BusIO_RegisterBits mlc_en =
      Adafruit_BusIO_RegisterBits(&fifo_en_a, 1, 7);

  game_en.write(sflpGame);
  gravity_en.write(sflpGravity);
  gbias_en.write(sflpGBias);
  step_en.write(stepCounter);
  mlc_en.write(mlc);

  //last, reset the config functions access to default state
  funcCfgAccess();
}

/****************************************************************
  end  F I F O
****************************************************************/
