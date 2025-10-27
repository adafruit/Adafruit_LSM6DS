
/*!
 *  @file Adafruit_LSM6DSV320.cpp
 *  Adafruit LSM6DSV320 9-DoF Accelerometer and Gyroscope and high-G
 * accelerometer library
 *
 *  @section intro_sec Introduction
 *  Sub-class of Adafruit_LSM6DSV that extends the functions for the V320 and
 * V80 chips
 *
 *  Aside: magnetic sensors from ST
 *      This sensor is likely going to be paired with a 3-axis compass to make a
 * 9-dof sensor. The best ST part for an Earth compass used to be the LIS3MDL
 * but that seems to be discontinued. The current LIS2MDL is less sensitive but
 * still adequate as a compass. The benefit seems to be that it can withstand
 * 10x the magnetic field (maybe from a carelessly-handled magnet) without
 * damage. The disadvantage is the LIS3 had a maximum G-shock rating of 10,000g
 *      [compared to the LSM6 chips rated for 20,000g] but the LIS2 has no
 * maximum G-shock and the data sheet warns "improper handling may cause
 * permanent damage to the part." So we want to build a high-G sensor with
 * compass, we may damage the compass.
 *
 *  @section author Author
 *
 *  Morgan Sandercock for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_LSM6DSV320.h"

// Data rates unique to the high-g accelerometer
static const float _V320_data_rate_arr[] = {
    [LSM6DSV320_HGRATE_SHUTDOWN] = 0.0f,
    [LSM6DSV320_HGRATE_NA1] = 0.0f,
    [LSM6DSV320_HGRATE_NA2] = 0.0f,
    [LSM6DSV320_HGRATE_480_HZ] = 480.0f,
    [LSM6DSV320_HGRATE_960_HZ] = 960.0f,
    [LSM6DSV320_HGRATE_1_92K_HZ] = 1920.0f,
    [LSM6DSV320_HGRATE_3_84K_HZ] = 3840.0f,
    [LSM6DSV320_HGRATE_7_68K_HZ] = 7680.0f,
};

/*!
 *    @brief  Instantiates a new LSM6DSV320 class
 */
Adafruit_LSM6DSV320::Adafruit_LSM6DSV320(void) {}

/*!
 *    @brief  Destructor required because base class declares it as virtual
 */
Adafruit_LSM6DSV320::~Adafruit_LSM6DSV320(void) {}

bool Adafruit_LSM6DSV320::_init(int32_t sensor_id) {
  // call base class _init() - ignore its return value as it will be looking for
  // the wrong chip ID
  Adafruit_LSM6DSV::_init(sensor_id);

  // now set up the high-g sensor
  setHighGDataRate(LSM6DSV320_HGRATE_480_HZ); // doesn't go any slower than this
  setHighGRange(
      LSM6DSV320_ACCEL_RANGE_128_G); // Maximum common to both V320 and V80

  return (chipID() ==
          LSM6DSV320_CHIP_ID); // identical chip ID for both V320 and V80
}

/**************************************************************************/
/*!
    @brief Gets the high-G data rate.
    @returns The the high-G data rate.
*/
lsm6dsv320_high_g_rate_t Adafruit_LSM6DSV320::getHighGDataRate(void) {

  Adafruit_BusIO_Register ctrl1HG = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL1_XL_HG);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1HG, 3, 3);

  return (lsm6dsv320_high_g_rate_t)accel_data_rate.read();
}

/**************************************************************************/
/*!
    @brief Sets the high-G data rate.
    @param  data_rate
            The the high-G data rate. Must be a valid lsm6dsv320_high_g_rate_t.
*/
void Adafruit_LSM6DSV320::setHighGDataRate(lsm6dsv320_high_g_rate_t data_rate) {

  Adafruit_BusIO_Register ctrl1HG = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL1_XL_HG);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1HG, 3, 3);

  accel_data_rate.write(data_rate);

  _high_g_enabled =
      (data_rate != LSM6DSV320_HGRATE_SHUTDOWN); // don't bother reading the
                                                 // output registers if it's off
}

/**************************************************************************/
/*!
    @brief Gets the high-G measurement range.
    @returns The the high-G measurement range.
*/
lsm6dsv320_accel_range_t Adafruit_LSM6DSV320::getHighGRange(void) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL1_XL_HG);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 3, 0);

  accelHighGBuffered = (lsm6dsv320_accel_range_t)accel_range.read();

  return accelHighGBuffered;
}
/**************************************************************************/
/*!
    @brief Sets the high-G measurement range.
    @param new_range The `lsm6dsv320_accel_range_t` range to set.
*/
void Adafruit_LSM6DSV320::setHighGRange(lsm6dsv320_accel_range_t new_range) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL1_XL_HG);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 3, 0);

  accel_range.write(new_range);

  accelHighGBuffered = new_range;

  setHighGScale();
}

/**************************************************************************/
/*!
    @brief Sets the high-g scale from the "buffered" range enum.
*/
void Adafruit_LSM6DSV320::setHighGScale() {

  // magic numbers from the datasheet are milli-gee per LSB, convert to m/s
  switch (accelHighGBuffered) {
  case LSM6DSV320_ACCEL_RANGE_32_G:
    highGScale = 0.976 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  case LSM6DSV320_ACCEL_RANGE_64_G:
    highGScale = 1.952 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  case LSM6DSV320_ACCEL_RANGE_128_G:
    highGScale = 3.904 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  case LSM6DSV320_ACCEL_RANGE_256_G:
    highGScale = 7.808 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  case LSM6DSV320_ACCEL_RANGE_320_G:
    highGScale = 10.417 * SENSORS_GRAVITY_STANDARD / 1000;
    break;
  }
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 1.
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
    @param drdy_hg true to output the data ready high-g interrupt
*/
void Adafruit_LSM6DSV320::configInt1(bool drdy_g, bool drdy_xl, bool drdy_hg) {

  // call the base class to do the regular accel and gyro on INT1_CTRL
  Adafruit_LSM6DSV::configInt1(drdy_g, drdy_xl);

  // high-g interrupt pin control is in CTRL7
  Adafruit_BusIO_Register ctrl7 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL7);

  Adafruit_BusIO_RegisterBits INT1_DRDY_XL_HG =
      Adafruit_BusIO_RegisterBits(&ctrl7, 1, 7);

  INT1_DRDY_XL_HG.write(drdy_hg);
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 2.
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
    @param drdy_hg true to output the data ready high-g interrupt
*/
void Adafruit_LSM6DSV320::configInt2(bool drdy_g, bool drdy_xl, bool drdy_hg) {

  // call the base class to do the regular accel and gyro on INT2_CTRL
  Adafruit_LSM6DSV::configInt2(drdy_g, drdy_xl);

  // high-g interrupt pin control is in CTRL7
  Adafruit_BusIO_Register ctrl7 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL7);

  Adafruit_BusIO_RegisterBits INT2_DRDY_XL_HG =
      Adafruit_BusIO_RegisterBits(&ctrl7, 1, 6);

  INT2_DRDY_XL_HG.write(drdy_hg);
}

/**************************************************************************/
/*!
    @brief Enables or disable the high-G sensor output registerss.
               Note that "power down" for the sensor is set by the data rate 000
                   this is just directing the output to the OIS registers.
    @param en True to enable high-G
*/
void Adafruit_LSM6DSV320::enableHighG(bool en) {

  Adafruit_BusIO_Register CTRL1_XL_HG = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSV_CTRL1_XL_HG);

  Adafruit_BusIO_RegisterBits XL_HG_REGOUT_EN =
      Adafruit_BusIO_RegisterBits(&CTRL1_XL_HG, 1, 7);

  XL_HG_REGOUT_EN.write(en);

  _high_g_enabled = en;
}

/**************************************************************************/
/*!
    @brief Gets the high-g accelerometer data rate, as adjusted by FREQ_FINE.
    @returns The data rate in Hz
*/
float Adafruit_LSM6DSV320::highGSampleRate(void) {
  int8_t fine = getInternalFreqFine();
  float freqFineAdjust = 1 + 0.0013 * fine;
  return _V320_data_rate_arr[this->getHighGDataRate()] * freqFineAdjust;
}

/**************************************************************************/
/*!
    @brief Check for available data from high-G accelerometer
    @returns 1 if available
*/
int Adafruit_LSM6DSV320::highGAvailable(void) {
  return (this->status() & 0b1000) ? 1 : 0;
}