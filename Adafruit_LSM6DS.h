/*!
 *  @file Adafruit_LSM6DS.h
 *
 * 	I2C Driver base for Adafruit LSM6DSxx 6-DoF Accelerometer and Gyroscope
 *      library
 *
 * 	Adafruit invests time and resources providing this open source code,
 *      please support Adafruit and open-source hardware by purchasing products
 *from Adafruit!
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DS_H
#define _ADAFRUIT_LSM6DS_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LSM6DS_I2CADDR_DEFAULT 0x6A ///< LSM6DS default i2c address

#define LSM6DS_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions register
#define LSM6DS_INT1_CTRL 0x0D      ///< Interrupt control for INT 1
#define LSM6DS_INT2_CTRL 0x0E      ///< Interrupt control for INT 2
#define LSM6DS_WHOAMI 0xF          ///< Chip ID register
#define LSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define LSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define LSM6DS_CTRL3_C 0x12        ///< Main configuration register
#define LSM6DS_OUT_TEMP_L 0x20     ///< First data register (temperature low)
#define LSM6DS_OUTX_L_G 0x22       ///< First gyro data register
#define LSM6DS_OUTX_L_A 0x28       ///< First accel data register

/** The accelerometer data rate */
typedef enum data_rate {
  LSM6DS_RATE_SHUTDOWN,
  LSM6DS_RATE_12_5_HZ,
  LSM6DS_RATE_26_HZ,
  LSM6DS_RATE_52_HZ,
  LSM6DS_RATE_104_HZ,
  LSM6DS_RATE_208_HZ,
  LSM6DS_RATE_416_HZ,
  LSM6DS_RATE_833_HZ,
  LSM6DS_RATE_1_66K_HZ,
  LSM6DS_RATE_3_33K_HZ,
  LSM6DS_RATE_6_66K_HZ,
  LSM6DSOX_RATE_1_6_HZ_LP,
} lsm6ds_data_rate_t;

/** The accelerometer data range */
typedef enum accel_range {
  LSM6DS_ACCEL_RANGE_2_G,
  LSM6DS_ACCEL_RANGE_16_G,
  LSM6DS_ACCEL_RANGE_4_G,
  LSM6DS_ACCEL_RANGE_8_G
} lsm6ds_accel_range_t;

/** The gyro data range */
typedef enum gyro_range {
  LSM6DS_GYRO_RANGE_125_DPS = 0b001,
  LSM6DS_GYRO_RANGE_250_DPS = 0b000,
  LSM6DS_GYRO_RANGE_500_DPS = 0b010,
  LSM6DS_GYRO_RANGE_1000_DPS = 0b100,
  LSM6DS_GYRO_RANGE_2000_DPS = 0b110
} lsm6ds_gyro_range_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DS I2C Accel/Gyro
 */
class Adafruit_LSM6DS {
public:
  Adafruit_LSM6DS();
  bool begin_I2C(uint8_t i2c_addr = LSM6DS_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensorID = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI, int32_t sensorID = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensorID = 0);

  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                sensors_event_t *temp);

  lsm6ds_data_rate_t getAccelDataRate(void);
  void setAccelDataRate(lsm6ds_data_rate_t data_rate);

  lsm6ds_accel_range_t getAccelRange(void);
  void setAccelRange(lsm6ds_accel_range_t new_range);

  lsm6ds_data_rate_t getGyroDataRate(void);
  void setGyroDataRate(lsm6ds_data_rate_t data_rate);

  lsm6ds_gyro_range_t getGyroRange(void);
  void setGyroRange(lsm6ds_gyro_range_t new_range);

  void reset(void);
  void configIntOutputs(bool active_low, bool open_drain);
  void configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl);
  void configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl);

  int16_t rawAccX, ///< Last reading's raw accelerometer X axis
    rawAccY,  ///< Last reading's raw accelerometer Y axis
    rawAccZ,  ///< Last reading's raw accelerometer Z axis
    rawTemp,   ///< Last reading's raw temperature reading
    rawGyroX,  ///< Last reading's raw gyro X axis
    rawGyroY,  ///< Last reading's raw gyro Y axis
    rawGyroZ;  ///< Last reading's raw gyro Z axis

protected:
  float temperature, accX, accY, accZ, gyroX, gyroY, gyroZ;
  uint8_t chipID();
  void _read(void);

  // unique initializer post i2c/spi init
  virtual bool _init(int32_t sensor_id) = 0;

  uint8_t _sensorid_accel, _sensorid_gyro, _sensorid_temp;

  Adafruit_I2CDevice *i2c_dev;
  Adafruit_SPIDevice *spi_dev;
};

#endif
