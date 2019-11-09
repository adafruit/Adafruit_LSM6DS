/*!
 *  @file Adafruit_LSM6DSOX.h
 *
 * 	I2C Driver for the Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope library
 *
 * 	This is a library for the Adafruit LSM6DSOX breakout:
 * 	https://www.adafruit.com/products/PID_HERE
 * 
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSOX_H
#define _ADAFRUIT_LSM6DSOX_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>


#define LSM6DSOX_I2CADDR_DEFAULT 0x6a ///< LSM6DSOX default i2c address
#define LSM6DSOX_CHIP_ID 0x6C ///< LSM6DSOX default device id from WHOAMI


#define LSM6DSOX_CTRL1_XL 0x10 ///< Main accelerometer config register
#define LSM6DSOX_CTRL2_G 0x11 ///< Main gyro config register
#define LSM6DSOX_CTRL3_C 0x12 ///< Main configuration register

#define LSM6DSOX_WHOAMI              0xF ///< Chip ID register
#define LSM6DSOX_OUTX_L_G  0x22 ///< First data register (gyro x low)
#define LSM6DSOX_OUTX_L_A  0x28 ///< First accel data register

/** The accelerometer data rate */
typedef enum data_rate {
  LSM6DSOX_RATE_SHUTDOWN,
  LSM6DSOX_RATE_12_5_HZ,
  LSM6DSOX_RATE_26_HZ,
  LSM6DSOX_RATE_52_HZ,
  LSM6DSOX_RATE_104_HZ,
  LSM6DSOX_RATE_208_HZ,
  LSM6DSOX_RATE_416_HZ,
  LSM6DSOX_RATE_833_HZ,
  LSM6DSOX_RATE_1_66K_HZ,
  LSM6DSOX_RATE_3_33K_HZ,
  LSM6DSOX_RATE_6_66K_HZ,
  LSM6DSOX_RATE_1_6_HZ_LP // Only works for the accelerometer in low power mode
} lsm6dsox_data_rate_t;

/** The accelerometer data range */
typedef enum accel_range {
  LSM6DSOX_ACCEL_RANGE_2_G,
  LSM6DSOX_ACCEL_RANGE_16_G,
  LSM6DSOX_ACCEL_RANGE_4_G,
  LSM6DSOX_ACCEL_RANGE_8_G
} lsm6dsox_accel_range_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DSOX I2C Digital Potentiometer
 */
class Adafruit_LSM6DSOX {
public:

  Adafruit_LSM6DSOX();  
  bool begin(uint8_t i2c_addr = LSM6DSOX_I2CADDR_DEFAULT,
                TwoWire *wire = &Wire,
                int32_t sensorID = 0);
  bool getEvent(sensors_event_t *accel);

  lsm6dsox_data_rate_t getAccelDataRate(void);
  void setAccelDataRate(lsm6dsox_data_rate_t data_rate);

  lsm6dsox_accel_range_t getAccelRange(void);
  void setAccelRange(lsm6dsox_accel_range_t new_range);

  void reset(void);

private:
  void _read(void);

  float temperature, accX, accY, accZ, gyroX, gyroY, gyroZ;
  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
  uint8_t _sensorid_accel;
  Adafruit_I2CDevice *i2c_dev;
};

#endif
