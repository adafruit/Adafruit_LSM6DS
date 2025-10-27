/*!
 *  @file Adafruit_LSM6DSV320.h
 *
 * 	I2C Driver for the Adafruit LSM6DSV320 and LSM6DSV80 IMU chips.
 *
 *  The V80 is identical to the V320 except it only has 3 high-G ranges available,
 *  so you can use this same class for both chips.
 *
 *
 * 	This is a library for the Adafruit LSM6DSV320 breakout:
 * 	https://www.adafruit.com/products/NO_ID_ASSIGNED_YET
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */
/*
notes
Initial value of LSM6DSV_CTRL1_XL_HG 0b10010000
Set range to 0b100                   0b10010000
Set rate to 0b11                     0b10011000
s=scale/range o=ODRate                   ooosss 

*/
#ifndef _ADAFRUIT_LSM6DSV320_H
#define _ADAFRUIT_LSM6DSV320_H

#include "Adafruit_LSM6DSV.h"
#define LSM6DSV320_CHIP_ID 0x73     ///< LSM6DSV320/80 default device id from WHOAMI register 0x0F

//Additional registers specific to the V320
#define LSM6DSV_CTRL1_XL_HG 0x4E    ///< location of the high-G control register for rate+range


/** The high-g data range 
  The data ranges on the CTRL1 register are always numbered 0, 1, 2, 3 for all of the LSM6DS devices
  and this is the same for the low-G accelerometer in the V320
  However in the +/-320G device the second accelerometer has ranges 0-4 for 32, 64, 128, 256, 320
  which is one more, so we can't just overlap the same enum. 
  The V80 only has 3 ranges available for the high-g */
typedef enum high_g_range {
  LSM6DSV320_ACCEL_RANGE_32_G,
  LSM6DSV320_ACCEL_RANGE_64_G,
  LSM6DSV320_ACCEL_RANGE_128_G,
  LSM6DSV320_ACCEL_RANGE_256_G,
  LSM6DSV320_ACCEL_RANGE_320_G
} lsm6dsv320_accel_range_t;
#define LSM6DSV80_ACCEL_RANGE_32_G LSM6DSV320_ACCEL_RANGE_32_G
#define LSM6DSV80_ACCEL_RANGE_64_G LSM6DSV320_ACCEL_RANGE_64_G
#define LSM6DSV80_ACCEL_RANGE_80_G LSM6DSV320_ACCEL_RANGE_128_G

/** The high-g data rate - not the same set of rates as the main accel+gyro and two of them are "N/A" */
typedef enum high_g_rate {
  LSM6DSV320_HGRATE_SHUTDOWN,
  LSM6DSV320_HGRATE_NA1,
  LSM6DSV320_HGRATE_NA2,
  LSM6DSV320_HGRATE_480_HZ,
  LSM6DSV320_HGRATE_960_HZ,
  LSM6DSV320_HGRATE_1_92K_HZ,
  LSM6DSV320_HGRATE_3_84K_HZ,
  LSM6DSV320_HGRATE_7_68K_HZ
} lsm6dsv320_high_g_rate_t;


/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DSV320/80 - has a complete second "high G" accelerometer
 */
class Adafruit_LSM6DSV320 : public Adafruit_LSM6DSV {
public:
  Adafruit_LSM6DSV320();
  ~Adafruit_LSM6DSV320();

  lsm6dsv320_high_g_rate_t getHighGDataRate(void);
  void setHighGDataRate(lsm6dsv320_high_g_rate_t data_rate);

  lsm6dsv320_accel_range_t getHighGRange(void);
  void setHighGRange(lsm6dsv320_accel_range_t new_range);

  void configInt1(bool drdy_g, bool drdy_xl, bool drdy_hg);
  void configInt2(bool drdy_g, bool drdy_xl, bool drdy_hg);
  void setHighGScale(void);
  void enableHighG(bool en);
  float highGSampleRate(void);
  int highGAvailable(void);

protected:  
  //! buffer for the unique high-g accelerometer range
  lsm6dsv320_accel_range_t accelHighGBuffered = LSM6DSV320_ACCEL_RANGE_32_G;

private:
  bool _init(int32_t sensor_id);
};

/*!
 *    @brief  Sub-class for the "V80" version .
              The only difference is two less high-G accelerometer ranges
 */
class Adafruit_LSM6DSV80 : public Adafruit_LSM6DSV320  {
public:
  Adafruit_LSM6DSV80() {};
};

#endif