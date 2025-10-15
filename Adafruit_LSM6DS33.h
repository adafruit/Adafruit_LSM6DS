/*!
 *  @file Adafruit_LSM6DS33.h
 *
 * 	I2C Driver for the Adafruit LSM6DS33 6-DoF Accelerometer and Gyroscope
 *library
 *
 * 	This is a library for the Adafruit LSM6DS33 breakout:
 * 	https://www.adafruit.com/products/4480
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DS33_H
#define _ADAFRUIT_LSM6DS33_H

#include "Adafruit_LSM6DS.h"

#define LSM6DS33_CHIP_ID 0x69 ///< LSM6DS33 default device id from WHOAMI

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DS33 I2C Digital Potentiometer
 */
class Adafruit_LSM6DS33 : public Adafruit_LSM6DS {
public:
  Adafruit_LSM6DS33();
  ~Adafruit_LSM6DS33(){};

private:
  bool _init(int32_t sensor_id);

  //Fifo works completely differently in this chip compared to the others in the LSM6DS family
  //I don't have one to test with so I will de-implement the Fifo by making all related functions private
  using Adafruit_LSM6DS::parseFifoTag;
  using Adafruit_LSM6DS::parseFifoTimeSlot;
  using Adafruit_LSM6DS::getFifoStatus;
  using Adafruit_LSM6DS::getFifoAvailable;
  using Adafruit_LSM6DS::getFifo;
  using Adafruit_LSM6DS::configFifoInt1;
  using Adafruit_LSM6DS::configFifoInt2;
  using Adafruit_LSM6DS::setFifoAccelRate;
  using Adafruit_LSM6DS::setFifoGyroRate;
  using Adafruit_LSM6DS::setFifoTempRate;
  using Adafruit_LSM6DS::setFifoTSDec;
  using Adafruit_LSM6DS::setFifoMode;
  using Adafruit_LSM6DS::setFifoWatermark;
  using Adafruit_LSM6DS::setFifoCounterBDR;
  using Adafruit_LSM6DS::setFifoEmbFunc;

};

#endif
