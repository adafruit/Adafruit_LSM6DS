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

#define LSM6DS_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions access register
#define LSM6DS_INT1_CTRL 0x0D      ///< Interrupt control for INT 1
#define LSM6DS_INT2_CTRL 0x0E      ///< Interrupt control for INT 2
#define LSM6DS_WHOAMI 0x0F         ///< Chip ID register
#define LSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define LSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define LSM6DS_CTRL3_C 0x12        ///< Main configuration register
#define LSM6DS_CTRL4_C 0x13        ///< Configuration register 4
#define LSM6DS_CTRL8_XL 0x17       ///< High and low pass for accel
#define LSM6DS_CTRL10_C 0x19       ///< Main configuration register
#define LSM6DS_WAKEUP_SRC 0x1B     ///< Why we woke up
#define LSM6DS_STATUS_REG 0X1E     ///< Status register
#define LSM6DS_OUT_TEMP_L 0x20     ///< First data register (temperature low)
#define LSM6DS_OUTX_L_G 0x22       ///< First gyro data register
#define LSM6DS_OUTX_L_A 0x28       ///< First accel data register
#define LSM6DS_STEPCOUNTER 0x4B    ///< 16-bit step counter
#define LSM6DS_TAP_CFG 0x58        ///< Tap/pedometer configuration
#define LSM6DS_WAKEUP_THS 0x5B     ///< Single and double-tap function threshold register
#define LSM6DS_WAKEUP_DUR 0x5C     ///< Free-fall, wakeup, timestamp and sleep mode duration
#define LSM6DS_MD1_CFG 0x5E        ///< Functions routing on INT1 register
#define LSM6DS_EMB_FUNC_FIFO_CFG 0x44   //enable routing embedded functions to Fifo (pedometer only for non-V chips)
#define LSM6DS_FIFO_CTRL1 0x07          //Fifo watermark
#define LSM6DS_FIFO_CTRL2 0x08          //Fifo config
#define LSM6DS_FIFO_CTRL3 0x09          //Fifo batch data rates
#define LSM6DS_FIFO_CTRL4 0x0A          //Fifo decimation, mode
#define LSM6DS_COUNTER_BDR_REG1 0x0B    //Fifo batch data controls + 2 high bits of threshold
#define LSM6DS_COUNTER_BDR_REG2 0x0C    //Fifo batch data threshold
#define LSM6DS_FIFO_STATUS1 0x3A        //Fifo quantity (lower 8 bits)
#define LSM6DS_FIFO_STATUS2 0x3B        //Fifo status and bits 9:8 of quantity
#define LSM6DS_FIFO_DATA_OUT_TAG 0x78   //First byte of Fifo output registers (7 bytes)
#define LSM6DS_SENSOR_HUB_1   0x02 //sensor hub first data byte (in the sensor hub register bank selected by func_cfg_access)
#define LSM6DS_MASTER_CONFIG  0x14 //sensor hub I2C master
#define LSM6DS_SLV0_ADD       0x15 //sensor hub slave chip address
#define LSM6DS_SLV0_SUBADD    0x16 //sensor hub slave register address
#define LSM6DS_SLV0_CONFIG    0x17 //sensor hub slave config
#define LSM6DS_SLV1_ADD       0x18 //sensor hub slave chip address
#define LSM6DS_SLV1_SUBADD    0x19 //sensor hub slave register address
#define LSM6DS_SLV1_CONFIG    0x1A //sensor hub slave config
#define LSM6DS_SLV2_ADD       0x1B //sensor hub slave chip address
#define LSM6DS_SLV2_SUBADD    0x1C //sensor hub slave register address
#define LSM6DS_SLV2_CONFIG    0x1D //sensor hub slave config
#define LSM6DS_SLV3_ADD       0x1E //sensor hub slave chip address
#define LSM6DS_SLV3_SUBADD    0x1F //sensor hub slave register address
#define LSM6DS_SLV3_CONFIG    0x20 //sensor hub slave config
#define LSM6DS_DATAWRITE_SLV0 0x21 //sensor hub data byte to write to slave 0
#define LSM6DS_STATUS_MASTER  0x22 //Sensor hub status (use the duplicate instead)
#define LSM6DS_STATUS_MASTER_MAINPAGE  0x39 //Sensor hub status - duplicate register in main page of registers

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
  LSM6DS_GYRO_RANGE_125_DPS = 0b0010,
  LSM6DS_GYRO_RANGE_250_DPS = 0b0000,
  LSM6DS_GYRO_RANGE_500_DPS = 0b0100,
  LSM6DS_GYRO_RANGE_1000_DPS = 0b1000,
  LSM6DS_GYRO_RANGE_2000_DPS = 0b1100,
  ISM330DHCX_GYRO_RANGE_4000_DPS = 0b0001
} lsm6ds_gyro_range_t;

/** The high pass filter bandwidth */
typedef enum hpf_range {
  LSM6DS_HPF_ODR_DIV_50 = 0,
  LSM6DS_HPF_ODR_DIV_100 = 1,
  LSM6DS_HPF_ODR_DIV_9 = 2,
  LSM6DS_HPF_ODR_DIV_400 = 3,
} lsm6ds_hp_filter_t;

/** The fifo tags */
typedef enum fifo_tag {
  LSM6_FIFO_TAG_EMPTY,
  LSM6_FIFO_TAG_GYRO,
  LSM6_FIFO_TAG_ACCEL,
  LSM6_FIFO_TAG_TEMP,
  LSM6_FIFO_TAG_TIME,
  LSM6_FIFO_TAG_CFG_CHANGE,
  LSM6_FIFO_TAG_ACCEL_NC_T_2,   //NC and C tags are related to data compression (not implemented in this library) See AN5922 or AN5192
  LSM6_FIFO_TAG_ACCEL_NC_T_1,
  LSM6_FIFO_TAG_ACCEL_2xC,
  LSM6_FIFO_TAG_ACCEL_3xC,
  LSM6_FIFO_TAG_GYRO_NC_T_2,
  LSM6_FIFO_TAG_GYRO_NC_T_1,
  LSM6_FIFO_TAG_GYRO_2xC,
  LSM6_FIFO_TAG_GYRO_3xC,
  LSM6_FIFO_TAG_SENSOR_0,    //sensor hub
  LSM6_FIFO_TAG_SENSOR_1,
  LSM6_FIFO_TAG_SENSOR_2,
  LSM6_FIFO_TAG_SENSOR_3,
  LSM6_FIFO_TAG_STEP,       
  //skip a lot of numbers - step was 0x12
  LSM6_FIFO_TAG_SENSOR_NACK = 0x19  
} lsm6_fifo_tag_t;

/** The fifo modes */
typedef enum fifo_mode {
  LSM6_FIFO_MODE_BYPASS,
  LSM6_FIFO_MODE_FIFO,
  LSM6_FIFO_MODE_CONTINUOUSWTM_TO_FULL,
  LSM6_FIFO_MODE_CONTINUOUS_TO_FIFO,
  LSM6_FIFO_MODE_BYPASS_TO_CONTINUOUS,
  LSM6_FIFO_MODE_RESERVED,
  LSM6_FIFO_MODE_CONTINUOUS,
  LSM6_FIFO_MODE_BYPASS_TO_FIFO
} lsm6_fifo_mode_t;

/** fifo timestamp decimation (divides highest rate of gyro or accel) */
typedef enum fifo_dec_ts {
  LSM6_FIFO_DEC_TS_SHUTDOWN,
  LSM6_FIFO_DEC_TS_1,
  LSM6_FIFO_DEC_TS_8,
  LSM6_FIFO_DEC_TS_32
} lsm6_fifo_dec_ts_t;

/** Sensor used as batch trigger for Counter_BDR */
typedef enum fifo_batch_trig {
  LSM6_FIFO_BATCH_TRIG_ACCEL,
  LSM6_FIFO_BATCH_TRIG_GYRO
} lsm6_fifo_batch_trig_t;

/** Data rate (or decimation rate) for sensor hub - different ranges depending on which chip you have */

class Adafruit_LSM6DS;

/** Adafruit Unified Sensor interface for temperature component of LSM6DS */
class Adafruit_LSM6DS_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the LSM6DS class */
  Adafruit_LSM6DS_Temp(Adafruit_LSM6DS *parent) { _theLSM6DS = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D0;
  Adafruit_LSM6DS *_theLSM6DS = NULL;
};

/** Adafruit Unified Sensor interface for accelerometer component of LSM6DS */
class Adafruit_LSM6DS_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the LSM6DS class */
  Adafruit_LSM6DS_Accelerometer(Adafruit_LSM6DS *parent) {
    _theLSM6DS = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D1;
  Adafruit_LSM6DS *_theLSM6DS = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of LSM6DS */
class Adafruit_LSM6DS_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the LSM6DS class */
  Adafruit_LSM6DS_Gyro(Adafruit_LSM6DS *parent) { _theLSM6DS = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D2;
  Adafruit_LSM6DS *_theLSM6DS = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DS I2C Accel/Gyro
 */
class Adafruit_LSM6DS {
public:
  Adafruit_LSM6DS();
  virtual ~Adafruit_LSM6DS();

  bool begin_I2C(uint8_t i2c_addr = LSM6DS_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensorID = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI, int32_t sensorID = 0,
                 uint32_t frequency = 1000000);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensorID = 0,
                 uint32_t frequency = 1000000);

  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                sensors_event_t *temp);
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro);

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
  void configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl,
                  bool step_detect = false, bool wakeup = false);
  void configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl);
  void configIntOpenDrain(bool OD = false);
  void configInt2OnInt1(bool allOn1 = false);
  void configInt2InHigh(bool activeHigh = false) {}; //No function, except on "V" chips
  void highPassFilter(bool enabled, lsm6ds_hp_filter_t filter);

  void enableWakeup(bool enable, uint8_t duration = 0, uint8_t thresh = 20);
  bool awake(void);
  bool shake(void);

  void enablePedometer(bool enable);
  void resetPedometer(void);
  uint16_t readPedometer(void);

  // Arduino compatible API
  int readAcceleration(float &x, float &y, float &z);
  float accelerationSampleRate(void);
  int accelerationAvailable(void);

  int readGyroscope(float &x, float &y, float &z);
  float gyroscopeSampleRate(void);
  int gyroscopeAvailable(void);

  int16_t rawAccX, ///< Last reading's raw accelerometer X axis
      rawAccY,     ///< Last reading's raw accelerometer Y axis
      rawAccZ,     ///< Last reading's raw accelerometer Z axis
      rawTemp,     ///< Last reading's raw temperature reading
      rawGyroX,    ///< Last reading's raw gyro X axis
      rawGyroY,    ///< Last reading's raw gyro Y axis
      rawGyroZ;    ///< Last reading's raw gyro Z axis

  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ;         ///< Last reading's gyro Z axis in rad/s

  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getAccelerometerSensor(void);
  Adafruit_Sensor *getGyroSensor(void);

  //Fifo
  lsm6_fifo_tag_t parseFifoTag(uint16_t x) {return((lsm6_fifo_tag_t)((x & 0b11111000) >> 3));}
  uint8_t parseFifoTimeSlot(uint16_t x) {return((x & 0b110) >> 1);}
  uint8_t getFifoStatus(); //status bits only
  int getFifoAvailable(); //number of entries available to read
  size_t getFifo(uint8_t *data, size_t numToGet = 1); //the actual tagged data
  void configFifoInt1(bool threshold = false, bool overflow = false, bool full = false, bool batch = false); //threshold interrupt when there are N sensor readings in the fifo, batch interrupt when there are N readings of one sensor out of accel/gyro/EIS gyro/High-G accel, not counting the other data in the fifo
  void configFifoInt2(bool threshold = false, bool overflow = false, bool full = false, bool batch = false); 
  void setFifoAccelRate(lsm6ds_data_rate_t data_rate);
  void setFifoGyroRate(lsm6ds_data_rate_t data_rate);
  void setFifoTempRate(lsm6ds_data_rate_t data_rate); //temperature can only use the 3 slowest rates 1.6Hz, 12.5Hz, 52Hz. LSM6DS_RATE_SHUTDOWN for no temperature in fifo.
  void setFifoTSDec(lsm6_fifo_dec_ts_t dec); //divides the rate of accel or gyro
  void setFifoMode(lsm6_fifo_mode_t mode);
  void setFifoWatermark(uint8_t wtm);
  void setFifoCounterBDR(lsm6_fifo_batch_trig_t trig, uint16_t threshold);
  void setFifoEmbFunc(bool stepCounter = false);

  //Sensor Hub - don't attempt to use these on chips without sensor hub: LSM6DSOP and LSM6DS33
  bool sHubConcluded(); //true when the "end operation" bit is set in status_master
  bool sHubWriteOnceDone(); //true when the write-once operation has been done (status_master)
  uint8_t sHubSlaveNAK(); //4 bits representing slave NAK status (status_master)
  void sHubConfigResetMaster(bool resetMaster = false);
  void sHubConfigWriteOnce(bool writeOnce = false);
  void sHubConfigStartConfig(bool startConfig = false);  //true for INT2 pin, false for accel/gyro trigger
  void sHubConfigPassThrough(bool passThrough = false);
  void sHubConfigPullUp(bool pullUp = false);
  void sHubConfigMasterOn(bool masterOn = false);
  void sHubConfigCountSensors(int countSensors = 0);
  int sHubGetCountSensors(); 
  void sHubSensor(int sensorNumber, uint8_t addr, bool readBit, uint8_t subAddr, int numReadOperations, bool enableFifo = false, uint8_t sHubODR = 0);
  //sensor number 0-3. Only sensor 0 has the ODR setting (12.5-104Hz) 
  void sHubDataWrite(uint8_t data); //single byte of data to write to sensor 0 (usually used for write-once to send one byte to the sensor)
  bool sHubData(uint8_t data[], size_t numBytes); //read bytes from the sensor hub registers (max 18) 


protected:
  uint8_t chipID(void);
  uint8_t status(void);
  virtual void _read(void);
  virtual void _readFast(void);
  virtual bool _init(int32_t sensor_id);
  void funcCfgAccess(bool embFunc = false, bool sHub = false);

  uint16_t _sensorid_accel, ///< ID number for accelerometer
      _sensorid_gyro,       ///< ID number for gyro
      _sensorid_temp;       ///< ID number for temperature

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  float temperature_sensitivity =
      256.0; ///< Temp sensor sensitivity in LSB/degC
  Adafruit_LSM6DS_Temp *temp_sensor = NULL; ///< Temp sensor data object
  Adafruit_LSM6DS_Accelerometer *accel_sensor =
      NULL;                                 ///< Accelerometer data object
  Adafruit_LSM6DS_Gyro *gyro_sensor = NULL; ///< Gyro data object

  //! buffer for the accelerometer range
  lsm6ds_accel_range_t accelRangeBuffered = LSM6DS_ACCEL_RANGE_2_G;
  //! buffer for the gyroscope range
  lsm6ds_gyro_range_t gyroRangeBuffered = LSM6DS_GYRO_RANGE_250_DPS;

private:
  friend class Adafruit_LSM6DS_Temp; ///< Gives access to private members to
                                     ///< Temp data object
  friend class Adafruit_LSM6DS_Accelerometer; ///< Gives access to private
                                              ///< members to Accelerometer data
                                              ///< object
  friend class Adafruit_LSM6DS_Gyro; ///< Gives access to private members to
                                     ///< Gyro data object

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
  void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
};

#endif
