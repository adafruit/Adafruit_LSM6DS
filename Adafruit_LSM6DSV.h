/*!
 *  @file Adafruit_LSM6DSV.h
 *
 * 	I2C Driver base for LSM6DSVxx Dual-Channel Accelerometer and Gyroscope
 *      library
 *
 *    LSM6DSV - "dual channel" allows a 16G accelerometer scale to appear on the "high G" outputs
 *    LSM6DSV16 - "dual channel" is always 16G, adds Qvar electrostatic inputs (not implemented)
 *
 *  Sub-class for chip with different chip ID
 *    LSM6DSV16B -  replaces second I2C interface with TDM audio interface (not implemented)
 *                  NOTE: for your PCB layout, INT2 is moved to pin 10 instead of pin 9.
 *
 *  Sub-class for chip with different max range:  
 *    LSM6DSV32 - "dual channel" is always 32G
 *
 *  Separate .h for true dual accelerometer chips, inherits from this class:
 *    LSM6DSV320 9-DoF 16G Accelerometer, 4000dps Gyroscope and 320G "high-G" Accelerometer
 *    LSM6DSV80 9-DoF 16G Accelerometer, 4000dps Gyroscope and 80G "high-G" Accelerometer
 *
 *  The register map on these chips with dual accelerometers is quite different to the 
 *  LSM6DSO series chips. All chips except the V80 and V16B have OIS abilities, which isn't supported
 *  by this library. All "V" chips have the sensor fusion low-power (SFLP) algorithm available.
 *
 *  Chips without a true high-g accelerometer return 0x70 for the who-am-I.
 *  The "B" TDM chip who-am-I is 0x71
 *  The 80 and 320 who-am-I is 0x73
 *
 *  Note on units used: 
 *      Acceleration is always meters/sec/sec
 *      Gyro is usually degrees/sec except when using the "event" sensor objects, where radians/sec is used
 *
 *  There are multiple ways of getting the data from the IMU:
 *      1. Call _read() or _readFast() then access the raw or scaled public values *RECOMMENDED*
 *      2. Call getEvent(sensor, sensor, sensor, sensor) to fill in Adafruit sensor objects
 *      3. Call read...(x, y, z) to get 12 bytes of XYZ values direct from the chip and scale to 
 *         floating-point output units. "Arduino API" *GOOD FOR JUST ONE SENSOR OF INTEREST AND SFLP*
 *      4. Call get...Sensor() to get a pointer to just one sensor object inside the class. 
 *         Then you can call .getEvent() on each sensor. Each getEvent does a full 26-byte data read on the chip.
 *         .getSensor() on a sensor will show the sensor details such as max/min but that is erased by a .getEvent()
 *      5. For "Gimme acceleration without gravity" you can enable the SFLP algorithm and read its outputs (up to 480Hz)
 *      6. All outputs can be routed through the FIFO, each set of 7 bytes read from FIFO has an identifier at the beginning
 *         to tell you what kind of data is stored in the 6 data bytes of the FIFO. 
 *         The SFLP rotation quaternion is special because there's 4 16-bit data items to be communicated.
 *         So it sends that FIFO tag twice, the first time has a zero in the first element and the second has a 1
 *         therefore indicating that the first one has (presumably, I didn't check) the W and X components and the second the Y and Z.
 *         It's using the first data item as an additional label so you know which half of the quaternion you're seeing.
 *
 *
 * 	Adafruit invests time and resources providing this open source code,
 *      please support Adafruit and open-source hardware by purchasing products
 *      from Adafruit!
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSV_H
#define _ADAFRUIT_LSM6DSV_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//#include <float16.h>  //Include this if you want to use the 16-bit floatng point values that the SFLP algorithm uses
                        //AND you have installed the float16 library https://github.com/RobTillaart/float16/tree/master
#ifndef FLOAT16_LIB_VERSION
#define float16 int16_t //Since you're not using the 16-bit floats, this line prevents compiler errors
#endif

#define LSM6DSV_I2CADDR_DEFAULT 0x6A ///< LSM6DSxxx default i2c address (address pin grounded)
#define LSM6DSV_CHIP_ID 0x70         ///< LSM6DSV default device id from WHOAMI register 0x0F
#define LSM6DSV16B_CHIP_ID 0x71      ///< LSM6DSV16B default device id from WHOAMI register 0x0F
 
#define LSM6DSV_FUNC_CFG_ACCESS 0x01 ///< Setting bits here controls access to the overlapping registers
#define LSM6DSV_PIN_CTRL 0x02        ///< Output pin control (interrupt drive strength)
#define LSM6DSV_IF_CFG 0x03          ///< Interface config
#define LSM6DSV_INT1_CTRL 0x0D       ///< Interrupt control for INT 1
#define LSM6DSV_INT2_CTRL 0x0E       ///< Interrupt control for INT 2
#define LSM6DSV_WHOAMI 0x0F          ///< Chip ID register
#define LSM6DSV_CTRL1 0x10           ///< Accelerometer mode and data rate
#define LSM6DSV_CTRL2 0x11           ///< Gyro mode and data rate
#define LSM6DSV_CTRL3 0x12           ///< Not used: software reset, reboot, block-update mode, block-read mode
#define LSM6DSV_CTRL6 0x15           ///< location of the gyro control register for LPF+range
#define LSM6DSV_CTRL7 0x16           ///< location of the interrupt-enable for high-G and Gyro LPF enable
#define LSM6DSV_CTRL8 0x17           ///< Accelerometer range and low/high pass
#define LSM6DSV_CTRL9 0x18           ///< Accelerometer filters enable
#define LSM6DSV_CTRL10 0x19          ///< debug/self-test
#define LSM6DSV_STATUS_REG 0X1E      ///< Status register (high-G+tepmerature+accel+gyro data-ready)
#define LSM6DSV_OUT_TEMP_L 0x20      ///< First data register (temperature low)
#define LSM6DSV_OUTX_L_G 0x22        ///< First gyro data register
#define LSM6DSV_OUTX_L_A 0x28        ///< First accel data register
#define LSM6DSV_OUTX_L_HG 0x34       ///< First high-g accel data register
#define LSM6DSV_INTERNAL_FREQ 0X4F   ///< Adjust internal clock frequency +/- in steps of 0.13%
#define LSM6DSV_FUNCTIONS_ENABLE 0x50 //interrupts and timestamp enable
#define LSM6DSV_MD1_CFG 0x5E         ///< Functions routing on INT1 register
#define LSM6DSV_X_OFS_USR 0x73       ///< Accelerometer user offset
#define LSM6DSV_Y_OFS_USR 0x74       ///< Accelerometer user offset
#define LSM6DSV_Z_OFS_USR 0x75       ///< Accelerometer user offset
#define LSM6DSV_HG_X_OFS_USR 0x6C    ///< High-G accelerometer user offset
#define LSM6DSV_HG_Y_OFS_USR 0x6D    ///< High-G accelerometer user offset
#define LSM6DSV_HG_Z_OFS_USR 0x6E    ///< High-G accelerometer user offset
#define LSM6DSV_EMB_FUNC_EN_A 0x04   ///< Embedded functions control register
#define LSM6DSV_EMB_FUNC_EXEC_STATUS 0x07 ///< Embedded functions status register (overflow and running/not running)
#define LSM6DSV_SFLP_ODR 0x5E        ///< SFLP output data rate (an embedded function register)
#define LSM6DSV_SFLP_GBIASX_L 0x16   ///< SFLP gyro bias output (an embedded function register)
#define LSM6DSV_SFLP_GRAVX_L 0x1E    ///< SFLP gravity output (an embedded function register)
#define LSM6DSV_SFLP_QUATW_L 0x2A    ///< SFLP quaternion output (an embedded function register) float16
#define LSM6DSV_SFLP_GBIASX_INIT_L 0x32 ///< SFLP gyro bias input/output (an embedded function register) float16
#define LSM6DSV_EMB_FUNC_CFG 0x63    //despite the name, this is in the main register bank
#define LSM6DSV_EMB_FUNC_INIT_B 0x67  //initialize embedded functions like high-G peak detect
#define LSM6DSV_EMB_FUNC_FIFO_EN_A 0x44 //enable routing embedded functions to Fifo
#define LSM6DSV_EMB_FUNC_FIFO_EN_B 0x45 //enable routing FSM to Fifo
#define LSM6DSV_FSM_ODR 0x5F          //[embedded] ouptut data rate (ODR) of FSM's (same set of rates as the SFLP)
#define LSM6DSV_FIFO_STATUS1 0x1B    //Fifo status and high bit of level
#define LSM6DSV_FIFO_STATUS2 0x1C    //Lower 8 bits of Fifo level
#define LSM6DSV_FIFO_CTRL1 0x07      //Fifo watermark threshold
#define LSM6DSV_FIFO_CTRL2 0x09      //Fifo controls, compression
#define LSM6DSV_FIFO_CTRL3 0x09      //accel+gyro data rates in Fifo
#define LSM6DSV_FIFO_CTRL4 0x0A      //Fifo mode, timestamp, temperature data rates
#define LSM6DSV_COUNTER_BDR_REG1 0x0B //batch data rate counter settings
#define LSM6DSV_COUNTER_BDR_REG2 0x0B //batch data rate counter threshold
#define LSM6DSV_FIFO_DATA_OUT_TAG 0x78 //Start of Fifo output data bytes

/** The accel or gyro data rate - note the gyro can't do 1.875Hz */
typedef enum data_rate {
  LSM6DSV_RATE_SHUTDOWN,
  LSM6DSV_RATE_1_875_HZ,
  LSM6DSV_RATE_7_5_HZ,
  LSM6DSV_RATE_15_HZ,
  LSM6DSV_RATE_30_HZ,
  LSM6DSV_RATE_60_HZ,
  LSM6DSV_RATE_120_HZ,
  LSM6DSV_RATE_240_HZ,
  LSM6DSV_RATE_480_HZ,
  LSM6DSV_RATE_960_HZ,
  LSM6DSV_RATE_1_92K_HZ,
  LSM6DSV_RATE_3_84K_HZ,
  LSM6DSV_RATE_7_68K_HZ
} lsm6dsv_data_rate_t;

/** The output data rates for the sensor fusion algorithm or FSM */
typedef enum sflp_data_rate {
  LSM6DSV_SFLP_RATE_15_HZ,
  LSM6DSV_SFLP_RATE_30_HZ,
  LSM6DSV_SFLP_RATE_60_HZ,
  LSM6DSV_SFLP_RATE_120_HZ,
  LSM6DSV_SFLP_RATE_240_HZ,
  LSM6DSV_SFLP_RATE_480_HZ
} lsm6dsv_sflp_data_rate_t;
typedef enum fsm_data_rate { //non "v" chips have a different set of rates, slower
  LSM6DSV_FSM_RATE_15_HZ,
  LSM6DSV_FSM_RATE_30_HZ,
  LSM6DSV_FSM_RATE_60_HZ,
  LSM6DSV_FSM_RATE_120_HZ,
  LSM6DSV_FSM_RATE_240_HZ,
  LSM6DSV_FSM_RATE_480_HZ
} lsm6dsv_fsm_data_rate_t;


/** The accelerometer data range - identical for all except DSV32*/
typedef enum accel_range {
  LSM6DSV_ACCEL_RANGE_2_G,
  LSM6DSV_ACCEL_RANGE_4_G,
  LSM6DSV_ACCEL_RANGE_8_G,
  LSM6DSV_ACCEL_RANGE_16_G,
  //Aliases for DSV32 only
  LSM6DSV32_ACCEL_RANGE_4_G = LSM6DSV_ACCEL_RANGE_2_G,
  LSM6DSV32_ACCEL_RANGE_8_G = LSM6DSV_ACCEL_RANGE_4_G,
  LSM6DSV32_ACCEL_RANGE_16_G = LSM6DSV_ACCEL_RANGE_8_G,
  LSM6DSV32_ACCEL_RANGE_32_G = LSM6DSV_ACCEL_RANGE_16_G
} lsm6dsv_accel_range_t;


/** The accelerometer operating modes*/
typedef enum accel_op_mode {
  LSM6DSV_ACCEL_MODE_HIGH_PERF, //default
  LSM6DSV_ACCEL_MODE_HIGH_ACC,  //not recommended as it requires other settings for HAODR mode
  LSM6DSV_ACCEL_MODE_RESERVED,
  LSM6DSV_ACCEL_MODE_ODR_TRIG, //External trigger on INT2
  LSM6DSV_ACCEL_MODE_LP_1,     //low power, allows speeds below 7.5Hz, some restrictions
  LSM6DSV_ACCEL_MODE_LP_2,
  LSM6DSV_ACCEL_MODE_LP_3,
  LSM6DSV_ACCEL_MODE_NORMAL    //data rate 7.5Hz to 1.92kHz, use high-performance for faster, low-power for slower
} lsm6dsv_accel_op_mode_t;

/** The gyro full-scale range */
typedef enum gyro_range {
  LSM6DSV_GYRO_RANGE_RESERVED,
  LSM6DSV_GYRO_RANGE_250_DPS,
  LSM6DSV_GYRO_RANGE_500_DPS,
  LSM6DSV_GYRO_RANGE_1000_DPS,
  LSM6DSV_GYRO_RANGE_2000_DPS,
  LSM6DSV_GYRO_RANGE_4000_DPS
} lsm6dsv_gyro_range_t;

/** The high pass and low pass filter bandwidth */
typedef enum hpf_range {
  LSM6DSV_FILTER_ODR_DIV_4,
  LSM6DSV_FILTER_ODR_DIV_10,
  LSM6DSV_FILTER_ODR_DIV_20,
  LSM6DSV_FILTER_ODR_DIV_45,
  LSM6DSV_FILTER_ODR_DIV_100,
  LSM6DSV_FILTER_ODR_DIV_200,
  LSM6DSV_FILTER_ODR_DIV_400,
  LSM6DSV_FILTER_ODR_DIV_800
} lsm6dsv_filter_t;

/** The fifo tags */
typedef enum fifo_tag {
  LSM6_FIFO_TAG_EMPTY,
  LSM6_FIFO_TAG_GYRO,
  LSM6_FIFO_TAG_ACCEL,
  LSM6_FIFO_TAG_TEMP,
  LSM6_FIFO_TAG_TIME,
  LSM6_FIFO_TAG_CFG_CHANGE,
  LSM6_FIFO_TAG_ACCEL_NC_T_2,   //datasheet gives no clues on what these "NC" and "xC" tags mean
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
  LSM6_FIFO_TAG_STEP,       //pedometer step counter/indicator, no functions in this library for this yet
  LSM6_FIFO_TAG_SFLP_ROTATION, //See full description above for this special tag
  LSM6_FIFO_UNUSED1, 
  LSM6_FIFO_UNUSED2,
  LSM6_FIFO_TAG_SFLP_BIAS,
  LSM6_FIFO_TAG_SFLP_GRAVITY,
  LSM6_FIFO_TAG_HIGHG_PEAK,   //only the FSM can start/stop peak recording
  LSM6_FIFO_TAG_SENSOR_NACK,  //sensor hub?
  LSM6_FIFO_TAG_MLC_RESULT,
  LSM6_FIFO_TAG_MLC_FILTER,
  LSM6_FIFO_TAG_MLC_FEATURE,
  LSM6_FIFO_TAG_HIGHG,
  LSM6_FIFO_TAG_EIS_GYRO,
  LSM6_FIFO_TAG_FSM_RESULTS, 
  //Aliases
  LSM6_FIFO_TAG_FSM_QVAR = LSM6_FIFO_TAG_FSM_RESULTS //V16BX stores QVAR in position 0x1F
  //V16BX is also unique for the FIFO storing accelerometer items as ZYX while gyros are XYZ
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
  LSM6_FIFO_BATCH_TRIG_GYRO,
  LSM6_FIFO_BATCH_TRIG_EIS,
  LSM6_FIFO_BATCH_TRIG_HIGHG
} lsm6_fifo_batch_trig_t;

class Adafruit_LSM6DSV;

/** Adafruit Unified Sensor interface for temperature component of LSM6DS */
class Adafruit_LSM6DSV_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the LSM6DSV class */
  Adafruit_LSM6DSV_Temp(Adafruit_LSM6DSV *parent) { _theLSM6DSV = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D0; //placeholder for unique sensor ID to be assigned later
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/** Adafruit Unified Sensor interface for accelerometer component of LSM6DSV */
class Adafruit_LSM6DSV_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the LSM6DSV class */
  Adafruit_LSM6DSV_Accelerometer(Adafruit_LSM6DSV *parent) {
    _theLSM6DSV = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D1;
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of LSM6DSV */
class Adafruit_LSM6DSV_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the LSM6DS class */
  Adafruit_LSM6DSV_Gyro(Adafruit_LSM6DSV *parent) { _theLSM6DSV = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D2;
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/** Adafruit Unified Sensor interface for high-g component of LSM6DSV */
class Adafruit_LSM6DSV_HighG : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the LSM6DSV class */
  Adafruit_LSM6DSV_HighG(Adafruit_LSM6DSV *parent) {
    _theLSM6DSV = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D3;
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DSV I2C Accel/Gyro/accel
 */
class Adafruit_LSM6DSV {
public:
  Adafruit_LSM6DSV();
  virtual ~Adafruit_LSM6DSV();

  bool begin_I2C(uint8_t i2c_addr = LSM6DSV_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensorID = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI, int32_t sensorID = 0,
                 uint32_t frequency = 1000000);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensorID = 0,
                 uint32_t frequency = 1000000);

  bool getEvent(sensors_event_t *accel, 
	            sensors_event_t *gyro);
  bool getEvent(sensors_event_t *accel, 
	            sensors_event_t *gyro,
                sensors_event_t *temp);
  bool getEvent(sensors_event_t *accel, 
	            sensors_event_t *gyro,
                sensors_event_t *temp,
                sensors_event_t *highG);

  lsm6dsv_sflp_data_rate_t getSFLPDataRate(void);
  void setSFLPDataRate(lsm6dsv_sflp_data_rate_t data_rate);

  lsm6dsv_fsm_data_rate_t getFSMDataRate(void);
  void setFSMDataRate(lsm6dsv_fsm_data_rate_t data_rate);

  lsm6dsv_data_rate_t getAccelDataRate(void);
  void setAccelDataRate(lsm6dsv_data_rate_t data_rate);

  lsm6dsv_accel_range_t getAccelRange(void);
  void setAccelRange(lsm6dsv_accel_range_t new_range);

  lsm6dsv_data_rate_t getGyroDataRate(void);
  void setGyroDataRate(lsm6dsv_data_rate_t data_rate);

  lsm6dsv_gyro_range_t getGyroRange(void);
  void setGyroRange(lsm6dsv_gyro_range_t new_range);

  lsm6dsv_data_rate_t getHighGDataRate(void);
  void setHighGDataRate(lsm6dsv_data_rate_t data_rate);

  lsm6dsv_accel_range_t getHighGRange(void) {return LSM6DSV_ACCEL_RANGE_16_G;};
  void setHighGRange(lsm6dsv_accel_range_t new_range) {};

  int8_t getInternalFreqFine(void);

  void reset(void);
  void configIntOutputs(bool active_low, bool open_drain, uint8_t drive_strength = 0b11);
  void configInt1(bool drdy_g, bool drdy_xl);
  void configInt2(bool drdy_g, bool drdy_xl);
  virtual void enableHighG(bool en);
  void accelHighPassFilter(bool enabled, lsm6dsv_filter_t filter);
  void accelLowPassFilter(bool enabled, lsm6dsv_filter_t filter);
  void gyroLowPassFilter(bool enabled, uint8_t filter);
  //high-G accelerometer only has a low-pass filter and it's not configurable
  void accelOffset(bool enabled, 
			bool weight = false, 
			int8_t offsetX = 0, 
			int8_t offsetY = 0, 
			int8_t offsetZ = 0);
  void highGOffset(int8_t offsetX, int8_t offsetY, int8_t offsetZ);

  /* wakeup and pedometer functions not implemented yet*/
  //void enableWakeup(bool enable, uint8_t duration = 0, uint8_t thresh = 20);
  //bool awake(void);
  //bool shake(void);

  //void enablePedometer(bool enable);
  //void resetPedometer(void);
  //uint16_t readPedometer(void);

  void enableSFLP(bool enable); //turn on/off the sensor-fusion-low-power (embedded function)
  int readSFLPQuaternion(float16 &w, float16 &x, float16 &y, float16 &z);
  int readSFLPGravity(float &x, float &y, float &z);
  int readSFLPGyroBias(float &x, float &y, float &z);
  //no read-write for the initial gyro bias or the "init" bit in EMB_FUNC_INIT_A

  // Arduino compatible API
  int readAcceleration(float &x, float &y, float &z);
  float accelerationSampleRate(void);
  int accelerationAvailable(void);

  int readGyroscope(float &x, float &y, float &z);
  float gyroscopeSampleRate(void);
  int gyroscopeAvailable(void);

  int readHighG(float &x, float &y, float &z);
  virtual float highGSampleRate(void);
  virtual int highGAvailable(void);

  int readTemperature(float &tempC);
  float temperatureSampleRate(void);
  int temperatureAvailable(void);

  int16_t rawAccX, ///< Last reading's raw accelerometer X axis
      rawAccY,     ///< Last reading's raw accelerometer Y axis
      rawAccZ,     ///< Last reading's raw accelerometer Z axis
      rawTemp,     ///< Last reading's raw temperature reading
      rawGyroX,    ///< Last reading's raw gyro X axis
      rawGyroY,    ///< Last reading's raw gyro Y axis
      rawGyroZ,    ///< Last reading's raw gyro Z axis
      rawHighGX,    ///< Last reading's raw high-g X axis
      rawHighGY,    ///< Last reading's raw high-g Y axis
      rawHighGZ;    ///< Last reading's raw high-g Z axis

  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in deg/s
      gyroY,         ///< Last reading's gyro Y axis in deg/s
      gyroZ,         ///< Last reading's gyro Z axis in deg/s
      highGX,        ///< Last reading's accelerometer X axis m/s^2
      highGY,        ///< Last reading's accelerometer Y axis m/s^2
      highGZ;        ///< Last reading's accelerometer Z axis m/s^2

  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getAccelerometerSensor(void);
  Adafruit_Sensor *getGyroSensor(void);
  Adafruit_Sensor *getHighGSensor(void);

  virtual void _read(void);
  virtual void _readFast(void);

  //Fifo
  lsm6_fifo_tag_t parseFifoTag(uint16_t x) {return((lsm6_fifo_tag_t)((x & 0b11111000) >> 3));}
  uint8_t parseFifoTimeSlot(uint16_t x) {return((x & 0b110) >> 1);}
  uint8_t getFifoStatus(); //status bits only
  int getFifoAvailable(); //number of entries available to read
  bool getFifo(uint8_t *data); //the actual tagged data
  void configFifoInt1(bool threshold = false, bool overflow = false, bool full = false, bool batch = false); //threshold interrupt when there are N sensor readings in the fifo, batch interrupt when there are N readings of one sensor out of accel/gyro/EIS gyro/High-G accel, not counting the other data in the fifo
  void configFifoInt2(bool threshold = false, bool overflow = false, bool full = false, bool batch = false); 
  void setFifoAccelRate(lsm6dsv_data_rate_t data_rate);
  void setFifoGyroRate(lsm6dsv_data_rate_t data_rate);
  void setFifoTempRate(lsm6dsv_data_rate_t data_rate); //temperature can only use the 3 slowest rates 1.875Hz, 15Hz, 60Hz. LSM6DSV_RATE_SHUTDOWN for no temperature in fifo.
  void setFifoTSDec(lsm6_fifo_dec_ts_t dec); //divides the rate of accel or gyro
  void setFifoGyroEISRate(bool en = false); //simple enable/disable
  void setFifoHighGRate(bool en = false); //simple enable/disable
  void setFifoMode(lsm6_fifo_mode_t mode);
  void setFifoWatermark(uint8_t wtm);
  void setFifoCounterBDR(lsm6_fifo_batch_trig_t trig, uint16_t threshold);
  void setFifoEmbFunc(bool sflpGame = false, bool sflpGravity = false, bool sflpGBias = false, bool stepCounter = false, bool mlc = false);

protected:
  uint8_t chipID(void);
  uint8_t status(void);
  virtual bool _init(int32_t sensor_id);
  void funcCfgAccess(bool embFunc = false, bool sHub = false, bool FSM = false, bool globalReset = false);

  uint16_t _sensorid_accel, ///< ID number for accelerometer
      _sensorid_gyro,       ///< ID number for gyro
      _sensorid_temp,       ///< ID number for temperature
      _sensorid_high_g;     ///< ID number for high-g accelerometer

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  Adafruit_LSM6DSV_Temp *temp_sensor = NULL;           ///< Temp sensor data object
  Adafruit_LSM6DSV_Accelerometer *accel_sensor = NULL; ///< Accelerometer data object
  Adafruit_LSM6DSV_Gyro *gyro_sensor = NULL;           ///< Gyro data object
  Adafruit_LSM6DSV_HighG *high_g_sensor = NULL;        ///< high-g data object 

  //! buffer for the accelerometer range
  lsm6dsv_accel_range_t accelRangeBuffered = LSM6DSV_ACCEL_RANGE_2_G;
  //! buffer for the gyroscope range
  lsm6dsv_gyro_range_t gyroRangeBuffered = LSM6DSV_GYRO_RANGE_2000_DPS;
  //! buffer for the high-g accelerometer range
  lsm6dsv_accel_range_t accelHighGBuffered = LSM6DSV_ACCEL_RANGE_16_G;

  //store scale factor for each sensor instead of re-calculating it on every read
  //The magic numbers come from the datasheet, as milli-gees/sec or milli-degrees/sec
  float accelScale = 0.122 * SENSORS_GRAVITY_EARTH / 1000;        //convert raw to meters/sec/sec
  float gyroScale = 70.0 * SENSORS_DPS_TO_RADS / 1000;            //convert raw to radians/sec/sec
  float highGScale = 0.488 * SENSORS_GRAVITY_EARTH / 1000;        //convert raw to meters/sec/sec
  virtual void setAccelScale(void);
  void setGyroScale(void);
  virtual void setHighGScale(void) {};
  float temperature_sensitivity = 256.0; ///< Temp sensor sensitivity in LSB/degC

  bool _high_g_enabled = false;

private:
  friend class Adafruit_LSM6DSV_Temp; ///< Gives access to private members to
                                     ///< Temp data object
  friend class Adafruit_LSM6DSV_Accelerometer; ///< Gives access to private
                                              ///< members to Accelerometer data
                                              ///< object
  friend class Adafruit_LSM6DSV_Gyro; ///< Gives access to private members to
                                     ///< Gyro data object
  friend class Adafruit_LSM6DSV_HighG; ///< Gives access to private members to
                                     ///< HighG data object

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
  void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
  void fillHighGEvent(sensors_event_t *accel, uint32_t timestamp);
};

/*!
 *    @brief  Sub-class for the "B" version which has the TDM interface.
              The only difference that concerns us is the different chip ID
 */
class Adafruit_LSM6DSV16B : public Adafruit_LSM6DSV  {
public:
  Adafruit_LSM6DSV16B() {};
private:
  bool _init(int32_t sensor_id) {Adafruit_LSM6DSV::_init(sensor_id); return (chipID() == LSM6DSV16B_CHIP_ID);};
};

/*!
 *    @brief  Sub-class for the 32G version.
              The only difference that concerns us is the different accelerometer scale
 */
class Adafruit_LSM6DSV32 : public Adafruit_LSM6DSV  {
public:
  Adafruit_LSM6DSV32() {};
private:
  bool _init(int32_t sensor_id) {bool tmp = Adafruit_LSM6DSV::_init(sensor_id); highGScale = 0.976 * SENSORS_GRAVITY_EARTH / 1000; return tmp;};
  void setAccelScale(void) override {Adafruit_LSM6DSV::setAccelScale(); accelScale*=2;};
};

#endif
