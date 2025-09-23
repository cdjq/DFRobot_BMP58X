/**
 * @file  DFRobot_BMP58X.cpp
 * @brief  Define the infrastructure of DFRobot_BMP58X class
 * @n      This is a pressure and temperature sensor that can be controlled through I2C/SPI/UART ports.
 * @n      BMP (581/585) has functions such as temperature compensation, data oversampling, IIR filtering, etc
 * @n      These features improve the accuracy of data collected by BMP (581/585) sensors.
 * @n      BMP (581/585) also has a FIFO data buffer, greatly improving its availability.
 * @n      Similarly, BMP (581/585) has an interrupt pin that can be used in an energy-efficient manner without using
 * software algorithms.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version     V1.0.0
 * @date        2025-07-23
 * @url         https://github.com/DFRobot/DFRobot_BMP58X
 */
#ifndef __DFRobot_BMP58X_H
#define __DFRobot_BMP58X_H
#include "Arduino.h"
#include "DFRobot_RTU.h"
#include "Wire.h"
#include "stdint.h"
#include <SPI.h>

#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...)                                                                                                       \
  {                                                                                                                    \
    Serial.print("[");                                                                                                 \
    Serial.print(__FUNCTION__);                                                                                        \
    Serial.print("(): ");                                                                                              \
    Serial.print(__LINE__);                                                                                            \
    Serial.print(" ] ");                                                                                               \
    Serial.println(__VA_ARGS__);                                                                                       \
  }
#else
#define DBG(...)
#endif

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

class DFRobot_BMP58X {
 private:
#define BMP5_REG_BIT_MASK(POS, WIDTH) (((1 << (WIDTH)) - 1) << (POS))
#define BMP5_REG_SET_BITS(reg, prefix, val)                                                                            \
  ((reg) = ((reg) & ~BMP5_REG_BIT_MASK(prefix##_POS, prefix##_WIDTH)) | ((val) << (prefix##_POS)))
#define BMP5_REG_GET_BITS(reg, prefix, type)                                                                           \
  static_cast<type>((((reg) >> (prefix##_POS)) & ((1 << (prefix##_WIDTH)) - 1)))
#define REG_DATA_LEN_MAX 0x04

  /** REGISTER TABLE */
#define REG_I_CHIP_ID 0x01
#define REG_I_REV_ID 0x02
#define REG_I_CHIP_STATUS 0x11
#define REG_H_DRIVE_CONFIG 0x13
#define REG_H_INT_CONFIG 0x14
#define REG_H_INT_SOURCE 0x15
#define REG_H_FIFO_CONFIG 0x16
#define REG_I_FIFO_COUNT 0x17
#define REG_H_FIFO_SEL 0x18
#define REG_I_TEMP_DATA_XLSB 0x1D
#define REG_I_TEMP_DATA_LSB 0x1E
#define REG_I_TEMP_DATA_MSB 0x1F
#define REG_I_PRESS_DATA_XLSB 0x20
#define REG_I_PRESS_DATA_LSB 0x21
#define REG_I_PRESS_DATA_MSB 0x22
#define REG_I_INT_STATUS 0x27
#define REG_I_STATUS 0x28
#define REG_I_FIFO_DATA 0x29
#define REG_H_NVM_ADDR 0x2B
#define REG_H_NVM_DATA_LSB 0x2C
#define REG_H_NVM_DATA_MSB 0x2D
#define REG_H_DSP_CONFIG 0x30
#define REG_H_DSP_IIR 0x31
#define REG_H_OOR_THR_P_LSB 0x32
#define REG_H_OOR_THR_P_MSB 0x33
#define REG_H_OOR_RANGE 0x34
#define REG_H_OOR_CONFIG 0x35
#define REG_H_OSR_CONFIG 0x36
#define REG_H_ODR_CONFIG 0x37
#define REG_I_OSR_EFF 0x38
#define REG_H_CMD 0x7E

#define REG_I_MODBUS_PID 0x00
#define REG_I_MODBUS_VID 0x01
#define REG_H_MODBUS_BAUD 0x01
#define OFFSET_REG 0x06

/** ENBALE */
#define ENABLE 0x01
#define DISABLE 0x00

#define DEEP_ENABLE 0x00
#define DEEP_DISABLE 0x01

#define BMP581_CHIP_ID 0x50
#define BMP585_CHIP_ID 0x51

/*! @name Soft reset command */
#define SOFT_RESET_CMD UINT8_C(0xB6)

  /** Definition of Position and Width Macros */
  // REG_INT_CONFIG (0x14)
#define INT_MODE_POS 0
#define INT_MODE_WIDTH 1
#define INT_POL_POS 1
#define INT_POL_WIDTH 1
#define INT_OD_POS 2
#define INT_OD_WIDTH 1
#define INT_EN_POS 3
#define INT_EN_WIDTH 1
#define PAD_INT_DRV_POS 4
#define PAD_INT_DRV_WIDTH 4

// REG_INT_SRC_SEL (0x15)
#define DRDY_DATA_REG_EN_POS 0
#define DRDY_DATA_REG_EN_WIDTH 1
#define FIFO_FULL_EN_POS 1
#define FIFO_FULL_EN_WIDTH 1
#define FIFO_THS_EN_POS 2
#define FIFO_THS_EN_WIDTH 1
#define OOR_P_EN_POS 3
#define OOR_P_EN_WIDTH 1

// REG_FIFO_CONFIG (0x16)
#define FIFO_THRESHOLD_POS 0
#define FIFO_THRESHOLD_WIDTH 5
#define FIFO_MODE_POS 5
#define FIFO_MODE_WIDTH 1

// REG_FIFO_COUNT (0x17)
#define FIFO_COUNT_POS 0
#define FIFO_COUNT_WIDTH 6

// REG_FIFO_SEL_CONFIG (0x18)
#define FIFO_FRAME_SEL_POS 0
#define FIFO_FRAME_SEL_WIDTH 2
#define FIFO_DEC_SEL_POS 2
#define FIFO_DEC_SEL_WIDTH 3

// REG_NVM_ADDRESS (0x2B)
#define NVM_ROW_ADDRESS_POS 0
#define NVM_ROW_ADDRESS_WIDTH 6
#define NVM_PROG_EN_POS 6
#define NVM_PROG_EN_WIDTH 1

// REG_DSP_CONFIG (0x30)
#define IIR_FLUSH_FORCED_EN_POS 2
#define IIR_FLUSH_FORCED_EN_WIDTH 1
#define SHDW_SEL_IIR_T_POS 3
#define SHDW_SEL_IIR_T_WIDTH 1
#define FIFO_SEL_IIR_T_POS 4
#define FIFO_SEL_IIR_T_WIDTH 1
#define SHDW_SEL_IIR_P_POS 5
#define SHDW_SEL_IIR_P_WIDTH 1
#define FIFO_SEL_IIR_P_POS 6
#define FIFO_SEL_IIR_P_WIDTH 1
#define OOR_SEL_IIR_P_POS 7
#define OOR_SEL_IIR_P_WIDTH 1

// REG_DSP_IIR_CONFIG (0x31)
#define SET_IIR_T_POS 0
#define SET_IIR_T_WIDTH 3
#define SET_IIR_P_POS 3
#define SET_IIR_P_WIDTH 3

// REG_OOR_CONFIG (0x35)
#define OOR_THR_P_16_POS 0
#define OOR_THR_P_16_WIDTH 1
#define CNT_LIM_POS 6
#define CNT_LIM_WIDTH 2

// REG_OSR_CONFIG (0x36)
#define OSR_T_POS 0
#define OSR_T_WIDTH 3
#define OSR_P_POS 3
#define OSR_P_WIDTH 3
#define PRESS_EN_POS 6
#define PRESS_EN_WIDTH 1

// REG_ODR_CONFIG (0x37)
#define PWR_MODE_POS 0
#define PWR_MODE_WIDTH 2
#define ODR_POS 2
#define ODR_WIDTH 5
#define DEEP_DIS_POS 7
#define DEEP_DIS_WIDTH 1

// REG_EFF_OSR_CONFIG (0x38)
#define OSR_T_EFF_POS 0
#define OSR_T_EFF_WIDTH 3
#define OSR_P_EFF_POS 3
#define OSR_P_EFF_WIDTH 3
#define ODR_IS_VALID_POS 7
#define ODR_IS_VALID_WIDTH 1

#define STANDARD_SEA_LEVEL_PRESSURE_PA 101325 ///< Standard sea level pressure, unit: pa

 public:
#define RET_CODE_OK 0
#define RET_CODE_ERROR 1

  DFRobot_BMP58X(/* args */);
  ~DFRobot_BMP58X();

  /**
   * @enum eEnable_t
   * @brief Enumerates enable/disable states.
   * @details Simple on/off states for basic functionality control.
   */
  typedef enum {
    eDisable,
    eEnable,
  } eEnable_t;

  /**
   * @enum eDeepEnable_t
   * @brief Enumerates deep enable/disable states.
   * @details Similar to eEnable_t but for deeper power management or initialization levels.
   */
  typedef enum {
    eDeepEnable,
    eDeepDisable,
  } eDeepEnable_t;

  /**
   * @enum eOdr_t
   * @brief Enumerates output data rate (ODR) settings.
   * @details Defines possible sampling frequencies for the sensor.
   * @note Values are in Hz and are defined with a 0x00U base.
   */
  typedef enum {
    eOdr240Hz = 0x00U,
    eOdr218_5Hz,
    eOdr199_1Hz,
    eOdr179_2Hz,
    eOdr160Hz,
    eOdr149_3Hz,
    eOdr140Hz,
    eOdr129_8Hz,
    eOdr120Hz,
    eOdr110_1Hz,
    eOdr100_2Hz,
    eOdr89_6Hz,
    eOdr80Hz,
    eOdr70Hz,
    eOdr60Hz,
    eOdr50Hz,
    eOdr45Hz,
    eOdr40Hz,
    eOdr35Hz,
    eOdr30Hz,
    eOdr25Hz,
    eOdr20Hz,
    eOdr15Hz,
    eOdr10Hz,
    eOdr5Hz,
    eOdr4Hz,
    eOdr3Hz,
    eOdr2Hz,
    eOdr1Hz,
    eOdr0_5Hz,
    eOdr0_250Hz,
    eOdr0_125Hz,
  } eOdr_t;

  /**
   * @enum eOverSampling_t
   * @brief Enumerates oversampling settings.
   * @details Controls the number of samples averaged per measurement for noise reduction.
   * @note Higher oversampling improves resolution but increases power consumption.
   */
  typedef enum {
    eOverSampling1 = 0x00U,
    eOverSampling2,
    eOverSampling4,
    eOverSampling8,
    eOverSampling16,
    eOverSampling32,
    eOverSampling64,
    eOverSampling128,
  } eOverSampling_t;

  /**
   * @enum eIIRFilter_t
   * @brief Enumerates IIR filter coefficients.
   * @details Controls the cutoff frequency of the Infinite Impulse Response (IIR) filter.
   * @note Higher filter coefficients provide stronger noise filtering but slower response times.
   */
  typedef enum {
    eFilterBypass = 0x00U,
    eFilter1,
    eFilter3,
    eFilter7,
    eFilter15,
    eFilter31,
    eFilter63,
    eFilter127,
  } eIIRFilter_t;

  /**
   * @enum eOORCountLimit_t
   * @brief Enumerates out-of-range (OOR) count limits.
   * @details Defines the number of consecutive out-of-range measurements required to trigger an OOR event.
   */
  typedef enum {
    eOORCountLimit1 = 0x00U,
    eOORCountLimit3,
    eOORCountLimit7,
    eOORCountLimit15,
  } eOORCountLimit_t;

  /**
   * @enum eMeasureMode_t
   * @brief Enumerates measurement modes.
   * @details Defines different operational modes for the sensor.
   */
  typedef enum {
    eSleep,
    eNormal,
    eSingleShot,
    eContinuous,
    eDeepSleep,
  } eMeasureMode_t;

  /**
   * @enum eFIFODataSel_t
   * @brief Enumerates FIFO data selection modes.
   * @details Controls which data types are stored in the First-In-First-Out (FIFO) buffer.
   */
  typedef enum {
    /*! FIFO disabled */
    eFIFODisable,
    /*! FIFO temperature data only enabled */
    eFIFOTempData,
    /*! FIFO pressure data only enabled */
    eFIFOPressData,
    /*! FIFO pressure and temperature data enabled */
    eFIFOPressAndTempData
  } eFIFODataSel_t;

  /**
   * @enum eFIFODownSampling_t
   * @brief Enumerates FIFO downsampling factors.
   * @details Defines the downsampling ratio applied to data before storing in the FIFO.
   * @note Values represent a division factor of 2^n.
   */
  /** 2^fifo_dec_sel */
  typedef enum {
    eNoDownSampling = 0x00U,
    eDownSampling2,
    eDownSampling4,
    eDownSampling8,
    eDownSampling16,
    eDownSampling32,
    eDownSampling64,
    eDownSampling128,
  } eFIFODownSampling_t;

  /**
   * @enum eFIFOWorkMode_t
   * @brief Enumerates FIFO operating modes.
   * @details Defines how the FIFO buffer behaves when it reaches capacity.
   */
  typedef enum { eFIFOOverwriteMode, eFIFOFullStopMode } eFIFOWorkMode_t;

  /**
   * @enum eIntStatus_t
   * @brief Enumerates interrupt status flags.
   * @details Bitmask values representing possible interrupt status conditions.
   */
  typedef enum {
    eIntStatusDataReady = 0x01,
    eIntStatusFIFOFull = 0x02,
    eIntStatusFIFOThres = 0x04,
    eIntStatusPressureOOR = 0x08,
    eIntStatusResetComplete = 0x10,
  } eIntStatus_t;

  /**
   * @enum eIntMode_t
   * @brief Enumerates interrupt trigger modes.
   * @details Defines whether interrupts are pulsed or latched.
   */
  typedef enum { eIntModePulsed = 0x00, eIntModeLatched = 0x01 } eIntMode_t;

  /**
   * @enum eIntPolarity_t
   * @brief Enumerates interrupt polarity options.
   * @details Defines whether interrupt signals are active low or high.
   */
  typedef enum { eIntLowActive = 0x00, eIntHighActive = 0x01 } eIntPolarity_t;

  /**
   * @enum eIntOutputMode_t
   * @brief Enumerates interrupt output modes.
   * @details Defines the electrical output type for interrupt signals.
   */
  typedef enum { eIntPushPull = 0x00, eIntOpenDrain = 0x01 } eIntOutputMode_t;

  /**
   * @enum eIntEnable_t
   * @brief Enumerates interrupt enable states.
   * @details Simple on/off states for interrupt functionality.
   */
  typedef enum { eIntDisable = 0x00, eIntEnable = 0x01 } eIntEnable_t;

  /**
   * @enum eIntSource_t
   * @brief Enumerates interrupt source options.
   * @details Bitmask values representing possible interrupt sources that can be enabled.
   */
  typedef enum {
    eIntDataReady = 0x01,
    eIntFIFOFull = 0x02,
    eIntFIFOThres = 0x04,
    eIntPressureOOR = 0x08,
  } eIntSource_t;

  /**
   * @struct sFIFOData_t
   * @brief Structure for storing FIFO buffer data from the pressure sensor.
   * @details Holds the data retrieved from the sensor's First-In-First-Out (FIFO) buffer,
   *          including both pressure and temperature readings.
   * @note The `len` field indicates the number of valid entries in both data arrays.
   *       Both arrays are sized to hold a maximum of 32 samples, which matches the typical
   *       FIFO depth of the sensor.
   */
  typedef struct {
    uint8_t len;
    float fifoPressPa[32];
    float fifoTempC[32];
  } sFIFOData_t;

  /**
   * @fn begin
   * @brief Initializes the sensor hardware interface
   * @return true if initialization succeeds, false on failure
   */
  bool begin(void);

  /**
   * @fn setODR
   * @brief Configures sensor output data rate (ODR)
   * @param odr Output data rate selection (see: eOdr_t)
   * @n Available rates:
   * @n - eOdr240Hz:    Indicates an output rate of 240 Hz
   * @n - eOdr218_5Hz:  Indicates an output rate of 218.5 Hz
   * @n - eOdr199_1Hz:  Indicates an output rate of 199.1 Hz
   * @n - eOdr179_2Hz:  Indicates an output rate of 179.2 Hz
   * @n - eOdr160Hz:    Indicates an output rate of 160 Hz
   * @n - eOdr149_3Hz:  Indicates an output rate of 149.3 Hz
   * @n - eOdr140Hz:    Indicates an output rate of 140 Hz
   * @n - eOdr129_8Hz:  Indicates an output rate of 129.8 Hz
   * @n - eOdr120Hz:    Indicates an output rate of 120 Hz
   * @n - eOdr110_1Hz:  Indicates an output rate of 110.1 Hz
   * @n - eOdr100_2Hz:  Indicates an output rate of 100.2 Hz
   * @n - eOdr89_6Hz:   Indicates an output rate of 89.6 Hz
   * @n - eOdr80Hz:     Indicates an output rate of 80 Hz
   * @n - eOdr70Hz:     Indicates an output rate of 70 Hz
   * @n - eOdr60Hz:     Indicates an output rate of 60 Hz
   * @n - eOdr50Hz:     Indicates an output rate of 50 Hz
   * @n - eOdr45Hz:     Indicates an output rate of 45 Hz
   * @n - eOdr40Hz:     Indicates an output rate of 40 Hz
   * @n - eOdr35Hz:     Indicates an output rate of 35 Hz
   * @n - eOdr30Hz:     Indicates an output rate of 30 Hz
   * @n - eOdr25Hz:     Indicates an output rate of 25 Hz
   * @n - eOdr20Hz:     Indicates an output rate of 20 Hz
   * @n - eOdr15Hz:     Indicates an output rate of 15 Hz
   * @n - eOdr10Hz:     Indicates an output rate of 10 Hz
   * @n - eOdr5Hz:      Indicates an output rate of 5 Hz
   * @n - eOdr4Hz:      Indicates an output rate of 4 Hz
   * @n - eOdr3Hz:      Indicates an output rate of 3 Hz
   * @n - eOdr2Hz:      Indicates an output rate of 2 Hz
   * @n - eOdr1Hz:      Indicates an output rate of 1 Hz
   * @n - eOdr0_5Hz:    Indicates an output rate of 0.5 Hz
   * @n - eOdr0_250Hz:  Indicates an output rate of 0.250 Hz
   * @n - eOdr0_125Hz:  Indicates an output rate of 0.125 Hz
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setODR(eOdr_t odr);

  /**
   * @fn setOSR
   * @brief Sets oversampling ratios for temperature and pressure
   * @param osrTemp Temperature oversampling (see: eOverSampling_t)
   * @param osrPress Pressure oversampling (see: eOverSampling_t)
   * @n Supported values:
   * @n - eOverSampling1:   1x oversampling
   * @n - eOverSampling2:   2x oversampling
   * @n - eOverSampling4:   4x oversampling
   * @n - eOverSampling8:   8x oversampling
   * @n - eOverSampling16:  16x oversampling
   * @n - eOverSampling32:  32x oversampling
   * @n - eOverSampling64:  64x oversampling
   * @n - eOverSampling128: 128x oversampling
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setOSR(eOverSampling_t osrTemp, eOverSampling_t osrPress);

  /**
   * @fn setMeasureMode
   * @brief Configures sensor power/measurement mode
   * @param mode Operation mode (see: eMeasureMode_t)
   * @n Available modes:
   * @n - eSleep:         Sleep mode
   * @n - eNormal:        Normal measurement mode
   * @n - eSingleShot:    Single-shot measurement
   * @n - eContinuous:    Continuous measurement
   * @n - eDeepSleep:     Deep Sleep mode
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setMeasureMode(eMeasureMode_t mode);

  /**
   * @fn reset
   * @brief Performs software reset of the sensor
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t reset(void);

  /**
   * @fn readTempC
   * @brief Reads calibrated temperature data
   * @return float Temperature in degrees Celsius
   */
  float readTempC(void);

  /**
   * @fn readPressPa
   * @brief Reads calibrated pressure data
   * @return float Pressure in Pascals (Pa)
   */
  float readPressPa(void);

  /**
   * @fn readAltitudeM
   * @brief Calculates altitude based on pressure reading
   * @note Uses formula:
   * @n altitude = (1 - (P/101325)^0.190284) * 44307.7
   * @n where P = current pressure in Pa
   * @return float Altitude in meters
   */
  float readAltitudeM(void);

  /**
   * @fn configIIR
   * @brief Configures IIR filter coefficients
   * @param iirTemp  Temperature IIR filter (see: eIIRFilter_t)
   * @param iirPress Pressure IIR filter (see: eIIRFilter_t)
   * @n Available coefficients:
   * @n - eFilterBypass:   Bypass filter
   * @n - eFilter1:        1st order filter
   * @n - eFilter3:        3rd order filter
   * @n - eFilter7:        7th order filter
   * @n - eFilter15:       15th order filter
   * @n - eFilter31:       31st order filter
   * @n - eFilter63:       63rd order filter
   * @n - eFilter127:      127th order filter
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t configIIR(eIIRFilter_t iirTemp, eIIRFilter_t iirPress);

  /**
   * @fn configFIFO
   * @brief Configures FIFO operation parameters
   * @param dataSel Data frame type (see: eFIFODataSel_t)
   * @n Available types:
   * @n - eFIFODisable:           FIFO disabled
   * @n - eFIFOTempData:          Temperature data only
   * @n - eFIFOPressData:         Pressure data only
   * @n - eFIFOPressAndTempData:  Pressure and temperature data
   *
   * @param downSampling Downsampling ratio (see: eFIFODownSampling_t)
   * @n Available ratios:
   * @n - eNoDownSampling:    No downsampling
   * @n - eDownSampling2:     2x downsampling
   * @n - eDownSampling4:     4x downsampling
   * @n - eDownSampling8:     8x downsampling
   * @n - eDownSampling16:    16x downsampling
   * @n - eDownSampling32:    32x downsampling
   * @n - eDownSampling64:    64x downsampling
   * @n - eDownSampling128:   128x downsampling
   *
   * @param mode FIFO operation mode (see: eFIFOWorkMode_t)
   * @n Available modes:
   * @n - eFIFOOverwriteMode:  Stream data continuously
   * @n - eFIFOFullStopMode:   Stop when FIFO full
   *
   * @param threshold FIFO trigger threshold (0=disable, 1-31=frames)]
   * @n - 0x0F: 15 frames. This is the maximum setting in PT-mode. The most
   * significant bit is ignored.
   * @n - 0x1F: 31 frames. This is the maximum setting in P- or T-mode.
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t configFIFO(eFIFODataSel_t dataSel, eFIFODownSampling_t downSampling, eFIFOWorkMode_t mode, uint8_t threshold);

  /**
   * @fn getFIFOCount
   * @brief Gets current number of frames in FIFO
   * @return uint8_t Number of stored data frames (0-31)
   */
  uint8_t getFIFOCount(void);

  /**
   * @fn getFIFOData
   * @brief Reads all data from FIFO
   * @return sFIFOData_t Struct containing pressure and temperature data
   * @n - len: Number of stored data frames (0-31)
   * @n - pressure: Array of pressure values
   * @n - temperature: Array of temperature values
   */
  sFIFOData_t getFIFOData(void);

  /**
   * @fn configInterrupt
   * @brief Configures interrupt behavior
   * @param intMode Trigger mode (see: eIntMode_t)
   * @n Available modes:
   * @n - eIntModePulsed: Pulsed mode
   * @n - eIntModeLatched: Latched mode
   *
   * @param intPol Signal polarity (see: eIntPolarity_t)
   * @n Available polarities:
   * @n - eIntLowActive:  Active low
   * @n - eIntHighActive: Active high
   *
   * @param intOd Output driver type (see: eIntOutputMode_t)
   * @n Available types:
   * @n - eIntPushPull: Push-pull output
   * @n - eIntOpenDrain: Open-drain output
   *
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t configInterrupt(eIntMode_t mode, eIntPolarity_t pol, eIntOutputMode_t outputMode);

  /**
   * @fn setIntSource
   * @brief Enables specific interrupt sources
   * @param source Bitmask of triggers (see: eIntSource_t)
   * @n Available sources:
   * @n - eIntDataReady:    Data ready interrupt
   * @n - eIntFIFOFull:    FIFO full interrupt
   * @n - eIntFIFOThres:   FIFO threshold interrupt
   * @n - eIntPressureOOR: Pressure out-of-range interrupt
   * @details You can combine multiple interrupt sources using bitwise OR (|).
   *          Example: Enable both data ready and FIFO full interrupts:
   *          @code
   *          setIntSource(bmp58x.eIntDataReady | bmp58x.eIntFIFOFull);
   *          @endcode
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setIntSource(uint8_t source);

  /**
   * @fn getIntStatus
   * @brief Reads current interrupt status flags
   * @return uint16_t Bitmask of active interrupts
   * @n Possible flags:
   * @n - eIntStatusDataReady: Data ready (0x01)
   * @n - eIntStatusFIFOFull: FIFO full  (0x02)
   * @n - eIntStatusFIFOThres: FIFO threshold reached (0x04)
   * @n - eIntStatusPressureOOR: Pressure out-of-range (0x08)
   * @n - eIntStatusResetComplete: Reset complete (0x10)
   */
  uint16_t getIntStatus(void);

  /**
   * @fn setPressOOR
   * @brief Configures pressure out-of-range detection
   * @param oor Threshold pressure value (0x00000-0x1FFFF)
   * @param range Hysteresis range (0-255)
   * @n oor - range < Pressure < oor + range
   * @param cntLimit Trigger persistence count
   * @n Available persistence settings:
   * @n - eOORCountLimit1:  1 count
   * @n - eOORCountLimit3:  3 counts
   * @n - eOORCountLimit7:  7 counts
   * @n - eOORCountLimit15: 15 counts
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setPressOOR(uint32_t oor, uint8_t range, eOORCountLimit_t cntLimit);

  /**
   * @fn calibratedAbsoluteDifference
   * @brief use the given current altitude as a reference value, eliminate the absolute difference of subsequent
   * pressure and altitude data
   * @param altitude current altitude
   * @return boolean, indicates whether the reference value is set successfully
   * @retval True indicates the reference value is set successfully
   * @retval False indicates fail to set the reference value
   */
  bool calibratedAbsoluteDifference(float altitude);

 private:
  /**
   * @fn enablePressure
   * @brief Enables/disables pressure measurement
   * @param enable Enable control (see ::eEnable_t)
   * @n Available options:
   * @n - eENABLE:  Enable pressure measurement
   * @n - eDisable: Disable pressure measurement
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t enablePressure(eEnable_t enable);

  /**
   * @fn setDeepStandbyMode
   * @brief Configures deep standby mode
   * @param deepMode Deep standby control
   * @n Valid parameters:
   * @n - eDeepEnable: Enter deep standby mode
   * @n - eDeepDisable: Exit deep standby mode
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setDeepStandbyMode(eDeepEnable_t deepMode);

  /**
   * @fn setPowerMode
   * @brief Sets sensor power operation mode
   * @param mode Power mode (see ::eMeasureMode_t)
   * @n Available modes:
   * @n - eSleep:         Sleep mode
   * @n - eNormal:        Normal measurement mode
   * @n - eSingleShot:    Single-shot measurement
   * @n - eContinuous:    Continuous measurement
   * @n - eDeepSleep:     Deep Sleep mode
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setPowerMode(eMeasureMode_t mode);

  /**
   * @fn getPowerMode
   * @brief Reads current power mode
   * @return eMeasureMode_t Current power mode (see ::eMeasureMode_t)
   * @n Possible return values:
   * @n - eSleep:         Sleep mode
   * @n - eNormal:        Normal measurement mode
   * @n - eSingleShot:    Single-shot measurement
   * @n - eContinuous:    Continuous measurement
   * @n - eDeepSleep:     Deep Sleep mode
   */
  eMeasureMode_t getPowerMode(void);

  /**
   * @fn verifyDeepSleepMode
   * @brief Verifies if sensor is in deep sleep
   * @return eMeasureMode_t Current mode with deep sleep status
   * @n Special return case:
   * @n - Returns eDeepSleep if sensor is unresponsive
   */
  eMeasureMode_t verifyDeepSleepMode(void);

  /**
   * @fn setFIFOThreshold
   * @brief Configures FIFO threshold with data pattern
   * @param data Pointer to threshold data pattern (16-bit array)
   * @param frame_sel Data frame type (see ::eFIFODataSel_t)
   * @n Supported types:
   * @n - eFIFODisable
   * @n - eFIFOTempData
   * @n - eFIFOPressData
   * @n - eFIFOPressAndTempData
   * @param threshold Trigger threshold (0-31 frames)
   * @return uint8_t 0 on success, 1 on error
   */
  uint8_t setFIFOThreshold(uint16_t *data, eFIFODataSel_t frame_sel, uint8_t threshold);

  /**
   * @fn calculateAltitude
   * @brief Calculates altitude with custom reference parameters
   * @param temperature_c Current temperature in Celsius
   * @param pressure_pa Current pressure in Pascals
   * @note Uses international barometric formula:
   * @n altitude = 44330 * [1 - (P/P0)^(1/5.255)]
   * @n where P0 = 101325 Pa (standard sea-level pressure)
   * @return float Calculated altitude in meters
   */
  float calculateAltitude(float temperature_c, float pressure_pa);
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len) = 0;
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len) = 0;
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len) = 0;

  float _sealevelAltitude = 0.0f;
  bool _calibrated = false;
};

class DFRobot_BMP58X_I2C : public DFRobot_BMP58X {
 private:
  TwoWire *_pWire;
  uint16_t _i2cAddr;
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len);
  bool readReg(uint8_t reg, void *data, uint8_t len);
  bool writeReg(uint8_t reg, void *data, uint8_t len);

 public:
  DFRobot_BMP58X_I2C(TwoWire *pWire, uint16_t addr);
  ~DFRobot_BMP58X_I2C(void);
  bool begin(void);
};

class DFRobot_BMP58X_SPI : public DFRobot_BMP58X {
 private:
  SPIClass *_pSpi;
  uint8_t _csPin;
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len);
  bool readReg(uint8_t reg, void *data, uint8_t len);
  bool writeReg(uint8_t reg, void *data, uint8_t len);

 public:
  DFRobot_BMP58X_SPI(SPIClass *pSpi, uint8_t csPin);
  ~DFRobot_BMP58X_SPI(void);
  bool begin(void);
};

class DFRobot_BMP58X_UART : public DFRobot_BMP58X, public DFRobot_RTU {
 private:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  SoftwareSerial *_serial;
#else
  HardwareSerial *_serial;
#endif
  uint32_t __baud;
  uint8_t __rxpin;
  uint8_t __txpin;
  uint16_t __addr;
  virtual uint8_t writeHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readHoldingReg(uint8_t reg, void *data, uint8_t len);
  virtual uint8_t readInputReg(uint8_t reg, void *data, uint8_t len);

 public:
  /**
   * @enum eBaud
   * @brief Enumerates standard UART baud rate settings.
   * @details Defines common communication speeds for serial communication interfaces.
   *          Values are represented in bits per second (bps).
   * @note The first value (e2400) is explicitly set to 0x0001,
   *       while subsequent values increment automatically.
   *       Actual hardware implementation may require mapping these values
   *       to specific configuration registers.
   */
  typedef enum {
    e2400 = 0x0001,
    e4800,
    e9600,
    e14400,
    e19200,
    e38400,
    e57600,
    e115200,
  } eBaud;
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  DFRobot_BMP58X_UART(SoftwareSerial *sSerial, uint32_t Baud, uint16_t addr);
#else
  DFRobot_BMP58X_UART(HardwareSerial *hSerial, uint32_t Baud, uint16_t addr, uint8_t rxpin = 0, uint8_t txpin = 0);
#endif
  ~DFRobot_BMP58X_UART(void);
  bool begin(void);
  /**
   * @fn setBaud
   * @brief Set the UART communication baud rate.
   * @details Configures the serial communication speed using the specified baud rate enum.
   *          This function initializes the necessary hardware registers to achieve the desired
   *          data transfer rate.
   * @param baud An eBaud enum value specifying the desired baud rate.
   *             Defaults to e9600 if not explicitly set.
   * @note Actual hardware configuration may vary depending on the microcontroller model.
   *       The function assumes a standard clock frequency; adjust clock settings
   *       if using non-default system clock configurations.
   * @warning Changing the baud rate during communication may cause data loss
   *          or communication errors if both devices are not synchronized.
   * @see eBaud for available baud rate options:
   *      - e2400: 2400 bits per second
   *      - e4800: 4800 bits per second
   *      - e9600: 9600 bits per second (default)
   *      - e14400: 14400 bits per second
   *      - e19200: 19200 bits per second
   *      - e38400: 38400 bits per second
   *      - e57600: 57600 bits per second
   *      - e115200: 115200 bits per second
   */
  void setBaud(eBaud baud);
};
#endif