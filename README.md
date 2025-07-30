# DFRobot_RTU

* [English Version](./README_CN.md)

This is a library for BMP58X, which reads temperature and pressure. BMP(585/581) is a digital sensor for pressure and temperature measurement based on reliable sensing principles.

![正反面svg效果图](https://github.com/Arya11111/DFRobot_MCP23017/blob/master/resources/images/SEN0245svg1.png)


## Product Link（链接到英文商城）
    
   
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
* This library supports BMP585/581 sensors.​
* This library supports reading temperature and pressure.​
* This library supports setting the sensor's working mode.​
* This library supports setting the sensor's output data rate.​
* This library supports setting the sensor's oversampling rate.​
* This library supports setting the sensor's IIR filter coefficients.​
* This library supports setting the sensor's FIFO operations.​
* This library supports setting the sensor's interrupt behavior.​
* This library supports setting the sensor's pressure out-of-range detection.​
* This library supports reading the sensor's interrupt status.

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
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
uint8_t setODR(eODR_t odr);

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
 * @n - eIntPressureOor: Pressure out-of-range interrupt
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
 * @n - eIntStatusPressureOor: Pressure out-of-range (0x08)
 * @n - eIntStatusResetComplete: Reset complete (0x10)
 */
uint16_t getIntStatus(void);

/**
 * @fn setOORPress
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
uint8_t setOORPress(uint32_t oor, uint8_t range, eOORCountLimit_t cntLimit);

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

```

## Compatibility


| MCU                | Work Well | Work Wrong | Untested | Remarks |
| ------------------ |:---------:|:----------:|:--------:| ------- |
| Arduino uno        |          |            | √         |         |
| FireBeetle esp32   |          |            | √         |         |
| FireBeetle esp8266 |          |            | √         |         |
| FireBeetle m0      |          |            | √         |         |
| Leonardo           |          |            | √         |         |
| Microbit           |          |            | √         |         |
| Arduino MEGA2560   |          |            | √         |         |


## History

- Data 2025-06-06
- Version V1.0

## Credits

Written by(yuanlong.yu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
