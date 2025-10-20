# DFRobot_BMP58X

* [中文版](./README_CN.md)

This is a library for BMP58X, which reads temperature and pressure. BMP(585/581) is a digital sensor for pressure and temperature measurement based on reliable sensing principles.

### SEN0664
<p align="center">
  <img src="../../resources/images/SEN0664 (3).png" width="45%">
  <img src="../../resources/images/SEN0664 (4).png" width="45%">
</p>

### SEN0665
<p align="center">
  <img src="../../resources/images/SEN0665 (1).png" width="45%">
  <img src="../../resources/images/SEN0665 (2).png" width="45%">
</p>

### SEN0666
<p align="center">
  <img src="../../resources/images/SEN0666 (3).png" width="45%">
  <img src="../../resources/images/SEN0666 (4).png" width="45%">
</p>

### SEN0667
<p align="center">
  <img src="../../resources/images/SEN0667 (1).png" width="45%">
  <img src="../../resources/images/SEN0667 (2).png" width="45%">
</p>

## Product Link (https://www.dfrobot.com)
    SKU: SEN0664/SEN0665/SEN0666/SEN0667

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

```python

def begin(self):
'''!
  @fn begin
  @brief Initializes the sensor hardware interface
  @return true if initialization succeeds, false on failure
'''

def set_odr(self, odr):
'''!
  @fn set_odr
  @brief Configures sensor output data rate (ODR)
  @param odr Output data rate selection
  @n Available rates:
  @n - eODR_240_HZ:    240 Hz
  @n - eODR_218_5_HZ:  218.5 Hz
  @n - eODR_199_1_HZ:  199.1 Hz
  @n - eODR_179_2_HZ:  179.2 Hz
  @n - eODR_160_HZ:    160 Hz
  @n - eODR_149_3_HZ:  149.3 Hz
  @n - eODR_140_HZ:    140 Hz
  @n - eODR_129_8_HZ:  129.8 Hz
  @n - eODR_120_HZ:    120 Hz
  @n - eODR_110_1_HZ:  110.1 Hz
  @n - eODR_100_2_HZ:  100.2 Hz
  @n - eODR_89_6_HZ:   89.6 Hz
  @n - eODR_80_HZ:     80 Hz
  @n - eODR_70_HZ:     70 Hz
  @n - eODR_60_HZ:     60 Hz
  @n - eODR_50_HZ:     50 Hz
  @n - eODR_45_HZ:     45 Hz
  @n - eODR_40_HZ:     40 Hz
  @n - eODR_35_HZ:     35 Hz
  @n - eODR_30_HZ:     30 Hz
  @n - eODR_25_HZ:     25 Hz
  @n - eODR_20_HZ:     20 Hz
  @n - eODR_15_HZ:     15 Hz
  @n - eODR_10_HZ:     10 Hz
  @n - eODR_05_HZ:     5 Hz
  @n - eODR_04_HZ:     4 Hz
  @n - eODR_03_HZ:     3 Hz
  @n - eODR_02_HZ:     2 Hz
  @n - eODR_01_HZ:     1 Hz
  @n - eODR_0_5_HZ:    0.5 Hz
  @n - eODR_0_250_HZ:  0.250 Hz
  @n - eODR_0_125_HZ:  0.125 Hz
  @return true if configuration succeeds, false on failure
'''

def set_osr(self, osr_t, osr_p):
'''!
  @fn set_osr
  @brief Sets oversampling ratios for temperature and pressure
  @param osr_temp Temperature oversampling
  @param osr_press Pressure oversampling
  @n Supported values:
  @n - OVERSAMPLING_1X:   1x oversampling
  @n - OVERSAMPLING_2X:   2x oversampling
  @n - OVERSAMPLING_4X:   4x oversampling
  @n - OVERSAMPLING_8X:   8x oversampling
  @n - OVERSAMPLING_16X:  16x oversampling
  @n - OVERSAMPLING_32X:  32x oversampling
  @n - OVERSAMPLING_64X:  64x oversampling
  @n - OVERSAMPLING_128X: 128x oversampling
  @return true if configuration succeeds, false on failure
'''

def set_measure_mode(self, mode):
'''!
  @fn setMeasureMode
  @brief  set the measurement mode of the sensor
  @param  mode: measurement mode
  @n      SLEEP_MODE = 0x00.        #// standby mode
  @n      NORMAL_MODE = 0x01.        #// normal mode
  @n      SINGLE_SHOT_MODE = 0x02.         #// forced mode > only perform once
  @n      CONTINOUS_MODE = 0x03.      #// continuous mode
  @n      DEEP_SLEEP_MODE = 0x04.   #// deep standby mode
  @return True if the setting is successful, False otherwise.
'''

def reset(self):
'''!
  @fn reset
  @brief  reset the sensor
  @return True if the reset is successful, False otherwise
'''

def get_temperature(self):
'''!
  @fn get_temperature
  @brief  get the temperature of the sensor
  @return temperature in Celsius
'''

def get_pressure(self):
'''!
  @fn get_pressure
  @brief  get the pressure of the sensor
  @return pressure in Pascal
'''

def get_altitude(self):
'''!
  @fn get_altitude
  @brief  get the altitude of the sensor
  @return altitude in meters
'''

def config_iir(self, iir_t, iir_p):
'''!
  @fn config_iir
  @brief Configures IIR filter coefficients
  @param iir_t Temperature IIR filter
  @param iir_p Pressure IIR filter
  @n Available coefficients:
  @n - IIR_FILTER_BYPASS:   Bypass filter
  @n - IIR_FILTER_COEFF_1:  1st order filter
  @n - IIR_FILTER_COEFF_3:  3rd order filter
  @n - IIR_FILTER_COEFF_7:  7th order filter
  @n - IIR_FILTER_COEFF_15: 15th order filter
  @n - IIR_FILTER_COEFF_31: 31st order filter
  @n - IIR_FILTER_COEFF_63: 63rd order filter
  @n - IIR_FILTER_COEFF_127:127th order filter
  @return True if configuration is successful, False otherwise
'''

def config_fifo(self, frame_sel=eFIFO_PRESS_TEMP_DATA, dec_sel=eFIFO_NO_DOWNSAMPLING, mode=eFIFO_STREAM_TO_FIFO_MODE, threshold=0x00):
'''!
  @fn config_fifo
  @brief Configures FIFO operation parameters
  @param frame_sel Data frame type
  @n Available types:
  @n - FIFO_DISABLED:    FIFO disabled
  @n - FIFO_TEMPERATURE_DATA: Temperature data only
  @n - FIFO_PRESSURE_DATA:    Pressure data only
  @n - FIFO_PRESS_TEMP_DATA:  Pressure and temperature data

  @param dec_sel Downsampling ratio
  @n Available ratios:
  @n - FIFO_NO_DOWNSAMPLING: No downsampling
  @n - FIFO_DOWNSAMPLING_2X:  2x downsampling
  @n - FIFO_DOWNSAMPLING_4X:  4x downsampling
  @n - FIFO_DOWNSAMPLING_8X:  8x downsampling
  @n - FIFO_DOWNSAMPLING_16X: 16x downsampling
  @n - FIFO_DOWNSAMPLING_32X: 32x downsampling
  @n - FIFO_DOWNSAMPLING_64X: 64x downsampling
  @n - FIFO_DOWNSAMPLING_128X:128x downsampling

  @param mode FIFO operation mode
  @n Available modes:
  @n - FIFO_STREAM_TO_FIFO_MODE: Stream data continuously
  @n - FIFO_STOP_ON_FULL_MODE:   Stop when FIFO full

  @param threshold FIFO trigger threshold (0=disable, 1-31=frames)]
  @n - 0x0F: 15 frames. This is the maximum setting in PT-mode. The most
  significant bit is ignored.
  @n - 0x1F: 31 frames. This is the maximum setting in P- or T-mode.
  @return True if configuration is successful, False otherwise.
'''

def get_fifo_count(self):
'''!
  @fn get_fifo_count
  @brief  Get the number of frames in the FIFO.
  @return Number of frames in the FIFO.
'''

def get_fifo_data(self):
'''!
  @fn getFIFOData
  @brief Reads all data from FIFO
  @return sFIFOData_t Struct containing pressure and temperature data
  @n - len: Number of stored data frames (0-31)
  @n - pressure: Array of pressure values
  @n - temperature: Array of temperature values
'''

def config_interrupt(self, int_mode, int_pol, int_od):
'''!
  @fn config_interrupt
  @brief Configures interrupt behavior
  @param int_mode Trigger mode
  @n Available modes:
  @n - INT_MODE_PULSED: Pulsed mode
  @n - INT_MODE_LATCHED: Latched mode

  @param int_pol Signal polarity
  @n Available polarities:
  @n - INT_POL_ACTIVE_LOW: Active low
  @n - INT_POL_ACTIVE_HIGH: Active high

  @param int_od Output driver type
  @n Available types:
  @n - INT_OD_PUSH_PULL: Push-pull output
  @n - INT_OD_OPEN_DRAIN: Open-drain output
  @return True if configuration is successful, False otherwise.
'''

def set_int_source(self, source):
'''!
  @fn set_int_source
  @brief Enables specific interrupt sources
  @param source Bitmask of triggers
  @n Available sources:
  @n - eINT_DATA_DRDY:    Data ready interrupt
  @n - eINT_FIFO_FULL:    FIFO full interrupt
  @n - eINT_FIFO_THRES:   FIFO threshold interrupt
  @n - eINT_PRESSURE_OOR: Pressure out-of-range interrupt
  @return True if configuration is successful, False otherwise.
'''

def get_int_status(self):
'''!
  @brief  Get the interrupt status of the sensor.
  @return Interrupt status.
  @n      eINT_STATUS_DRDY = 0x01,                   # // data ready
  @n      eINT_STATUS_FIFO_FULL = 0x02,              # // FIFO full
  @n      eINT_STATUS_FIFO_THRES = 0x04,             # // FIFO threshold
  @n      eINT_STATUS_PRESSURE_OOR = 0x08,           # // pressure out of range
  # // power on reset/soft reset complete
  @n      eINT_STATUS_POR_SOFTRESET_COMPLETE = 0x10,
'''

def set_oor_press(self, oor, range_val, cnt_lim):
'''!
  @brief  Set the out of range_val pressure of the sensor.
  @param  oor: Out of range_val pressure.
  @n      0x00000 - 0x1FFFF: Out of range_val pressure
  @param  range_val: Out of range_val pressure range_val.
  @n      0x00 - 0xFF: Out of range_val pressure range_val  (oor - range_val, oor + range_val)
  @param  cnt_lim: Out of range_val pressure count limit.
  @n      eOOR_COUNT_LIMIT_1  = 0x00
  @n      eOOR_COUNT_LIMIT_3  = 0x01
  @n      eOOR_COUNT_LIMIT_7  = 0x02
  @n      eOOR_COUNT_LIMIT_15 = 0x03
  @return True if configuration is successful, False otherwise.
'''

def calibrated_absolute_difference(self, altitude):
'''!
  @fn calibratedAbsoluteDifference
  @brief use the given current altitude as a reference value, eliminate the absolute difference of subsequent pressure and altitude data
  @param altitude current altitude
  @return boolean, indicates whether the reference value is set successfully
  @retval True indicates the reference value is set successfully
  @retval False indicates fail to set the reference value
'''

def set_baud(self, baud):
'''!
  @fn setBaud
  @brief Set the UART communication baud rate.
  @details Configures the serial communication speed using the specified baud rate enum.
          This function initializes the necessary hardware registers to achieve the desired
          data transfer rate.
  @param baud An eBaud enum value specifying the desired baud rate.
              Defaults to e9600 if not explicitly set.
  @note Actual hardware configuration may vary depending on the microcontroller model.
        The function assumes a standard clock frequency; adjust clock settings
        if using non-default system clock configurations.
  @warning Changing the baud rate during communication may cause data loss
          or communication errors if both devices are not synchronized.
  @see eBaud for available baud rate options:
      - BAUD_2400: 2400 bits per second
      - BAUD_4800: 4800 bits per second
      - BAUD_9600: 9600 bits per second (default)
      - BAUD_14400: 14400 bits per second
      - BAUD_19200: 19200 bits per second
      - BAUD_38400: 38400 bits per second
      - BAUD_57600: 57600 bits per second
      - BAUD_115200: 115200 bits per second
'''
```

## Compatibility


* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |           |            |     √    |         |
| Python3 |     √     |            |          |         |

## History

- Data 2025-09-23
- Version V1.0

## Credits

Written by(yuanlong.yu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
