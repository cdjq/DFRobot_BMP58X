# -*- coding: utf-8 -*-
'''!
  @file  interrupt_using_fifo.py
  @brief  Acquire temperature/pressure data via FIFO threshold or FIFO full interrupts
  @details  Demonstrate data acquisition from BMP58X using:
            - FIFO threshold interrupt (when FIFO reaches configured frame count)
            - FIFO full interrupt (when FIFO buffer is completely filled)
            - Data is read from FIFO buffer upon interrupt trigger
  @copyright  Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     yuanlong.yu(yuanlong.yu@dfrobot.com)
  @version    V1.0.0
  @date       2025-06-06
  @url        https://github.com/DFRobot/DFRobot_BMP58X
'''

import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_BMP58X import *

DEV_ADDR   = 0x47

# Please choose your communication method below:
# mode = "UART"
# mode = "SPI"
mode = "I2C"

# Configure the interrupt mode you need to use:
# Using Ture is a latch interrupt, otherwise it is a pulse interrupt
BMP5_INT_MODE_LATCHED = True

#If there is no need to eliminate the absolute measurement difference, please set it to False
CALIBRATE_ABSOLUTE_DIFFERENCE = True

if mode == "I2C":
    I2C_BUS    = 0x01
    bmp5 = DFRobot_BMP58X_I2C(I2C_BUS, DEV_ADDR)
elif mode == "SPI":
    CS         = 8
    bmp5 = DFRobot_BMP58X_SPI(cs=CS, bus=0, dev=0, speed=8000000)
elif mode == "UART":
    bmp5 = DFRobot_BMP58X_UART(9600, DEV_ADDR)

global interrupt_flag
interrupt_flag = False

def int_callback(channel):
    global interrupt_flag
    interrupt_flag = True

gpio_interrupt = 27
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_interrupt, GPIO.IN)
GPIO.add_event_detect(gpio_interrupt, GPIO.RISING, callback=int_callback)

def setup():
    while not bmp5.begin():
        print("sensor init error,please check connect!")
        time.sleep(1)
    
    '''!
      @brief Configures FIFO operation parameters
      @param frame_sel Data frame type
      @n Available types:
      @n - FIFO_NOT_ENABLED:    FIFO disabled
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
    '''
    bmp5.config_fifo(bmp5.FIFO_PRESS_TEMP_DATA, bmp5.FIFO_NO_DOWNSAMPLING, bmp5.FIFO_STREAM_TO_FIFO_MODE, 0x02)

    '''!
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
    '''
    if BMP5_INT_MODE_LATCHED:
        bmp5.config_interrupt(int_mode=bmp5.INT_MODE_LATCHED,
                              int_pol=bmp5.INT_POL_ACTIVE_HIGH,
                              int_od=bmp5.INT_OD_PUSH_PULL)
    else:
        bmp5.config_interrupt(int_mode=bmp5.INT_MODE_PULSED,
                            int_pol=bmp5.INT_POL_ACTIVE_HIGH,
                            int_od=bmp5.INT_OD_PUSH_PULL)
    '''!
      @brief Enables specific interrupt sources
      @param source Bitmask of triggers
      @n Available sources:
      @n - INT_DATA_DRDY:    Data ready interrupt
      @n - INT_FIFO_FULL:    FIFO full interrupt
      @n - INT_FIFO_THRES:   FIFO threshold interrupt
      @n - INT_PRESSURE_OOR: Pressure out-of-range interrupt
    '''
    bmp5.set_int_source(bmp5.INT_FIFO_THRES)

    '''!
      # Calibrate the sensor according to the current altitude
      # In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). Please change to the local altitude when using it.
      # If this interface is not called, the measurement data will not eliminate the absolute difference
      # Notice: This interface is only valid for the first call
      # If you do not need to eliminate the absolute difference of measurement, please comment the following line
    '''
    if CALIBRATE_ABSOLUTE_DIFFERENCE:
        bmp5.calibrated_absolute_difference(540.0)
    '''!
        @brief  set the measurement mode of the sensor
        @param  mode: measurement mode
        @n      SLEEP_MODE = 0x00.        #// standby mode
        @n      NORMAL_MODE = 0x01.        #// normal mode
        @n      SINGLE_SHOT_MODE = 0x02.         #// forced mode > only perform once
        @n      CONTINOUS_MODE = 0x03.      #// continuous mode
        @n      DEEP_SLEEP_MODE = 0x04.   #// deep standby mode
    '''
    bmp5.set_measure_mode(bmp5.NORMAL_MODE)

def loop():
    if BMP5_INT_MODE_LATCHED:
        bmp5.get_int_status()
        
    global interrupt_flag
    if interrupt_flag:
        data = bmp5.get_fifo_data()
        print("FIFO len: {} ".format(data.len))
        for i in range(data.len):
            print("Temperature: {0:.2f} C, Pressure: {1:.2f} Pa".format(data.temperature[i], data.pressure[i]))
        interrupt_flag = False

if __name__ == "__main__":
    setup()
    while True:
        loop()