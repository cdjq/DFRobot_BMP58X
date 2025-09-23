# -*- coding: utf-8 -*
'''!
  @file  get_data_single_mode.py
  @details  This program communicates with the BMP58x pressure sensor and explains how to use a single acquisition mode
  @n        to collect pressure, temperature, and altitude data through external interrupts.
  @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
  @version     V1.0.0
  @date        2025-09-17
  @url         https://github.com/DFRobot/DFRobot_BMP58X
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
    CS         = 16
    bmp5 = DFRobot_BMP58X_SPI(cs=CS, bus=0, dev=0, speed=8000000)
elif mode == "UART":
    bmp5 = DFRobot_BMP58X_UART(9600, DEV_ADDR)

global interrupt_flag
interrupt_flag = False

def drdy_callback(channel):
    global interrupt_flag
    interrupt_flag = True

gpio_interrupt = 27
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_interrupt, GPIO.IN)
GPIO.add_event_detect(gpio_interrupt, GPIO.RISING, callback=drdy_callback)

def setup():
    while not bmp5.begin():
        print("sensor init error,please check connect!")
        time.sleep(1)
    
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
    bmp5.set_int_source(bmp5.INT_DATA_DRDY)

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
    bmp5.set_measure_mode(bmp5.SINGLE_SHOT_MODE)

def loop():
    global interrupt_flag
    if interrupt_flag:
        interrupt_flag = False
        if bmp5.get_int_status() & bmp5.INT_DATA_DRDY:
            print("temperature : %.2f (C)" % (bmp5.read_temperature()))
            print("Pressure : %.2f (Pa)" % (bmp5.read_pressure()))
            print("Altitude : %.2f (M)" % (bmp5.read_altitude()))
            print("")
            time.sleep(0.5)
            bmp5.set_measure_mode(bmp5.SINGLE_SHOT_MODE)

if __name__ == "__main__":
    setup()
    while True:
        loop()