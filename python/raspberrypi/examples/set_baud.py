# -*- coding: utf-8 -*
'''!
  @file  set_baud.py
  @brief  The UART baud rate of the Gravity version can be configured, but this modification can only be performed in UART mode.
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

bmp5 = DFRobot_BMP58X_UART(9600, DEV_ADDR)

def setup():
    while not bmp5.begin():
        print("sensor init error ,please check connect!")
        time.sleep(1)

    bmp5.set_baud(bmp5.BAUD_57600)
    print("set baud to 57600")

if __name__ == "__main__":
    setup()