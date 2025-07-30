/**
 * @file  setBaud.ino
 * @brief  The UART baud rate of the Gravity version can be configured, but this modification can only be performed in UART mode.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version     V1.0.0
 * @date        2025-07-29
 * @url         https://github.com/DFRobot/DFRobot_BMP58X
 */

#include "DFRobot_BMP58X.h"
#include "DFRobot_RTU.h"

/**
 * The sensor can communicate via two specific addresses (0x47 and 0x46).
 * The way to switch between addresses depends on your board type:
 * "Dip switch" (for Gravity version): A small switch on the board that you can toggle by hand.
 * "Solder pads" (for Fermion version): Small metal points on the board that need to be connected (or disconnected) using solder. 
 */
const uint8_t ADDR = 0x47;


/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
#include <SoftwareSerial.h>
#endif
#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)
  SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
  DFRobot_BMP58X_UART bmp58x(&mySerial, 9600, ADDR);
#elif defined(ESP32)
  DFRobot_BMP58X_UART bmp58x(&Serial1, 9600, ADDR, /*rx*/ D2, /*tx*/ D3);
#else
  DFRobot_BMP58X_UART bmp58x(&Serial1,9600, ADDR);
#endif


void setup() {
  Serial.begin(9600);
  while(!bmp58x.begin()){
    Serial.println("Sensor init fail!");
    delay(1000);
  }

  bmp58x.setBaud(bmp58x.e115200);
  Serial.println("Set Baud succeed!");
}

void loop() {
}
