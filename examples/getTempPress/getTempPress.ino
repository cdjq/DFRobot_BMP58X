/**
 * @file  getTempPress.ino
 * @brief  Get the temperature and pressure data of the BMP58X
 * @details  The temperature and pressure data of the BMP58X is obtained by calling the readPressPa and readTempC functions respectively.
 * @n  The altitude data is obtained by calling the get_altitude() function.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version     V1.0.0
 * @date        2025-09-17
 * @url         https://github.com/DFRobot/DFRobot_BMP58X
 */
#include "DFRobot_BMP58X.h"

/* >> 1.Please choose your communication method below: */
// #define BMP5_COMM_UART
#define BMP5_COMM_I2C
// #define BMP5_COMM_SPI

/* >> 2.If there is no need to eliminate the absolute measurement error, please annotate the following line */
#define CALIBRATE_ABSOLUTE_DIFFERENCE

/**
 * The sensor can communicate via two specific addresses (0x47 and 0x46).
 * The way to switch between addresses depends on your board type:
 * "Dip switch" (for Gravity version): A small switch on the board that you can toggle by hand.
 * "Solder pads" (for Fermion version): Small metal points on the board that need to be connected (or disconnected) using solder. 
 */
const uint8_t ADDR = 0x47;

#if defined(BMP5_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  26/D3|     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  25/D2|     X      |  rx1  |
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
#elif defined(BMP5_COMM_I2C)
  DFRobot_BMP58X_I2C bmp58x(&Wire, ADDR);
#elif defined(BMP5_COMM_SPI)
  #if defined(ESP32)
    DFRobot_BMP58X_SPI bmp58x(&SPI, 26 /*D3*/);
  #elif defined(ARDUINO_BBC_MICROBIT_V2)
    DFRobot_BMP58X_SPI bmp58x(&SPI, 16);
  #else 
    DFRobot_BMP58X_SPI bmp58x(&SPI, 5);
  #endif
#else
  #error
#endif

void setup() {
  Serial.begin(115200);
  while(!bmp58x.begin()){
    Serial.println("Sensor init fail!");
    delay(1000);
  }
  Serial.println("Sensor init success!");
  #if defined(CALIBRATE_ABSOLUTE_DIFFERENCE)
  /**
   * Calibrate the sensor according to the current altitude
   * In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). 
   * Please change to the local altitude when using it.
   * If this interface is not called, the measurement data will not eliminate the absolute difference.
   */
    bmp58x.calibratedAbsoluteDifference(540.0);
  #endif
  /**
   * @param mode Operation mode (see: eMeasureMode_t)
   * @n Available modes:
   * @n - eSleep:         Sleep mode
   * @n - eNormal:        Normal measurement mode
   * @n - eSingleShot:    Single-shot measurement
   * @n - eContinuous:    Continuous measurement
   * @n - eDeepSleep:     Deep Sleep mode
   */
  bmp58x.setMeasureMode(bmp58x.eNormal);
}

void loop() {
  delay(1000);
  Serial.print("temp: ");
  Serial.print(bmp58x.readTempC());
  Serial.print(" (C)  press: ");
  Serial.print(bmp58x.readPressPa());
  Serial.print(" (Pa)  alt: ");
  Serial.print(bmp58x.readAltitudeM());
  Serial.println(" (M)");
}
