/**
 * @file  interruptDataDrdy.ino
 * @brief  Get the temperature and pressure data of the BMP58X through interrupts
 * @details  Obtain BMP58X data by using data in  ready interrupt.
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
 * >> 3.Configure the interrupt mode you need to use: Opening the macro below is a latch interrupt,
 * otherwise it is a pulse interrupt。Latched interrupts must read the interrupt status register before new interrupts can be generated.
*/
// #define BMP5_INT_MODE_LATCHED

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
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
#endif
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(/*rx =*/4, /*tx =*/5);
DFRobot_BMP58X_UART bmp58x(&mySerial, 9600, ADDR);
#elif defined(ESP32)
DFRobot_BMP58X_UART bmp58x(&Serial1, 9600, ADDR, /*rx*/ D2, /*tx*/ D3);
#else
DFRobot_BMP58X_UART bmp58x(&Serial1, 9600, ADDR);
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

volatile uint8_t flag = 0;
#if defined(ESP8266)
void IRAM_ATTR interrupt()
#else
void interrupt()
#endif
{
  if (flag == 0) {
    flag = 1;
  }
}

void setup() {
  Serial.begin(115200);
  while (!bmp58x.begin()) {
    Serial.println("Sensor init fail!");
    delay(1000);
  }
  Serial.println("Sensor init success!");
  /**
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
   */
#if defined(BMP5_INT_MODE_LATCHED)
  bmp58x.configInterrupt(bmp58x.eIntModeLatched, bmp58x.eIntHighActive, bmp58x.eIntPushPull);
#else
  bmp58x.configInterrupt(bmp58x.eIntModePulsed, bmp58x.eIntHighActive, bmp58x.eIntPushPull);
#endif

  /**
   * @brief Enables specific interrupt sources
   * @param source Bitmask of triggers (see: eIntSource_t)
   * @n Available sources:
   * @n - eIntDataReady:    Data ready interrupt
   * @n - eIntFIFOFull:    FIFO full interrupt
   * @n - eIntFIFOThres:   FIFO threshold interrupt
   * @n - eIntPressureOOR: Pressure out-of-range interrupt
   */
  bmp58x.setIntSource(bmp58x.eIntDataReady);

  #if defined(CALIBRATE_ABSOLUTE_DIFFERENCE)
  /**
   * Calibrate the sensor according to the current altitude
   * In this example, we use an altitude of 540 meters in Wenjiang District of Chengdu (China). 
   * Please change to the local altitude when using it.
   * If this interface is not called, the measurement data will not eliminate the absolute difference.
   */
    bmp58x.calibratedAbsoluteDifference(540.0);
  #endif

#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, interrupt, RISING);
#elif defined(ESP8266)
  #if defined (BMP5_COMM_UART)
  const uint8_t interruptPin = 12;
  #elif defined (BMP5_COMM_I2C)
  const uint8_t interruptPin = 13;
  #elif defined (BMP5_COMM_SPI)
  const uint8_t interruptPin = 4;
  #else
  #error
  #endif
  attachInterrupt(digitalPinToInterrupt(interruptPin) , interrupt, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, interrupt, RISING);
#else
  /* The Correspondence Table of AVR Series Arduino Interrupt Pins And Terminal Numbers
     * ---------------------------------------------------------------------------------------
     * |                                        |  DigitalPin  | 2  | 3  |                   |
     * |    Uno, Nano, Mini, other 328-based    |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  |                   |
     * |-------------------------------------------------------------------------------------|
     * |                                        |    Pin       | 2  | 3  | 21 | 20 | 19 | 18 |
     * |               Mega2560                 |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  | 5  |
     * |-------------------------------------------------------------------------------------|
     * |                                        |    Pin       | 3  | 2  | 0  | 1  | 7  |    |
     * |    Leonardo, other 32u4-based          |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  |    |
     * |--------------------------------------------------------------------------------------
     * ---------------------------------------------------------------------------------------------------------------------------------------------
     *                      The Correspondence Table of micro:bit Interrupt Pins And Terminal Numbers
     * ---------------------------------------------------------------------------------------------------------------------------------------------
     * |             micro:bit                       | DigitalPin |P0-P20 can be used as an external interrupt                                     |
     * |  (When using as an external interrupt,      |---------------------------------------------------------------------------------------------|
     * |no need to set it to input mode with pinMode)|Interrupt No|Interrupt number is a pin digital value, such as P0 interrupt number 0, P1 is 1 |
     * |-------------------------------------------------------------------------------------------------------------------------------------------|
     */
  attachInterrupt(/*Interrupt No*/ 0, interrupt, RISING);  // Open the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                           // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
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
  if (flag == 1) {
    flag = 0;
    if (bmp58x.getIntStatus() & bmp58x.eIntDataReady){
      Serial.print("temp: ");
      Serial.print(bmp58x.readTempC());
      Serial.print(" (C)  press: ");
      Serial.print(bmp58x.readPressPa());
      Serial.print(" (Pa)  alt: ");
      Serial.print(bmp58x.readAltitudeM());
      Serial.println(" (M)");
    }
  }
}