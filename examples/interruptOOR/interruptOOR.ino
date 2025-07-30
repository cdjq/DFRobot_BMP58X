/**
 * @file  interruptOOR.ino
 * @brief  Get the temperature and pressure data of the BMP58X through interrupts
 * @details  Obtain BMP58X data by using data in OOR interrupt. !!! The pressure OOR interrupt is triggered using raw, 
 * @n        uncalibrated measurement data.For now, you cannot set up both calibrated data parameters and
 * @n        OOR interrupts at the same time—these two functions cannot be configured together.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version     V1.0.0
 * @date        2025-07-29
 * @url         https://github.com/DFRobot/DFRobot_BMP58X
 */

#include "DFRobot_BMP58X.h"
#include "DFRobot_RTU.h"

/* >> 1.Please choose your communication method below: */
// #define BMP5_COMM_UART
#define BMP5_COMM_I2C
// #define BMP5_COMM_SPI

/**
 * >> 2.Configure the interrupt mode you need to use: Opening the macro below is a latch interrupt,
 * otherwise it is a pulse interrupt。
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
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
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
DFRobot_BMP58X_SPI bmp58x(&SPI, D3);
#else
#error
#endif

volatile uint8_t flag = 0;
void interrupt() {
  if (flag == 0) {
    flag = 1;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Start..");
  while (!bmp58x.begin()) {
    Serial.println("Sensor init fail!");
    delay(1000);
  }

  /**
   * Configure the BMP5 sensor's Out-of-Range (OOR) pressure detection
   * 
   * Parameters:
   * 1. 94658 - Reference pressure value (in Pa). The sensor uses this as the baseline for OOR detection.
   * 2. 50    - Pressure range tolerance (in Pa). The valid pressure range is calculated as:
   *             (Reference - Range) < Actual Pressure < (Reference + Range)
   *             i.e., 94608Pa < Actual Pressure < 94708Pa
   * 3. bmp58x.eOORCountLimit1 - Threshold for consecutive out-of-range readings before triggering an OOR event
   * 
   * Functionality:
   * When the sensor detects a pressure value outside the range of 94608-94708Pa for 1 consecutive reading,
   * it triggers an OOR event, which can be monitored via interrupts or polling.
   */
  bmp58x.setOORPress(94658, 50, bmp58x.eOORCountLimit1);

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
   * @n - eIntPressureOor: Pressure out-of-range interrupt
   */
  bmp58x.setIntSource(bmp58x.eIntPressureOor);

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

#if defined(ESP32) || defined(ESP8266)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  attachInterrupt(digitalPinToInterrupt(D6) /* Query the interrupt number of the D6 pin */, interrupt, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 5 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  attachInterrupt(digitalPinToInterrupt(5) /* Query the interrupt number of the 5 pin */, interrupt, RISING);
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
  
}
void loop() {
#if defined(BMP5_INT_MODE_LATCHED)
  bmp58x.getIntStatus(); // Latched interrupts must read the interrupt status register before new interrupts can be generated.
#endif
  if (flag == 1) {
    flag = 0;
    Serial.print("temp: ");
    Serial.print(bmp58x.readTempC());
    Serial.print(" (C)  press: ");
    Serial.print(bmp58x.readPressPa());
    Serial.print(" (Pa)  alt: ");
    Serial.print(bmp58x.readAltitudeM());
    Serial.println(" (M)");
  }
}