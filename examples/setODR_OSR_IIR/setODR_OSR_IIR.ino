/**
 * @file  setODR_OSR_IIR.ino
 * @brief  Configure ODR (Output Data Rate), OSR (Oversampling Rate) and IIR filter for BMP58X sensor
 * @details  Demonstrate how to set key parameters of the sensor, including:
 *           - Configure ODR (output data rate)
 *           - Configure OSR (temperature and pressure oversampling rates)
 *           - Configure IIR filter coefficients
 *           - Enable data ready interrupt and read sensor data
 * @copyright  Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license    The MIT License (MIT)
 * @author     yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version    V1.0.0
 * @date       2025-06-06
 * @url        https://github.com/DFRobot/DFRobot_BMP58X
*/

#include "DFRobot_BMP58X.h"
#include "DFRobot_RTU.h"

/* >> 1.Please choose your communication method below: */
// #define BMP5_COMM_UART
#define BMP5_COMM_I2C
// #define BMP5_COMM_SPI

/* >> 2.If there is no need to eliminate the absolute measurement error, please annotate the following line */
#define CALIBRATE_ABSOLUTE_DIFFERENCE

/**
 * >> 3.Configure the interrupt mode you need to use: Opening the macro below is a latch interrupt,
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
/**
 * Note: The chip select (CS) pin used here is the digital pin D3 on the ESP32 board. You may also choose another 
 * non-conflicting GPIO pin as the external interrupt pin.
 */
uint8_t csPin = D3;
DFRobot_BMP58X_SPI bmp58x(&SPI, csPin);
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
   * Maximum nominal ODR setting per OSR settings in NORMAL mode
   * max ODR [Hz] represents the maximum output data rate (unit: Hertz).
   * OSR_T is a parameter related to the temperature oversampling rate, 
   * with values of 1, 2, 4, 8, 16, 32, 64, 128.
   * OSR_P is a parameter related to the pressure oversampling rate, 
   * with values of 1, 2, 4, 8, 16, 32, 64, 128.
   * The table content is as follows:
   * | max ODR [Hz] | OSR_T = 1 | OSR_T = 2 | OSR_T = 4 | OSR_T = 8 | OSR_T = 16 | OSR_T = 32 | OSR_T = 64 | OSR_T = 128 |
   * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
   * | OSR_P = 1 | 240.00 | 240.00 | 240.00 | 240.00 | 200.00 | 130.00 | 80.00 | 40.00 |
   * | OSR_P = 2 | 240.00 | 240.00 | 240.00 | 220.00 | 180.00 | 120.00 | 70.00 | 40.00 |
   * | OSR_P = 4 | 220.00 | 220.00 | 200.00 | 180.00 | 140.00 | 100.00 | 70.00 | 40.00 |
   * | OSR_P = 8 | 140.00 | 140.00 | 130.00 | 120.00 | 100.00 | 80.00 | 50.00 | 35.00 |
   * | OSR_P = 16 | 80.00 | 80.00 | 80.00 | 70.00 | 70.00 | 50.00 | 45.00 | 30.00 |
   * | OSR_P = 32 | 45.00 | 45.00 | 40.00 | 40.00 | 40.00 | 35.00 | 30.00 | 20.00 |
   * | OSR_P = 64 | 20.00 | 20.00 | 20.00 | 20.00 | 20.00 | 20.00 | 15.00 | 15.00 |
   * | OSR_P = 128 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 10.00 | 5.00 |
   */
  /**
   * Maximum nominal ODR setting per OSR settings in NORMAL mode 
   * for temperature only measurements
   * max ODR [Hz] represents the maximum output data rate (unit: Hertz).
   * OSR_T is a parameter related to the temperature oversampling rate, 
   * with values of 1, 2, 4, 8, 16, 32, 64, 128.
   * The table content is as follows:
   * | max ODR [Hz] | OSR_T = 1 | OSR_T = 2 | OSR_T = 4 | OSR_T = 8 | OSR_T = 16 | OSR_T = 32 | OSR_T = 64 | OSR_T = 128 |
   * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
   * | - | 240.00 | 240.00 | 240.00 | 240.00 | 200.00 | 130.00 | 80.00 | 40.00 |
   */

  bmp58x.setODR(bmp58x.eOdr5Hz);
  bmp58x.setOSR(bmp58x.eOverSampling16, bmp58x.eOverSampling16);

  /**
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
   */
  bmp58x.configIIR(bmp58x.eFilter15, bmp58x.eFilter15);


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

  // bmp58x.setMeasureMode(bmp58x.eContinuous);


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