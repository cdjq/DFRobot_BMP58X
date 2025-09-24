/**
 * @file  DFRobot_BMP58X.cpp
 * @brief  Define the infrastructure of DFRobot_BMP58X class
 * @n      This is a pressure and temperature sensor that can be controlled through I2C/SPI/UART ports.
 * @n      BMP (581/585) has functions such as temperature compensation, data oversampling, IIR filtering, etc
 * @n      These features improve the accuracy of data collected by BMP(581/585) sensors.
 * @n      BMP (581/585) also has a FIFO data buffer, greatly improving its availability.
 * @n      Similarly, BMP (581/585) has an interrupt pin that can be used in an energy-efficient manner without using software algorithms.
 * @copyright   Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      yuanlong.yu(yuanlong.yu@dfrobot.com)
 * @version     V1.0.0
 * @date        2025-07-23
 * @url         https://github.com/DFRobot/DFRobot_BMP58X
 */

#include "DFRobot_BMP58X.h"

DFRobot_BMP58X::DFRobot_BMP58X() {}

DFRobot_BMP58X::~DFRobot_BMP58X() {}

bool DFRobot_BMP58X::begin(void) {
  uint16_t data = 0;
  reset();
  if (readInputReg(REG_I_CHIP_ID, &data, 1) != RET_CODE_OK || (data != BMP581_CHIP_ID && data != BMP585_CHIP_ID)) {
    return false;
  }
  enablePressure(eEnable);
  return true;
}

uint8_t DFRobot_BMP58X::setODR(eOdr_t odr) {
  uint16_t data;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, ODR, odr);
  writeHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP58X::setOSR(eOverSampling_t osrTemp, eOverSampling_t osrPress) {
  uint16_t data;
  readHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, OSR_T, osrTemp);
  BMP5_REG_SET_BITS(data, OSR_P, osrPress);
  writeHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP58X::enablePressure(eEnable_t enable) {
  uint16_t data = 0;
  readHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, PRESS_EN, enable);
  writeHoldingReg(REG_H_OSR_CONFIG, &data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP58X::setMeasureMode(eMeasureMode_t mode) {
  eMeasureMode_t currMode;
  uint16_t data = 0;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  currMode = BMP5_REG_GET_BITS(data, PWR_MODE, eMeasureMode_t);
  if (currMode != eSleep) {
    setPowerMode(eSleep);
    delayMicroseconds(2500);
  }
  switch (mode) {
  case eDeepSleep:
    setDeepStandbyMode(eDeepEnable);
    break;
  case eSleep:
    break;
  case eContinuous:
  case eSingleShot:
  case eNormal:
    setPowerMode(mode);
    break;
  default:
    return RET_CODE_ERROR;
  }
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP58X::reset(void) {
  uint16_t data = SOFT_RESET_CMD;
  writeHoldingReg(REG_H_CMD, &data, 1);
  delayMicroseconds(2000);

  readInputReg(REG_I_CHIP_ID, &data, 1); //< Resetting in SPI communication mode is necessary to restore communication.

  uint16_t intStatus = getIntStatus();
  if (intStatus & eIntStatusResetComplete) {
    enablePressure(eEnable);
    return RET_CODE_OK;
  }
  return RET_CODE_ERROR;
}

float DFRobot_BMP58X::readTempC(void) {
  uint16_t data[3] = {0};
  readInputReg(REG_I_TEMP_DATA_XLSB, data, 3);
  int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) | ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
  // (MSB LSB XLSB) / 2^16;
  return (float)tmpData / 65536.0;
}

float DFRobot_BMP58X::readPressPa(void) {
  uint16_t data[3] = {0};
  readInputReg(REG_I_PRESS_DATA_XLSB, data, 3);
  int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) | ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
  // (MSB LSB XLSB) / 2^6;
  float retData = (float)pressData / 64.0;
  return retData;
}

float DFRobot_BMP58X::readAltitudeM(void) {
  uint16_t data[6] = {0};
  readInputReg(REG_I_TEMP_DATA_XLSB, data, 6);
  int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) | ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);

  int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[5])) << 16) | ((uint32_t)(data[4]) << 8) | ((uint32_t)data[3]);
  float pressure = (float)pressData / 64.0;
  if (_calibrated) {
    float seaLevelPressPa = (pressure / pow(1.0 - (_sealevelAltitude / 44307.7), 5.255302));
    pressure = pressure - seaLevelPressPa + STANDARD_SEA_LEVEL_PRESSURE_PA;
  }
  return calculateAltitude((float)tmpData / 65536, pressure);
}

uint8_t DFRobot_BMP58X::configIIR(eIIRFilter_t iirTemp, eIIRFilter_t iirPress) {
  eMeasureMode_t currMode = getPowerMode();
  setMeasureMode(eSleep);
  uint16_t data[2] = {0};
  readHoldingReg(REG_H_DSP_CONFIG, data, 2);
  BMP5_REG_SET_BITS(data[0], SHDW_SEL_IIR_T, eEnable);
  BMP5_REG_SET_BITS(data[0], SHDW_SEL_IIR_P, eEnable);
  BMP5_REG_SET_BITS(data[0], IIR_FLUSH_FORCED_EN, eEnable);

  BMP5_REG_SET_BITS(data[1], SET_IIR_T, iirTemp);
  BMP5_REG_SET_BITS(data[1], SET_IIR_P, iirPress);

  writeHoldingReg(REG_H_DSP_CONFIG, data, 2);
  if (currMode != eSleep && currMode != eDeepSleep) {
    return setMeasureMode(currMode);
  }
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP58X::configFIFO(eFIFODataSel_t dataSel, eFIFODownSampling_t downSampling, eFIFOWorkMode_t mode, uint8_t threshold) {
  eMeasureMode_t currMode = getPowerMode();
  setMeasureMode(eSleep);
  uint16_t data = 0;
  uint16_t ret = RET_CODE_OK;
  readHoldingReg(REG_H_DSP_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, FIFO_SEL_IIR_T, eEnable);
  BMP5_REG_SET_BITS(data, FIFO_SEL_IIR_P, eEnable);
  writeHoldingReg(REG_H_DSP_CONFIG, &data, 1);

  readHoldingReg(REG_H_FIFO_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, FIFO_MODE, mode);
  ret = setFIFOThreshold(&data, dataSel, threshold);
  if (ret == RET_CODE_OK) {
    writeHoldingReg(REG_H_FIFO_CONFIG, &data, 1);

    readHoldingReg(REG_H_FIFO_SEL, &data, 1);
    BMP5_REG_SET_BITS(data, FIFO_FRAME_SEL, dataSel);
    BMP5_REG_SET_BITS(data, FIFO_DEC_SEL, downSampling);
    writeHoldingReg(REG_H_FIFO_SEL, &data, 1);
  }
  if (currMode != eSleep && currMode != eDeepSleep) {
    return setMeasureMode(currMode);
  }
  return ret;
}

uint8_t DFRobot_BMP58X::getFIFOCount(void) {
  uint16_t data = 0;
  readInputReg(REG_I_FIFO_COUNT, &data, 1);
  uint8_t count = BMP5_REG_GET_BITS(data, FIFO_COUNT, uint8_t);
  return count;
}

DFRobot_BMP58X::sFIFOData_t DFRobot_BMP58X::getFIFOData(void) {
  sFIFOData_t fifoData = {0, {0}, {0}};
  uint16_t regData = 0;
  readHoldingReg(REG_H_FIFO_SEL, &regData, 1);
  uint8_t fifo_frame_sel = BMP5_REG_GET_BITS(regData, FIFO_FRAME_SEL, uint8_t);
  uint8_t count = getFIFOCount();
  if (fifo_frame_sel == eFIFOTempData) {
    for (uint8_t i = 0; i < count; i++) {
      uint16_t data[3] = {0};
      readInputReg(REG_I_FIFO_DATA, data, 3);
      int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) | ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
      fifoData.fifoTempC[i] = (float)tmpData / 65536;
    }
  } else if (fifo_frame_sel == eFIFOPressData) {
    for (uint8_t i = 0; i < count; i++) {
      uint16_t data[3] = {0};
      readInputReg(REG_I_FIFO_DATA, data, 3);
      int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) | ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
      float pressure = (float)pressData / 64.0;
      fifoData.fifoPressPa[i] = pressure;
    }
  } else if (fifo_frame_sel == eFIFOPressAndTempData) {
    for (uint8_t i = 0; i < count; i++) {
      uint16_t data[6] = {0};
      readInputReg(REG_I_FIFO_DATA, data, 6);
      int32_t tmpData = ((int32_t)((int8_t)((uint8_t)data[2])) << 16) | ((uint32_t)(data[1]) << 8) | ((uint32_t)data[0]);
      fifoData.fifoTempC[i] = (float)tmpData / 65536;

      int32_t pressData = ((int32_t)((int8_t)((uint8_t)data[5])) << 16) | ((uint32_t)(data[4]) << 8) | ((uint32_t)data[3]);
      float pressure = (float)pressData / 64.0;
      fifoData.fifoPressPa[i] = pressure;
    }
  }
  fifoData.len = count;
  return fifoData;
}

uint8_t DFRobot_BMP58X::configInterrupt(eIntMode_t mode, eIntPolarity_t pol, eIntOutputMode_t outputMode) {
  uint16_t data = 0, source_data = 0;
  readHoldingReg(REG_H_INT_SOURCE, &source_data, 1);
  writeHoldingReg(REG_H_INT_SOURCE, &data, 1);
  readHoldingReg(REG_H_INT_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, INT_MODE, mode);
  BMP5_REG_SET_BITS(data, INT_POL, pol);
  BMP5_REG_SET_BITS(data, INT_OD, outputMode);
  BMP5_REG_SET_BITS(data, INT_EN, eIntEnable);
  writeHoldingReg(REG_H_INT_CONFIG, &data, 1);
  writeHoldingReg(REG_H_INT_SOURCE, &source_data, 1);
  return RET_CODE_OK;
}

uint8_t DFRobot_BMP58X::setIntSource(uint8_t source) {
  const uint8_t VALID_INT_MASK = eIntDataReady | eIntFIFOFull | eIntFIFOThres | eIntPressureOOR;
  // source &= VALID_INT_MASK;
  uint16_t data = ((uint8_t)source & VALID_INT_MASK);
  writeHoldingReg(REG_H_INT_SOURCE, &data, 1);
  return RET_CODE_OK;
}

uint16_t DFRobot_BMP58X::getIntStatus(void) {
  uint16_t data = 0;
  readInputReg(REG_I_INT_STATUS, &data, 1);
  // DBG(REG_I_INT_STATUS);
  // DBG(data);
  return data;
}

uint8_t DFRobot_BMP58X::setPressOOR(uint32_t oor, uint8_t range, eOORCountLimit_t cntLimit) {
  eMeasureMode_t currMode = getPowerMode();
  setMeasureMode(eSleep);

  uint16_t data[4] = {0};
  readHoldingReg(REG_H_OOR_THR_P_LSB, data, 4);
  data[0] = oor & 0xFF;
  data[1] = (oor >> 8) & 0xFF;
  BMP5_REG_SET_BITS(data[3], OOR_THR_P_16, ((oor >> 16) & 0x01));

  data[2] = range;

  BMP5_REG_SET_BITS(data[3], CNT_LIM, cntLimit);
  writeHoldingReg(REG_H_OOR_THR_P_LSB, data, 4);
  if (currMode != eSleep && currMode != eDeepSleep) {
    return setMeasureMode(currMode);
  }
  return RET_CODE_OK;
}

bool DFRobot_BMP58X::calibratedAbsoluteDifference(float altitude) {
  bool ret = false;
  if (altitude > 0) {
    ret = true;
    _calibrated = true;
    _sealevelAltitude = altitude;
  }
  return ret;
}

uint8_t DFRobot_BMP58X::setDeepStandbyMode(eDeepEnable_t deepMode) {
  uint16_t data = 0;
  uint8_t ret = RET_CODE_OK;
  if (deepMode == eDeepEnable) {
    readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
    BMP5_REG_SET_BITS(data, DEEP_DIS, eDeepEnable);
    BMP5_REG_SET_BITS(data, ODR, eOdr1Hz);
    writeHoldingReg(REG_H_ODR_CONFIG, &data, 1);

    readHoldingReg(REG_H_FIFO_SEL, &data, 1);
    BMP5_REG_SET_BITS(data, FIFO_FRAME_SEL, eFIFODisable);
    writeHoldingReg(REG_H_FIFO_SEL, &data, 1);

    readHoldingReg(REG_H_DSP_IIR, &data, 1);
    BMP5_REG_SET_BITS(data, SET_IIR_T, eFilterBypass);
    BMP5_REG_SET_BITS(data, SET_IIR_P, eFilterBypass);
    writeHoldingReg(REG_H_DSP_IIR, &data, 1);
    ret = RET_CODE_OK;
  } else {
    eMeasureMode_t currMode;
    currMode = getPowerMode();
    if (currMode == eDeepSleep) {
      return setMeasureMode(eSleep);
    }
  }
  return ret;
}

uint8_t DFRobot_BMP58X::setPowerMode(eMeasureMode_t mode) {
  uint16_t data = 0;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  BMP5_REG_SET_BITS(data, DEEP_DIS, DEEP_DISABLE);
  BMP5_REG_SET_BITS(data, PWR_MODE, mode);
  writeHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  return 0;
}

DFRobot_BMP58X::eMeasureMode_t DFRobot_BMP58X::getPowerMode(void) {
  eMeasureMode_t currMode;
  uint16_t data = 0, deep_dis = 0;
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  currMode = BMP5_REG_GET_BITS(data, PWR_MODE, eMeasureMode_t);
  deep_dis = BMP5_REG_GET_BITS(data, DEEP_DIS, uint16_t);
  if (currMode == eSleep && deep_dis == DEEP_ENABLE) {
    return verifyDeepSleepMode();
  }
  return currMode;
}

DFRobot_BMP58X::eMeasureMode_t DFRobot_BMP58X::verifyDeepSleepMode(void) {
  uint16_t data = 0;
  eMeasureMode_t mode = eSleep;
  readHoldingReg(REG_H_FIFO_SEL, &data, 1);
  uint8_t fifo_frame_sel = BMP5_REG_GET_BITS(data, FIFO_FRAME_SEL, uint8_t);
  readHoldingReg(REG_H_ODR_CONFIG, &data, 1);
  eOdr_t odr = BMP5_REG_GET_BITS(data, ODR, eOdr_t);
  readHoldingReg(REG_H_DSP_IIR, &data, 1);
  eIIRFilter_t iir_t = BMP5_REG_GET_BITS(data, SET_IIR_T, eIIRFilter_t);
  eIIRFilter_t iir_p = BMP5_REG_GET_BITS(data, SET_IIR_P, eIIRFilter_t);
  if (odr > eOdr5Hz && fifo_frame_sel == DISABLE && iir_t == eFilterBypass && iir_p == eFilterBypass) {
    mode = eDeepSleep;
  }
  return mode;
}

uint8_t DFRobot_BMP58X::setFIFOThreshold(uint16_t *data, eFIFODataSel_t frame_sel, uint8_t threshold) {
  uint8_t ret = RET_CODE_ERROR;
  if (frame_sel == eFIFOTempData || frame_sel == eFIFOPressData) {
    if (threshold <= 0x1F) {
      BMP5_REG_SET_BITS(*data, FIFO_THRESHOLD, threshold);
      ret = RET_CODE_OK;
    }
  } else if (frame_sel == eFIFOPressAndTempData) {
    if (threshold <= 0x0F) {
      BMP5_REG_SET_BITS(*data, FIFO_THRESHOLD, threshold);
      ret = RET_CODE_OK;
    }
  }
  return ret;
}

float DFRobot_BMP58X::calculateAltitude(float temperature_c, float pressure_pa) {
  (void)temperature_c;
  return (1.0 - pow(pressure_pa / 101325, 0.190284)) * 44307.7;
}

uint8_t DFRobot_BMP58X_I2C::writeHoldingReg(uint8_t reg, void *data, uint8_t len) { return writeReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR; }

uint8_t DFRobot_BMP58X_I2C::readHoldingReg(uint8_t reg, void *data, uint8_t len) { return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR; }

uint8_t DFRobot_BMP58X_I2C::readInputReg(uint8_t reg, void *data, uint8_t len) { return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR; }

bool DFRobot_BMP58X_I2C::readReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  uint8_t count = 0;
  _pWire->beginTransmission((uint8_t)_i2cAddr);
  _pWire->write(reg);
  _pWire->endTransmission();
  _pWire->requestFrom((uint8_t)_i2cAddr, len);

  while (_pWire->available()) {
    tempData[count++] = _pWire->read();
  }
  return count == len;
}

bool DFRobot_BMP58X_I2C::writeReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  uint8_t regData[REG_DATA_LEN_MAX] = {0};
  for (uint8_t i = 0; i < len; ++i) {
    regData[i] = (uint8_t)(tempData[i] & 0xFF);
  }
  _pWire->beginTransmission((uint8_t)_i2cAddr);
  _pWire->write(reg);
  _pWire->write(regData, len);
  _pWire->endTransmission();
  delay(2);
  return true;
}

DFRobot_BMP58X_I2C::DFRobot_BMP58X_I2C(TwoWire *pWire, uint16_t addr) {
  _pWire = pWire;
  _i2cAddr = addr;
}

DFRobot_BMP58X_I2C::~DFRobot_BMP58X_I2C(void) {}

bool DFRobot_BMP58X_I2C::begin(void) {
  _pWire->begin();
  return DFRobot_BMP58X::begin();
}
uint8_t DFRobot_BMP58X_SPI::writeHoldingReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  uint8_t ret = RET_CODE_OK, i = 0;
  while(ret && i < len){
    ret = writeReg(reg+i, &tempData[i], 1);
    ++i;
  }
  return ret; 
}
uint8_t DFRobot_BMP58X_SPI::readHoldingReg(uint8_t reg, void *data, uint8_t len) { return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR; }
uint8_t DFRobot_BMP58X_SPI::readInputReg(uint8_t reg, void *data, uint8_t len) { return readReg(reg, data, len) ? RET_CODE_OK : RET_CODE_ERROR; }
bool DFRobot_BMP58X_SPI::readReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  _pSpi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _pSpi->transfer(reg | 0x80);
  while (len--) {
    *tempData = _pSpi->transfer(0x00);
    tempData++;
  }
  digitalWrite(_csPin, HIGH);
  _pSpi->endTransaction();
  return true;
}
bool DFRobot_BMP58X_SPI::writeReg(uint8_t reg, void *data, uint8_t len) {
  uint16_t *tempData = static_cast<uint16_t *>(data);
  uint8_t regData[REG_DATA_LEN_MAX] = {0};
  for (uint8_t i = 0; i < len; ++i) {
    regData[i] = (uint8_t)(tempData[i] & 0xFF);
  }
  _pSpi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _pSpi->transfer(reg & 0x7F);
  for (uint8_t i = 0; i < len; ++i) {
    _pSpi->transfer(regData[i]);
  }
  digitalWrite(_csPin, HIGH);
  _pSpi->endTransaction();
  // delay(2);
  return true;
}

DFRobot_BMP58X_SPI::DFRobot_BMP58X_SPI(SPIClass *pSpi, uint8_t csPin) {
  _pSpi = pSpi;
  _csPin = csPin;
}

DFRobot_BMP58X_SPI::~DFRobot_BMP58X_SPI(void) {}

bool DFRobot_BMP58X_SPI::begin(void) {
  pinMode(_csPin, OUTPUT);
  _pSpi->begin();
  digitalWrite(_csPin, HIGH);
  return DFRobot_BMP58X::begin();
}

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
DFRobot_BMP58X_UART::DFRobot_BMP58X_UART(SoftwareSerial *sSerial, uint32_t Baud, uint16_t addr) : DFRobot_RTU(sSerial) {
  _serial = sSerial;
  __baud = Baud;
  __addr = addr;
}
#else
DFRobot_BMP58X_UART::DFRobot_BMP58X_UART(HardwareSerial *hSerial, uint32_t Baud, uint16_t addr, uint8_t rxpin, uint8_t txpin) : DFRobot_RTU(hSerial) {
  _serial = hSerial;
  __baud = Baud;
  __rxpin = rxpin;
  __txpin = txpin;
  __addr = addr;
}
#endif
DFRobot_BMP58X_UART::~DFRobot_BMP58X_UART(void) {}

bool DFRobot_BMP58X_UART::begin(void) {
#ifdef ESP32
  _serial->begin(__baud, SERIAL_8N1, __rxpin, __txpin);
  delay(100);
#elif defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  _serial->begin(__baud);
  delay(100);
#else
  _serial->begin(__baud);
#endif
  uint16_t data = 0;
  readInputReg(REG_I_REV_ID, &data, 1);
  return DFRobot_BMP58X::begin();
}

void DFRobot_BMP58X_UART::setBaud(eBaud baud) {
  uint16_t data = baud;
  writeHoldingRegister(__addr, REG_H_MODBUS_BAUD, &data, 1);
}

uint8_t DFRobot_BMP58X_UART::writeHoldingReg(uint8_t reg, void *data, uint8_t len) {
  return writeHoldingRegister(__addr, reg + OFFSET_REG, static_cast<uint16_t *>(data), len);
}
uint8_t DFRobot_BMP58X_UART::readHoldingReg(uint8_t reg, void *data, uint8_t len) {
  return readHoldingRegister(__addr, reg + OFFSET_REG, static_cast<uint16_t *>(data), len);
}
uint8_t DFRobot_BMP58X_UART::readInputReg(uint8_t reg, void *data, uint8_t len) {
  return readInputRegister(__addr, reg + OFFSET_REG, static_cast<uint16_t *>(data), len);
}