#include <HardwareSerial.h>
#include <LTR329ALS01.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef DEBUG_LTR329
#define DEBUG_LTR329(x)
#endif

LTR329ALS01::LTR329ALS01() {}
#if defined(ESP32) || defined(ESP8266)
LTR329ALS01::LTR329ALS01(uint8_t sda, uint8_t scl) : sda(sda), scl(scl) {}
#endif
LTR329ALS01::~LTR329ALS01() {}

void LTR329ALS01::begin(unsigned long timeout) {
  // set default timeout
  this->timeout = timeout;

  // Initial wait after Power Up
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // Initialize I2C
  if (sda != -1 && scl != -1) {
    Wire.begin(sda, scl);
  } else {
    Wire.begin();
  }
  // Reset LTR329
  reset();
  // Initialize Parameters
  isActiveMode = true;
  gain = LTR329ALS01_ALS_GAIN_x1;
  // Set Control REG
  writetControl(isActiveMode, gain);
  // Wait for Activate LTR329
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void LTR329ALS01::reset() {
  DEBUG_LTR329("[LTR329] Resetting");
  ALS_CONTR_REG ctrl = {};
  ctrl.resetState = true;
  writeByte(LTR329ALS01_ADDR_ALS_CONTROL, ctrl.raw);

  unsigned long readTimeout = millis() + this->timeout;

  // Wait for resetting
  do {
    DEBUG_LTR329(".");
    ctrl.raw = readByte(LTR329ALS01_ADDR_ALS_CONTROL);

    if (millis() > readTimeout) {
      DEBUG_LTR329("[LTR329] Reset timeout");
      return;
    }

  } while (ctrl.resetState);

  DEBUG_LTR329("[LTR329] Reset finished");
}

float LTR329ALS01::readLux() {
  ALS_PS_STATUS_REG status;

  unsigned long readTimeout = millis() + this->timeout;

  do {
    status = readStatus();
    DEBUG_LTR329("LTR329 STATUS REG: 0b");
    DEBUG_LTR329(status.raw);
    DEBUG_LTR329("\n");

    if (millis() > readTimeout) {
      DEBUG_LTR329("[LTR329] Read status timeout");
      return 0;
    }
  } while (!status.isNewData || status.isInValid);

  uint16_t data_ch0;
  uint16_t data_ch1;

  data_ch1 = readAlsData(1);
  data_ch0 = readAlsData(0);
  DEBUG_LTR329("Read Data CH1: ");
  DEBUG_LTR329(data_ch1);
  DEBUG_LTR329("\n");
  DEBUG_LTR329("Read Data CH0: ");
  DEBUG_LTR329(data_ch0);
  DEBUG_LTR329("\n");

  if (data_ch0 + data_ch1 == 0)
    return 0; // Avoid to Div by Zero
  float ratio = data_ch1 / (data_ch0 + data_ch1);
  float lux;
  float pFactor = LTR329ALS01_PFACTOR;

  if (ratio < 0.45) {
    lux = (1.7743 * data_ch0 + 1.1059 * data_ch1) / ALS_GAIN[gain] /
          ALS_INT[intTime] / pFactor;
  } else if (ratio < 0.64 && ratio >= 0.45) {
    lux = (4.2785 * data_ch0 - 1.9548 * data_ch1) / ALS_GAIN[gain] /
          ALS_INT[intTime] / pFactor;
  } else if (ratio < 0.85 && ratio >= 0.64) {
    lux = (0.5926 * data_ch0 + 0.1185 * data_ch1) / ALS_GAIN[gain] /
          ALS_INT[intTime] / pFactor;
  } else {
    lux = 0;
  }

  return lux;
}

void LTR329ALS01::writetControl(bool _isActiveMode, ALS_GAIN_Enum _gain) {
  isActiveMode = _isActiveMode;
  gain = _gain;

  ALS_CONTR_REG ctrl = {};
  ctrl.activeMode = isActiveMode;
  ctrl.gain = gain;

  writeByte(LTR329ALS01_ADDR_ALS_CONTROL, ctrl.raw);
}

ALS_CONTR_REG LTR329ALS01::readControl() {
  ALS_CONTR_REG ctrl;
  ctrl.raw = readByte(LTR329ALS01_ADDR_ALS_CONTROL);
  return ctrl;
}

void LTR329ALS01::writeMeasRate(ALS_INT_Enum _intTime,
                                ALS_MEAS_Enum _measRate) {
  intTime = _intTime;
  measRate = _measRate;

  ALS_MEAS_RATE_REG mr = {};
  mr.intTime = intTime;
  mr.measRate = measRate;

  writeByte(LTR329ALS01_ADDR_ALS_MEAS_RATE, mr.raw);
}

ALS_MEAS_RATE_REG LTR329ALS01::readMeasRate() {
  ALS_MEAS_RATE_REG mr;
  mr.raw = readByte(LTR329ALS01_ADDR_ALS_MEAS_RATE);
  return mr;
}

ALS_PS_STATUS_REG LTR329ALS01::readStatus() {
  ALS_PS_STATUS_REG status;
  status.raw = readByte(LTR329ALS01_ADDR_ALS_STATUS);
  return status;
}

uint8_t LTR329ALS01::readManufacturerId() {
  uint8_t data = readByte(LTR329ALS01_ADDR_MANUFAC_ID);
  return data;
}

uint8_t LTR329ALS01::readPartNumber() {
  uint8_t data = readByte(LTR329ALS01_ADDR_PART_ID);
  return data >> 4;
}

uint8_t LTR329ALS01::readRevisionId() {
  uint8_t data = readByte(LTR329ALS01_ADDR_PART_ID);
  return data & 0x0F;
}

uint8_t LTR329ALS01::readByte(uint8_t addr) {
  uint8_t rdData;
  uint8_t rdDataCount;
  unsigned long wireReqTimeout = millis() + this->timeout;

  do {
    Wire.beginTransmission(LTR329ALS01_I2C_ADDRESS);
    Wire.write(addr);
    Wire.endTransmission(false); // Restart
    vTaskDelay(10 / portTICK_PERIOD_MS);
    rdDataCount = Wire.requestFrom(LTR329ALS01_I2C_ADDRESS, 1);

    if (millis() > wireReqTimeout) {
      DEBUG_LTR329("Wire request timeout");
      return 0;
    }

  } while (rdDataCount == 0);

  unsigned long wireReadTimeout = millis() + this->timeout;

  while (Wire.available()) {
    rdData = Wire.read();

    if (millis() > wireReadTimeout) {
      DEBUG_LTR329("Wire read timeout");
      return 0;
    }
  }
  return rdData;
}

uint16_t LTR329ALS01::readAlsData(uint8_t ch) {
  uint8_t data_lsb, data_msb;
  uint8_t addr_lsb, addr_msb;
  if (ch == 0) {
    addr_lsb = LTR329ALS01_ADDR_ALS_DATA_CH_0_0;
    addr_msb = LTR329ALS01_ADDR_ALS_DATA_CH_0_1;
  } else {
    addr_lsb = LTR329ALS01_ADDR_ALS_DATA_CH_1_0;
    addr_msb = LTR329ALS01_ADDR_ALS_DATA_CH_1_1;
  }
  data_lsb = readByte(addr_lsb);
  data_msb = readByte(addr_msb);

  char tempString[64];
  sprintf(tempString, "Read CH %d LSB: %d", ch, data_lsb);
  DEBUG_LTR329(tempString);
  sprintf(tempString, "Read CH %d MSB: %d", ch, data_msb);
  DEBUG_LTR329(tempString);

  return ((uint16_t)data_msb << 8) | (uint16_t)data_lsb;
}

void LTR329ALS01::writeByte(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(LTR329ALS01_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}