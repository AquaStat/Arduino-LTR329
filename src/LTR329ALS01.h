#ifndef LTR329ALS01_H
#define LTR329ALS01_H

#include <Wire.h>
#include <stdbool.h>
#include <stdint.h>

// Device Address
#define LTR329ALS01_I2C_ADDRESS 0x29 // 7-bit Address
// Register Address
#define LTR329ALS01_ADDR_ALS_CONTROL 0x80
#define LTR329ALS01_ADDR_ALS_MEAS_RATE 0x85
#define LTR329ALS01_ADDR_PART_ID 0x86
#define LTR329ALS01_ADDR_MANUFAC_ID 0x87
#define LTR329ALS01_ADDR_ALS_DATA_CH_1_0 0x88
#define LTR329ALS01_ADDR_ALS_DATA_CH_1_1 0x89
#define LTR329ALS01_ADDR_ALS_DATA_CH_0_0 0x8A
#define LTR329ALS01_ADDR_ALS_DATA_CH_0_1 0x8B
#define LTR329ALS01_ADDR_ALS_STATUS 0x8C
// PFactoer for Calculating lux
#define LTR329ALS01_PFACTOR 1
// ALS Gain Bit
typedef enum {
  LTR329ALS01_ALS_GAIN_x1 = 0b000,
  LTR329ALS01_ALS_GAIN_x2 = 0b001,
  LTR329ALS01_ALS_GAIN_x4 = 0b010,
  LTR329ALS01_ALS_GAIN_x8 = 0b011,
  LTR329ALS01_ALS_GAIN_x48 = 0b110,
  LTR329ALS01_ALS_GAIN_x96 = 0b111,
  LTR329ALS01_ALS_GAIN_COUNT = 8,
} ALS_GAIN_Enum;
// ALS Integration Time
typedef enum {
  LTR329ALS01_ALS_INT_100ms = 0b000,
  LTR329ALS01_ALS_INT_50ms = 0b001,
  LTR329ALS01_ALS_INT_200ms = 0b010,
  LTR329ALS01_ALS_INT_400ms = 0b011,
  LTR329ALS01_ALS_INT_150ms = 0b100,
  LTR329ALS01_ALS_INT_250ms = 0b101,
  LTR329ALS01_ALS_INT_300ms = 0b110,
  LTR329ALS01_ALS_INT_350ms = 0b111,
  LTR329ALS01_ALS_INT_COUNT = 8,
} ALS_INT_Enum;
// ALS Measuement Rate
typedef enum {
  LTR329ALS01_ALS_RATE_50ms = 0b000,
  LTR329ALS01_ALS_RATE_100ms = 0b001,
  LTR329ALS01_ALS_RATE_200ms = 0b010,
  LTR329ALS01_ALS_RATE_500ms = 0b011,
  LTR329ALS01_ALS_RATE_1000ms = 0b100,
  LTR329ALS01_ALS_RATE_2000ms = 0b101,
} ALS_MEAS_Enum;

// Co-efficient for Calc lux
const uint8_t ALS_GAIN[LTR329ALS01_ALS_GAIN_COUNT] = {
    1,  // LTR329ALS01_ALS_GAIN_x1
    2,  // LTR329ALS01_ALS_GAIN_x2
    4,  // LTR329ALS01_ALS_GAIN_x4
    8,  // LTR329ALS01_ALS_GAIN_x8
    0,  // Reserved
    0,  // Reserved
    48, // LTR329ALS01_ALS_GAIN_x48
    96, // LTR329ALS01_ALS_GAIN_x96
};
const float ALS_INT[LTR329ALS01_ALS_INT_COUNT] = {
    1,   // LTR329ALS01_ALS_INT_100ms
    0.5, // LTR329ALS01_ALS_INT_50ms
    2,   // LTR329ALS01_ALS_INT_200ms
    4,   // LTR329ALS01_ALS_INT_400ms
    1.5, // LTR329ALS01_ALS_INT_150ms
    2.5, // LTR329ALS01_ALS_INT_250ms
    3,   // LTR329ALS01_ALS_INT_300ms
    3.5, // LTR329ALS01_ALS_INT_350ms
};

typedef union {
  uint8_t raw;
  struct {
    bool activeMode : 1;
    bool resetState : 1;
    uint8_t gain : 3;
    uint8_t reserved : 3;
  };
} ALS_CONTR_REG;
typedef union {
  uint8_t raw;
  struct {
    uint8_t measRate : 3;
    uint8_t intTime : 3;
    uint8_t reserved : 2;
  };
} ALS_MEAS_RATE_REG;
typedef union {
  uint8_t raw;
  struct {
    uint8_t revId : 4;
    uint8_t partNum : 4;
  };
} PART_ID_REG;
typedef union {
  uint8_t raw;
  struct {
    uint8_t manId : 8;
  };
} MANUFAC_ID_REG;
typedef union {
  uint8_t raw;
  struct {
    uint8_t reseved0 : 2;
    bool isNewData : 1;
    uint8_t reseved1 : 1;
    uint8_t gainRange : 3;
    bool isInValid : 1;
  };
} ALS_PS_STATUS_REG;

class LTR329ALS01 {
public:
  LTR329ALS01(void);
#if defined(ESP32) || defined(ESP8266)
  LTR329ALS01(uint8_t sda, uint8_t scl);
#endif
  virtual ~LTR329ALS01();

  void begin(unsigned long timeout = 10 * 1000);

  void reset();

  float readLux();

  void writetControl(bool _isActiveMode, ALS_GAIN_Enum _gain);
  ALS_CONTR_REG readControl();

  void writeMeasRate(ALS_INT_Enum _intTime, ALS_MEAS_Enum _measRate);
  ALS_MEAS_RATE_REG readMeasRate();

  uint8_t readPartNumber();
  uint8_t readRevisionId();
  uint8_t readManufacturerId();

  ALS_PS_STATUS_REG readStatus();

private:
  uint8_t sda = -1;
  uint8_t scl = -1;
  uint8_t readByte(uint8_t addr);
  uint16_t readAlsData(uint8_t ch);

  void writeByte(uint8_t addr, uint8_t data);

  bool isActiveMode = false;
  ALS_GAIN_Enum gain = LTR329ALS01_ALS_GAIN_x1;
  ALS_INT_Enum intTime = LTR329ALS01_ALS_INT_100ms;
  ALS_MEAS_Enum measRate = LTR329ALS01_ALS_RATE_500ms;

  unsigned long timeout;
};

#endif
