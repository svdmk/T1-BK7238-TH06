#include "gxht4x.h"

#define GXHT4X_ADDRESS 0x44
#define GXHT4X_CMD_MEASURE_HPM 0xFD
#define GXHT4X_CMD_RESET 0x94

static uint8_t scl_pin = 0;
static uint8_t sda_pin = 0;

static inline void sclHigh() {
  pinMode(scl_pin, OUTPUT);
  digitalWrite(scl_pin, HIGH);
}

static inline void sclLow() {
  pinMode(scl_pin, OUTPUT);
  digitalWrite(scl_pin, LOW);
}

static inline void sdaRelease() {
  pinMode(sda_pin, INPUT_PULLUP);
}

static inline void sdaLow() {
  pinMode(sda_pin, OUTPUT);
  digitalWrite(sda_pin, LOW);
}

static inline int sdaRead() {
  return digitalRead(sda_pin);
}

// Busy wait: the GXHT40 clone is disturbed by the chip entering sleep during
// the measurement sequence, so never yield to the RTOS here.
static void busyWaitMs(uint32_t ms) {
  while (ms--) delayMicroseconds(1000);
}

static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void i2cStart() {
  sdaRelease();
  sclHigh();
  delayMicroseconds(5);
  sdaLow();
  delayMicroseconds(6);
  sclLow();
}

static void i2cStop() {
  sclLow();
  sdaLow();
  sclHigh();
  delayMicroseconds(6);
  sdaRelease();
  delayMicroseconds(6);
}

static void i2cAck() {
  sclLow();
  sdaLow();
  delayMicroseconds(2);
  sclHigh();
  delayMicroseconds(5);
  sclLow();
}

static void i2cNack() {
  sclLow();
  sdaRelease();
  delayMicroseconds(2);
  sclHigh();
  delayMicroseconds(5);
  sclLow();
}

static bool i2cWaitAck() {
  sdaRelease();
  delayMicroseconds(1);
  sclHigh();
  delayMicroseconds(1);

  uint16_t timeout = 0;
  while (sdaRead()) {
    if (++timeout > 250) {
      i2cStop();
      return false;
    }
    delayMicroseconds(1);
  }

  sclLow();
  delayMicroseconds(1);
  return true;
}

static void i2cWriteByte(uint8_t value) {
  sclLow();
  for (uint8_t i = 0; i < 8; i++) {
    if (value & 0x80) {
      sdaRelease();
    } else {
      sdaLow();
    }
    value <<= 1;
    delayMicroseconds(1);
    sclHigh();
    delayMicroseconds(2);
    sclLow();
    delayMicroseconds(2);
  }
}

static uint8_t i2cReadByte(bool ack) {
  uint8_t value = 0;
  sdaRelease();

  for (uint8_t i = 0; i < 8; i++) {
    sclLow();
    delayMicroseconds(2);
    sclHigh();
    value <<= 1;
    if (sdaRead()) value++;
    delayMicroseconds(1);
  }

  if (ack) {
    i2cAck();
  } else {
    i2cNack();
  }

  return value;
}

void gxht4xInit(uint8_t sclPin, uint8_t sdaPin) {
  scl_pin = sclPin;
  sda_pin = sdaPin;

  sdaRelease();
  sclHigh();
  busyWaitMs(10);

  i2cStart();
  i2cWriteByte(GXHT4X_ADDRESS << 1 | 0);
  i2cWaitAck();
  i2cWriteByte(GXHT4X_CMD_RESET);
  i2cWaitAck();
  i2cStop();
  busyWaitMs(2);
}

bool gxht4xRead(float *temperature, float *humidity) {
  i2cStart();
  i2cWriteByte(GXHT4X_ADDRESS << 1 | 0);
  if (!i2cWaitAck()) return false;
  i2cWriteByte(GXHT4X_CMD_MEASURE_HPM);
  if (!i2cWaitAck()) return false;
  i2cStop();

  busyWaitMs(10);

  i2cStart();
  i2cWriteByte(GXHT4X_ADDRESS << 1 | 1);
  if (!i2cWaitAck()) {
    i2cStop();
    return false;
  }

  uint8_t buff[6];
  for (uint8_t i = 0; i < 6; i++) {
    buff[i] = i2cReadByte(i != 5);
  }
  i2cStop();

  if (crc8(buff, 2) != buff[2] || crc8(buff + 3, 2) != buff[5]) {
    return false;
  }

  uint16_t raw_temp = ((uint16_t)buff[0] << 8) | buff[1];
  uint16_t raw_hum = ((uint16_t)buff[3] << 8) | buff[4];

  float t = 175.0f * (float)raw_temp / 65535.0f - 45.0f;
  float h = 125.0f * (float)raw_hum / 65535.0f - 6.0f;

  if (h >= 100.0f) h = 100.0f;
  if (t < -45.0f || t > 125.0f || h < 0.0f || h > 100.0f) {
    return false;
  }

  *temperature = t;
  *humidity = h;
  return true;
}
