#pragma once

#include <Arduino.h>

void gxht4xInit(uint8_t sclPin, uint8_t sdaPin);
bool gxht4xRead(float *temperature, float *humidity);
