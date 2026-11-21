#pragma once

#include <Arduino.h>

uint16_t retentionRead();
void retentionWrite(uint16_t value);
void deepSleepRetain(uint16_t value, uint32_t sleepMs);
