#pragma once

#include <Arduino.h>

#define BAT_ADC_PIN A4
#define BAT_ADC_VREF 3.6f
#define BAT_ADC_MAX 4095.0f
#define BAT_DIVIDER 2.0f
#define BAT_CAL 0.981f

uint16_t adcScanReadPinMv(uint8_t gpio);
uint16_t adcScanReadVbatMv();
float adcScanBatteryVoltage();
