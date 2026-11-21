#include <Arduino.h>
#include "adc_scan.h"

extern "C" {
#include <sdk_private.h>
}

uint16_t adcScanReadPinMv(uint8_t gpio) {
  return analogReadVoltage(gpio);
}

float adcScanBatteryVoltage() {
  uint16_t code = analogReadVoltage(BAT_ADC_PIN);
  return (float)code * (BAT_ADC_VREF / BAT_ADC_MAX) * BAT_DIVIDER * BAT_CAL;
}

uint16_t adcScanReadVbatMv() {
  UINT32 status = 0;
  saradc_desc_t adc;
  DD_HANDLE handle;
  uint16_t data = 0;

  saradc_config_param_init(&adc);
  adc.channel = 0;
  adc.mode = ADC_CONFIG_MODE_CONTINUE;
  adc.pData = &data;
  adc.data_buff_size = 1;

  handle = ddev_open(SARADC_DEV_NAME, &status, (UINT32)&adc);
  if (handle == (DD_HANDLE)-1 || status != SARADC_SUCCESS) {
    if (handle != (DD_HANDLE)-1) ddev_close(handle);
    return 0;
  }

  uint32_t timeout = 50;
  while (!adc.has_data || adc.current_sample_data_cnt < 1) {
    delay(1);
    if (--timeout == 0) break;
  }

  uint8_t run_stop = 0;
  ddev_control(handle, SARADC_CMD_RUN_OR_STOP_ADC, &run_stop);
  ddev_close(handle);

  return data;
}
