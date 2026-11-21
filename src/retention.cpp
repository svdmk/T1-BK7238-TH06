#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "retention.h"

extern "C" {
#include <sdk_private.h>
#include <sys_ctrl_pub.h>
}

uint16_t retentionRead() {
  return (uint16_t)(REG_READ(SCTRL_SW_RETENTION) & SW_RETENTION_VAL_MASK);
}

void retentionWrite(uint16_t value) {
  REG_WRITE(SCTRL_SW_RETENTION, (UINT32)value);
}

void deepSleepRetain(uint16_t value, uint32_t sleepMs) {
  PS_DEEP_CTRL_PARAM param;
  memset(&param, 0, sizeof(param));

  param.wake_up_way = PS_DEEP_WAKEUP_RTC;
  uint64_t ticks = (uint64_t)32768 * sleepMs / 1000;
  param.sleep_time = (ticks >= 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
  param.lpo_32k_src = LPO_SELECT_ROSC;

  retentionWrite(value);

  GLOBAL_INT_DECLARATION();
  GLOBAL_INT_DISABLE();
  sctrl_enter_rtos_deep_sleep_fix(&param);
  ps_delay(500);
  GLOBAL_INT_RESTORE();
}
