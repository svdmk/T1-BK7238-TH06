#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WIFI_CONFIG_SSID_LEN 33
#define WIFI_CONFIG_PASS_LEN 65
#define WIFI_CONFIG_ID_LEN 21
#define WIFI_CONFIG_HOST_LEN 64
#define WIFI_CONFIG_USER_LEN 33

struct DeviceConfig {
  char ssid[WIFI_CONFIG_SSID_LEN];
  char pass[WIFI_CONFIG_PASS_LEN];
  char device_id[WIFI_CONFIG_ID_LEN];
  char mqtt_host[WIFI_CONFIG_HOST_LEN];
  char mqtt_user[WIFI_CONFIG_USER_LEN];
  char mqtt_pass[WIFI_CONFIG_PASS_LEN];
  uint8_t ip[4];
  uint8_t sleep_mode;
  uint8_t reserved;
  float temp_change;
  uint16_t mqtt_port;
};

bool wifiConfigLoad(DeviceConfig *cfg);
bool wifiConfigSave(const DeviceConfig *cfg);
