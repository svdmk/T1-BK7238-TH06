#include "wifi_config.h"

#include <libretiny.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Конфигурацията се пази в началото на userdata партишъна (не в app-а),
// за да оцелява след deep sleep, soft reset и OTA ъпдейт на firmware-а.
#define WIFI_CONFIG_ADDR 0x1EC000UL
#define WIFI_CONFIG_MAGIC 0x32464354UL
#define WIFI_CONFIG_VERSION 3

struct WifiConfigBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  DeviceConfig cfg;
  uint32_t crc;
};

// Стар (v2) формат без MQTT полета - пазим го само за миграция при ъпдейт.
struct DeviceConfigV2 {
  char ssid[WIFI_CONFIG_SSID_LEN];
  char pass[WIFI_CONFIG_PASS_LEN];
  char device_id[WIFI_CONFIG_ID_LEN];
  uint8_t ip[4];
  uint8_t sleep_mode;
  uint8_t reserved;
  float temp_change;
};

struct WifiConfigBlobV2 {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  DeviceConfigV2 cfg;
  uint32_t crc;
};

// CRC предпазва от зареждане на повредена/частично записана конфигурация.
static uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }
  return ~crc;
}

static bool cfgValid(const DeviceConfig *c) {
  if (c->ssid[WIFI_CONFIG_SSID_LEN - 1] != 0 || c->ssid[0] == 0) return false;
  if (c->pass[WIFI_CONFIG_PASS_LEN - 1] != 0) return false;
  if (c->device_id[WIFI_CONFIG_ID_LEN - 1] != 0 || c->device_id[0] == 0) return false;
  if (c->mqtt_host[WIFI_CONFIG_HOST_LEN - 1] != 0 || c->mqtt_host[0] == 0) return false;
  if (c->mqtt_user[WIFI_CONFIG_USER_LEN - 1] != 0) return false;
  if (c->mqtt_pass[WIFI_CONFIG_PASS_LEN - 1] != 0) return false;
  if (c->mqtt_port == 0) return false;
  if (c->sleep_mode < 1 || c->sleep_mode > 4) return false;
  if (!(c->temp_change > 0.0f) || c->temp_change > 20.0f) return false;
  if (c->ip[0] == 0 || c->ip[0] == 255) return false;
  return true;
}

static bool cfgV2Valid(const DeviceConfigV2 *c) {
  if (c->ssid[WIFI_CONFIG_SSID_LEN - 1] != 0 || c->ssid[0] == 0) return false;
  if (c->pass[WIFI_CONFIG_PASS_LEN - 1] != 0) return false;
  if (c->device_id[WIFI_CONFIG_ID_LEN - 1] != 0 || c->device_id[0] == 0) return false;
  if (c->sleep_mode < 1 || c->sleep_mode > 4) return false;
  if (!(c->temp_change > 0.0f) || c->temp_change > 20.0f) return false;
  if (c->ip[0] == 0 || c->ip[0] == 255) return false;
  return true;
}

static void cfgFromV2(DeviceConfig *dst, const DeviceConfigV2 *src) {
  memset(dst, 0, sizeof(*dst));
  memcpy(dst->ssid, src->ssid, sizeof(dst->ssid));
  memcpy(dst->pass, src->pass, sizeof(dst->pass));
  memcpy(dst->device_id, src->device_id, sizeof(dst->device_id));
  memcpy(dst->ip, src->ip, sizeof(dst->ip));
  dst->sleep_mode = src->sleep_mode;
  dst->temp_change = src->temp_change;
}

// Първо опитваме актуалния формат (v3). Ако не стане, опитваме стария v2
// и запазваме старите полета; MQTT полетата остават празни и setup() слага defaults.
bool wifiConfigLoad(DeviceConfig *cfg) {
  WifiConfigBlob blob;
  if (lt_flash_read(WIFI_CONFIG_ADDR, (uint8_t *)&blob, sizeof(blob)) == sizeof(blob)) {
    if (blob.magic == WIFI_CONFIG_MAGIC && blob.version == WIFI_CONFIG_VERSION &&
        blob.crc == crc32((const uint8_t *)&blob, offsetof(WifiConfigBlob, crc)) &&
        cfgValid(&blob.cfg)) {
      memcpy(cfg, &blob.cfg, sizeof(DeviceConfig));
      return true;
    }
  }

  WifiConfigBlobV2 old;
  if (lt_flash_read(WIFI_CONFIG_ADDR, (uint8_t *)&old, sizeof(old)) == sizeof(old)) {
    if (old.magic == WIFI_CONFIG_MAGIC && old.version == 2 &&
        old.crc == crc32((const uint8_t *)&old, offsetof(WifiConfigBlobV2, crc)) &&
        cfgV2Valid(&old.cfg)) {
      cfgFromV2(cfg, &old.cfg);
      return true;
    }
  }

  return false;
}

bool wifiConfigSave(const DeviceConfig *cfg) {
  if (!cfgValid(cfg)) {
    return false;
  }

  WifiConfigBlob blob;
  memset(&blob, 0, sizeof(blob));
  blob.magic = WIFI_CONFIG_MAGIC;
  blob.version = WIFI_CONFIG_VERSION;
  memcpy(&blob.cfg, cfg, sizeof(DeviceConfig));
  blob.crc = crc32((const uint8_t *)&blob, offsetof(WifiConfigBlob, crc));

  // Flash може да се пише само след изтриване на 4KB сектора.
  if (!lt_flash_erase_block(WIFI_CONFIG_ADDR)) {
    return false;
  }
  return lt_flash_write(WIFI_CONFIG_ADDR, (const uint8_t *)&blob, sizeof(blob)) == sizeof(blob);
}
