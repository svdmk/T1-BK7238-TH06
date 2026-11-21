#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <WiFi.h>
#include <WebServer.h>
#include <libretiny.h>
#include "gxht4x.h"
#include "adc_scan.h"
#include "retention.h"
#include "mqtt_qos1.h"
#include "wifi_config.h"

#define LED_PIN 21
#define SCL_PIN 24
#define SDA_PIN 17

// ===== Стойности по подразбиране (при първо пускане или изтрит флаш) =====
#define DEVICE_ID_DEFAULT "TH06"
#define DEVICE_IP_LAST_DEFAULT 35
#define SLEEP_MODE_DEFAULT 2
#define TEMP_CHANGE_DEFAULT 0.5f

#define WIFI_SSID_DEFAULT "wifi"
#define WIFI_PASS_DEFAULT "password"

#define MQTT_HOST_DEFAULT "192.168.1.20"
#define MQTT_PORT_DEFAULT 1883
#define MQTT_USER_DEFAULT "user"
#define MQTT_PASS_DEFAULT "pass"

// ===== Конфигурационен режим (задържан бутон при wake) =====
#define BUTTON_PIN 22
#define CONFIG_PORTAL_TIMEOUT_MS 600000UL
#define AP_SSID "TH06"
#define AP_PASSWORD ""  // празно = отворена мрежа

// ===== Профили: 1 = 1 мин, 2 = 2 мин, 3 = 5 мин, 4 = 20 s (тест; heartbeat 2 ч) =====
// sleep_ms        - време за deep sleep между събужданията
// temp_scale/offset/code_max - кодиране на температурата в retention (резолюция / код при 0 °C / макс. код)
// counter_bits/mask - колко бита от 16-битовия retention остават за брояча на събужданията
// heartbeat_wakes - след колко wake-а без промяна се праща принудително (2 часа)
struct SleepProfile {
  uint32_t sleep_ms;
  float temp_scale;
  int temp_offset;
  int temp_code_max;
  uint8_t counter_bits;
  uint16_t counter_mask;
  uint16_t heartbeat_wakes;
};

static const SleepProfile SLEEP_PROFILES[5] = {
    {0, 0, 0, 0, 0, 0, 0},                    // 0 - не се ползва
    {60000UL, 4, 256, 511, 7, 0x7F, 120},     // 1: 1 мин, 0.25 °C
    {120000UL, 10, 400, 1023, 6, 0x3F, 60},   // 2: 2 мин, 0.1 °C
    {300000UL, 10, 400, 2047, 5, 0x1F, 24},   // 3: 5 мин, 0.1 °C
    {20000UL, 2, 40, 127, 9, 0x1FF, 360},     // 4: 20 сек (тест), 0.5 °C
};

#define WIFI_TIMEOUT_MS 20000       // колко да чака свързване към рутера
#define MQTT_QOS 1                  // QoS1 за данните (чака PUBACK)
#define MQTT_RETAIN 1               // задържани стойности в брокера
#define MQTT_PUB_TIMEOUT_MS 3000    // таймаут за едно публикуване
#define DISCOVERY_REFRESH_ON_HEARTBEAT 0  //(сложи 0 за изключване и 1 за включване) DISCOVERY_REFRESH само при първоначално пускане, не и при събуждане
#define RETRY_BACKOFF_MS 1800000UL  // 30 min при 3 последователни неуспеха

DeviceConfig cfg;  // runtime конфигурация, заредена от флаша
const SleepProfile *sleepProf = &SLEEP_PROFILES[SLEEP_MODE_DEFAULT];

// DEVICE_ID влиза в client_id, MQTT темите и discovery -> трябва да е уникално за всяко устройство
char mqtt_client_id[40];
char topic_temp[48];
char topic_hum[48];
char topic_bat[48];



float current_temp = 0;
float current_hum = 0;
float current_bat = 0;

WiFiClient wifiClient;

static void ledSet(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

static bool buttonIsHeld() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(20);
  if (digitalRead(BUTTON_PIN) != LOW) return false;
  for (int i = 0; i < 15; i++) {
    delay(10);
    if (digitalRead(BUTTON_PIN) != LOW) return false;
  }
  return true;
}

// ===== OTA през уеб портала =====
// Качва се firmware.uf2 (същият файл, който се флашва). uf2ota записва само
// блоковете за device:download в OTA партишъна (0x12C000) и игнорира
// flasher блоковете. Накрая lt_reboot() кара bootloader-а да приложи образа.
static lt_ota_ctx_t otaCtx;
static bool otaActive = false;  // има започнато качване
static bool otaOk = false;      // всички блокове са записани успешно

static String htmlEscape(const char *s) {
  String out;
  while (s && *s) {
    switch (*s) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += *s;
    }
    s++;
  }
  return out;
}

static void setConfigDefaults(DeviceConfig *c) {
  memset(c, 0, sizeof(*c));
  strncpy(c->ssid, WIFI_SSID_DEFAULT, sizeof(c->ssid) - 1);
  strncpy(c->pass, WIFI_PASS_DEFAULT, sizeof(c->pass) - 1);
  strncpy(c->device_id, DEVICE_ID_DEFAULT, sizeof(c->device_id) - 1);
  strncpy(c->mqtt_host, MQTT_HOST_DEFAULT, sizeof(c->mqtt_host) - 1);
  strncpy(c->mqtt_user, MQTT_USER_DEFAULT, sizeof(c->mqtt_user) - 1);
  strncpy(c->mqtt_pass, MQTT_PASS_DEFAULT, sizeof(c->mqtt_pass) - 1);
  c->mqtt_port = MQTT_PORT_DEFAULT;
  c->ip[0] = 192;
  c->ip[1] = 168;
  c->ip[2] = 110;
  c->ip[3] = DEVICE_IP_LAST_DEFAULT;
  c->sleep_mode = SLEEP_MODE_DEFAULT;
  c->temp_change = TEMP_CHANGE_DEFAULT;
}

static bool parseIp(const char *s, uint8_t out[4]) {
  unsigned a, b, c, d;
  char extra;
  if (sscanf(s, "%u.%u.%u.%u%c", &a, &b, &c, &d, &extra) != 4) return false;
  if (a > 255 || b > 255 || c > 255 || d > 255) return false;
  if (a == 0 || a == 255 || d == 0 || d == 255) return false;
  out[0] = (uint8_t)a;
  out[1] = (uint8_t)b;
  out[2] = (uint8_t)c;
  out[3] = (uint8_t)d;
  return true;
}

static bool validDeviceId(const char *s) {
  size_t n = strlen(s);
  if (n == 0 || n >= WIFI_CONFIG_ID_LEN) return false;
  for (size_t i = 0; i < n; i++) {
    char c = s[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-'))
      return false;
  }
  return true;
}

// Сглобява MQTT имената от DEVICE_ID; вика се след зареждане на конфигурацията
static void buildRuntimeStrings() {
  snprintf(mqtt_client_id, sizeof(mqtt_client_id), "LibreTiny_bk7238_%s", cfg.device_id);
  snprintf(topic_temp, sizeof(topic_temp), "%s/current_temp", cfg.device_id);
  snprintf(topic_hum, sizeof(topic_hum), "%s/current_hum", cfg.device_id);
  snprintf(topic_bat, sizeof(topic_bat), "%s/current_bat", cfg.device_id);
}

// Уеб портал за конфигурация. Влиза се само при задържан бутон при wake.
// AP по подразбиране в LibreTiny е 192.168.43.1; тук изрично го правим 192.168.4.1.
static void startConfigPortal() {
  ledSet(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  const char *apPass = AP_PASSWORD[0] ? AP_PASSWORD : nullptr;
  if (!WiFi.softAP(AP_SSID, apPass)) {
    WiFi.mode(WIFI_OFF);
    ledSet(false);
    return;
  }

  WebServer server(80);

  server.on("/", HTTP_GET, [&server]() {
    char ipbuf[16];
    char tbuf[16];
    char portbuf[8];
    snprintf(ipbuf, sizeof(ipbuf), "%u.%u.%u.%u", cfg.ip[0], cfg.ip[1], cfg.ip[2], cfg.ip[3]);
    int tc = (int)(cfg.temp_change * 100.0f + 0.5f);
    snprintf(tbuf, sizeof(tbuf), "%d.%02d", tc / 100, tc % 100);
    snprintf(portbuf, sizeof(portbuf), "%u", (unsigned)cfg.mqtt_port);

    String page;
    page.reserve(2000);
    page += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
    page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    page += "<title>TH06</title></head><body style='font-family:sans-serif;max-width:460px;margin:auto;padding:16px'>";
    page += "<h2>TH06 настройки</h2><form method='post' action='/save'>";
    page += "<p>WiFi мрежа (SSID)<br><input name='ssid' maxlength='32' required style='width:100%;padding:8px' value='";
    page += htmlEscape(cfg.ssid);
    page += "'></p>";
    page += "<p>WiFi парола<br><input name='pass' type='password' maxlength='64' placeholder='(непроменена)' style='width:100%;padding:8px'></p>";
    page += "<p><label><input type='checkbox' name='open' value='1'> Отворена мрежа (без парола)</label></p>";
    page += "<h3>MQTT брокер</h3>";
    page += "<p>MQTT IP / host<br><input name='mqtt_host' maxlength='63' required style='width:100%;padding:8px' value='";
    page += htmlEscape(cfg.mqtt_host);
    page += "'></p>";
    page += "<p>MQTT порт<br><input name='mqtt_port' type='number' min='1' max='65535' required style='width:100%;padding:8px' value='";
    page += portbuf;
    page += "'></p>";
    page += "<p>MQTT потребител<br><input name='mqtt_user' maxlength='32' style='width:100%;padding:8px' value='";
    page += htmlEscape(cfg.mqtt_user);
    page += "'></p>";
    page += "<p>MQTT парола<br><input name='mqtt_pass' type='password' maxlength='64' placeholder='(непроменена)' style='width:100%;padding:8px'></p>";
    page += "<p>IP адрес<br><input name='ip' maxlength='15' required style='width:100%;padding:8px' value='";
    page += ipbuf;
    page += "'></p>";
    page += "<p>DEVICE_ID<br><input name='device_id' maxlength='20' required style='width:100%;padding:8px' value='";
    page += htmlEscape(cfg.device_id);
    page += "'></p>";
    page += "<p>SLEEP_MODE<br><select name='sleep_mode' style='width:100%;padding:8px'>";
    for (int m = 1; m <= 4; m++) {
      page += "<option value='";
      page += m;
      page += "'";
      if (m == cfg.sleep_mode) page += " selected";
      page += ">";
      if (m == 1) page += "1 - 1 мин";
      else if (m == 2) page += "2 - 2 мин";
      else if (m == 3) page += "3 - 5 мин";
      else page += "4 - 20 сек (тест)";
      page += "</option>";
    }
    page += "</select></p>";
    page += "<p>TEMP_CHANGE (°C)<br><input name='temp_change' type='number' step='0.1' min='0.1' max='20' required style='width:100%;padding:8px' value='";
    page += tbuf;
    page += "'></p>";
    page += "<p><button type='submit' style='padding:10px 24px'>Запази</button></p></form>";
    page += "<p><a href='/ota'>Обновяване на фърмуера (OTA)</a></p>";
    page += "<p style='color:#666'>AP: TH06 | IP: 192.168.4.1</p></body></html>";
    server.send(200, "text/html", page);
  });

  server.on("/save", HTTP_POST, [&server]() {
    String ssid = server.arg("ssid");
    String id = server.arg("device_id");
    String ip = server.arg("ip");
    String pass = server.arg("pass");
    String temp = server.arg("temp_change");
    String mqttHost = server.arg("mqtt_host");
    String mqttUser = server.arg("mqtt_user");
    String mqttPass = server.arg("mqtt_pass");
    bool openNet = server.hasArg("open");
    ssid.trim();
    id.trim();
    ip.trim();
    mqttHost.trim();
    mqttUser.trim();

    DeviceConfig next = cfg;
    bool ok = ssid.length() > 0 && ssid.length() <= 32 && validDeviceId(id.c_str()) &&
              parseIp(ip.c_str(), next.ip);
    int mode = server.arg("sleep_mode").toInt();
    int mqttPort = server.arg("mqtt_port").toInt();
    float tc = temp.toFloat();
    if (mode < 1 || mode > 4 || !(tc > 0.0f) || tc > 20.0f) ok = false;
    if (mqttHost.length() == 0 || mqttHost.length() >= WIFI_CONFIG_HOST_LEN) ok = false;
    if (mqttUser.length() >= WIFI_CONFIG_USER_LEN) ok = false;
    if (mqttPort < 1 || mqttPort > 65535) ok = false;
    if (mqttPass.length() >= WIFI_CONFIG_PASS_LEN) ok = false;
    if (!ok) {
      server.send(200, "text/html",
                  "<html><body style='font-family:sans-serif'><h3>Невалидни данни.</h3>"
                  "<a href='/'>Назад</a></body></html>");
      return;
    }

    strncpy(next.ssid, ssid.c_str(), sizeof(next.ssid) - 1);
    next.ssid[sizeof(next.ssid) - 1] = 0;
    if (openNet) {
      memset(next.pass, 0, sizeof(next.pass));
    } else if (pass.length() > 0) {
      if (pass.length() > 64) {
        server.send(200, "text/html",
                    "<html><body style='font-family:sans-serif'><h3>Невалидна парола.</h3>"
                    "<a href='/'>Назад</a></body></html>");
        return;
      }
      strncpy(next.pass, pass.c_str(), sizeof(next.pass) - 1);
      next.pass[sizeof(next.pass) - 1] = 0;
    }
    strncpy(next.device_id, id.c_str(), sizeof(next.device_id) - 1);
    next.device_id[sizeof(next.device_id) - 1] = 0;
    strncpy(next.mqtt_host, mqttHost.c_str(), sizeof(next.mqtt_host) - 1);
    next.mqtt_host[sizeof(next.mqtt_host) - 1] = 0;
    strncpy(next.mqtt_user, mqttUser.c_str(), sizeof(next.mqtt_user) - 1);
    next.mqtt_user[sizeof(next.mqtt_user) - 1] = 0;
    if (mqttPass.length() > 0) {
      strncpy(next.mqtt_pass, mqttPass.c_str(), sizeof(next.mqtt_pass) - 1);
      next.mqtt_pass[sizeof(next.mqtt_pass) - 1] = 0;
    }
    next.mqtt_port = (uint16_t)mqttPort;
    next.sleep_mode = (uint8_t)mode;
    next.temp_change = roundf(tc * 100.0f) / 100.0f;

    if (!wifiConfigSave(&next)) {
      server.send(200, "text/html",
                  "<html><body style='font-family:sans-serif'><h3>Грешка при запис.</h3>"
                  "<a href='/'>Назад</a></body></html>");
      return;
    }

    cfg = next;
    // Нулираме retention: смяна на SLEEP_MODE променя кодирането на температурата/брояча,
    // така след рестарта стартира "чисто" и прави initial публикуване.
    retentionWrite(0);
    server.send(200, "text/html",
                "<html><body style='font-family:sans-serif'><h3>Запазено. Рестартиране...</h3></body></html>");
    delay(500);
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    ledSet(false);
    lt_reboot();
  });

  server.on("/ota", HTTP_GET, [&server]() {
    String page;
    page.reserve(900);
    page += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
    page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    page += "<title>TH06 OTA</title></head><body style='font-family:sans-serif;max-width:460px;margin:auto;padding:16px'>";
    page += "<h2>Обновяване на фърмуера</h2>";
    page += "<p>Избери <b>firmware.uf2</b> (същият файл, който се флашва).</p>";
    page += "<form method='post' action='/ota' enctype='multipart/form-data'>";
    page += "<p><input type='file' name='firmware' accept='.uf2' required style='width:100%;padding:8px'></p>";
    page += "<p><button type='submit' style='padding:10px 24px'>Качи и обнови</button></p></form>";
    page += "<p style='color:#666'>Не изключвай захранването по време на обновяването.</p>";
    page += "<p><a href='/'>Назад към настройките</a></p></body></html>";
    server.send(200, "text/html", page);
  });

  server.on("/ota", HTTP_POST,
            // Финален отговор след края на качването
            [&server]() {
              bool ok = otaOk && !otaActive;
              String page;
              page.reserve(700);
              page += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
              page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
              page += "<title>TH06 OTA</title></head><body style='font-family:sans-serif;max-width:460px;margin:auto;padding:16px'>";
              if (ok) {
                page += "<h3>Обновлението е записано успешно. Рестартиране...</h3>";
              } else {
                page += "<h3 style='color:#b00'>Грешка при обновяването.</h3>";
                page += "<p>Опитай отново с валиден <b>firmware.uf2</b> файл.</p>";
                page += "<p><a href='/ota'>Назад</a></p>";
              }
              page += "</body></html>";
              server.send(200, "text/html", page);
              if (ok) {
                delay(500);
                server.stop();
                WiFi.softAPdisconnect(true);
                WiFi.mode(WIFI_OFF);
                ledSet(false);
                lt_reboot();
              }
            },
            // Извиква се за всяка част от файла, докато се получава
            [&server]() {
              HTTPUpload &up = server.upload();
              if (up.status == UPLOAD_FILE_START) {
                otaActive = false;
                otaOk = false;
                String name = up.filename;
                name.toLowerCase();
                if (!name.endsWith(".uf2")) return;
                lt_ota_begin(&otaCtx, 0);  // размерът се взема от block_count
                otaActive = true;
                otaOk = true;
              } else if (up.status == UPLOAD_FILE_WRITE) {
                if (otaActive && otaOk) {
                  if (lt_ota_write(&otaCtx, up.buf, up.currentSize) != up.currentSize)
                    otaOk = false;
                }
              } else if (up.status == UPLOAD_FILE_END) {
                if (otaActive) {
                  if (!lt_ota_end(&otaCtx)) otaOk = false;
                  otaActive = false;
                }
              } else if (up.status == UPLOAD_FILE_ABORTED) {
                if (otaActive) {
                  lt_ota_end(&otaCtx);
                  otaActive = false;
                }
                otaOk = false;
              }
            });

  server.onNotFound([&server]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();

  unsigned long start = millis();
  while (millis() - start < CONFIG_PORTAL_TIMEOUT_MS) {
    server.handleClient();
    delay(2);
  }

  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  ledSet(false);
}

// Статичен IP от конфигурацията; шлюз/DNS се извеждат като x.y.z.1, маска /24
static bool wifiConnect() {
  IPAddress local_ip(cfg.ip[0], cfg.ip[1], cfg.ip[2], cfg.ip[3]);
  IPAddress gateway(cfg.ip[0], cfg.ip[1], cfg.ip[2], 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns = gateway;

  WiFi.mode(WIFI_STA);
  WiFi.config(local_ip, gateway, subnet, dns);
  WiFi.begin(cfg.ssid, cfg.pass);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

static bool mqttConnect() {
  return mqttQos1Connect(wifiClient, cfg.mqtt_host, cfg.mqtt_port, mqtt_client_id,
                         cfg.mqtt_user, cfg.mqtt_pass, 30);
}

// Ръчно форматиране на float (без %f), защото printf във firmware-а е орязан
static bool publishFloat(const char* topic, float value) {
  long scaled = (long)(value * 100.0f + (value >= 0 ? 0.5f : -0.5f));
  long whole = scaled / 100;
  long frac = labs(scaled % 100);
  char buf[16];

  if (scaled < 0 && whole == 0) {
    snprintf(buf, sizeof(buf), "-0.%02ld", frac);
  } else {
    snprintf(buf, sizeof(buf), "%ld.%02ld", whole, frac);
  }

  return mqttQos1Publish(wifiClient, topic, buf, MQTT_QOS, MQTT_RETAIN, MQTT_PUB_TIMEOUT_MS);
}

static bool publishSensors() {
  bool ok = publishFloat(topic_temp, current_temp);
  ok = publishFloat(topic_hum, current_hum) && ok;
  ok = publishFloat(topic_bat, current_bat) && ok;
  return ok;
}

// MQTT Discovery за Home Assistant: QoS0 + retained, за да оцелее след deep sleep.
// Изтриваме старите t1_* discovery конфигурации, за да не остават "дух" ентити.
static void publishDiscovery() {
  char payload[448];
  char dev[200];
  char cfg_topic[80];

  snprintf(dev, sizeof(dev),
           "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"TH06 %s\","
           "\"manufacturer\":\"DIY\",\"model\":\"BK7238 GXHT4x\"}",
           cfg.device_id, cfg.device_id);

  mqttQos1Publish(wifiClient, "homeassistant/sensor/t1_temp/config", "", 0, true,
                  MQTT_PUB_TIMEOUT_MS);
  mqttQos1Publish(wifiClient, "homeassistant/sensor/t1_hum/config", "", 0, true,
                  MQTT_PUB_TIMEOUT_MS);
  mqttQos1Publish(wifiClient, "homeassistant/sensor/t1_bat/config", "", 0, true,
                  MQTT_PUB_TIMEOUT_MS);

  snprintf(payload, sizeof(payload),
           "{\"name\":\"Temperature\",\"unique_id\":\"%s_temp\","
           "\"state_topic\":\"%s\",\"unit_of_measurement\":\"\\u00b0C\","
           "\"device_class\":\"temperature\",\"state_class\":\"measurement\",%s}",
           cfg.device_id, topic_temp, dev);
  snprintf(cfg_topic, sizeof(cfg_topic), "homeassistant/sensor/%s_temp/config",
           cfg.device_id);
  mqttQos1Publish(wifiClient, cfg_topic, payload, 0, true, MQTT_PUB_TIMEOUT_MS);

  snprintf(payload, sizeof(payload),
           "{\"name\":\"Humidity\",\"unique_id\":\"%s_hum\","
           "\"state_topic\":\"%s\",\"unit_of_measurement\":\"%%\","
           "\"device_class\":\"humidity\",\"state_class\":\"measurement\",%s}",
           cfg.device_id, topic_hum, dev);
  snprintf(cfg_topic, sizeof(cfg_topic), "homeassistant/sensor/%s_hum/config",
           cfg.device_id);
  mqttQos1Publish(wifiClient, cfg_topic, payload, 0, true, MQTT_PUB_TIMEOUT_MS);

  snprintf(payload, sizeof(payload),
           "{\"name\":\"Battery\",\"unique_id\":\"%s_bat\","
           "\"state_topic\":\"%s\",\"unit_of_measurement\":\"V\","
           "\"device_class\":\"voltage\",\"state_class\":\"measurement\","
           "\"suggested_display_precision\":2,%s}",
           cfg.device_id, topic_bat, dev);
  snprintf(cfg_topic, sizeof(cfg_topic), "homeassistant/sensor/%s_bat/config",
           cfg.device_id);
  mqttQos1Publish(wifiClient, cfg_topic, payload, 0, true, MQTT_PUB_TIMEOUT_MS);
}

// 3 проби за стабилност. Между тях се ползва delayMicroseconds (busy-wait),
// НЕ delay(), защото delay() отдава на RTOS и може да събуди I2C по средата (бъг при GXHT4x клонинги).
static bool measure(float *temp, float *hum) {
  float temp_sum = 0;
  float hum_sum = 0;
  int count = 0;

  for (int i = 0; i < 3; i++) {
    float t = 0;
    float h = 0;
    if (gxht4xRead(&t, &h)) {
      temp_sum += t;
      hum_sum += h;
      count++;
    }
    delayMicroseconds(20000);
  }

  if (count == 0) return false;

  *temp = temp_sum / count;
  *hum = hum_sum / count;
  return true;
}

// Retention е само 16 бита (AON регистър, оцелява deep sleep).
// Горните битове пазят кода на последната изпратена температура,
// долните counter_bits - брояч на wake-овете без промяна / retry състояние.
// raw == 0x0000 или 0xFFFF означава "няма запазено състояние" (студен старт).
static uint16_t packState(float temp, uint8_t counter) {
  long code = lroundf(temp * sleepProf->temp_scale) + sleepProf->temp_offset;
  if (code < 1) code = 1;
  if (code > sleepProf->temp_code_max) code = sleepProf->temp_code_max;
  return (uint16_t)(((uint16_t)code << sleepProf->counter_bits) |
                    (counter & sleepProf->counter_mask));
}

static bool unpackState(uint16_t raw, float *temp, uint8_t *counter) {
  if (raw == 0x0000 || raw == 0xFFFF) return false;
  uint16_t code = raw >> sleepProf->counter_bits;
  if (code == 0) return false;
  if (temp) *temp = ((float)code - sleepProf->temp_offset) / sleepProf->temp_scale;
  if (counter) *counter = raw & sleepProf->counter_mask;
  return true;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledSet(false);  // LED свети, докато е будно; гаси се преди deep sleep при false не свети
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Зареждаме конфигурацията от флаша. Ако липсва/е невалидна -> defaults.
  // Миграция от v2: старите полета се запазват, а MQTT полетата се пълнят с defaults.
  if (!wifiConfigLoad(&cfg)) {
    setConfigDefaults(&cfg);
  } else if (cfg.mqtt_host[0] == 0) {
    strncpy(cfg.mqtt_host, MQTT_HOST_DEFAULT, sizeof(cfg.mqtt_host) - 1);
    strncpy(cfg.mqtt_user, MQTT_USER_DEFAULT, sizeof(cfg.mqtt_user) - 1);
    strncpy(cfg.mqtt_pass, MQTT_PASS_DEFAULT, sizeof(cfg.mqtt_pass) - 1);
    cfg.mqtt_port = MQTT_PORT_DEFAULT;
  }
  if (cfg.sleep_mode < 1 || cfg.sleep_mode > 4) cfg.sleep_mode = SLEEP_MODE_DEFAULT;
  sleepProf = &SLEEP_PROFILES[cfg.sleep_mode];
  buildRuntimeStrings();

  // Задържан бутон при wake -> конфигурационен AP режим вместо мерене/публикуване
  if (buttonIsHeld()) {
    uint16_t raw = retentionRead();
    startConfigPortal();
    deepSleepRetain(raw, sleepProf->sleep_ms);
    return;
  }

  // Възстановяваме последно изпратената температура и брояча от retention
  uint16_t raw = retentionRead();
  float stored = 0;
  uint8_t counter = 0;
  bool has_stored = unpackState(raw, &stored, &counter);

  gxht4xInit(SCL_PIN, SDA_PIN);

  float temp = 0;
  float hum = 0;
  bool measured = measure(&temp, &hum);

  if (!measured) {
    deepSleepRetain(raw, sleepProf->sleep_ms);
    return;
  }

  bool initial = !has_stored;
  bool heartbeat = false;
  // Стойности >= heartbeat_wakes означават недовършен retry (виж логиката долу)
  bool retry_pending = has_stored && (counter >= sleepProf->heartbeat_wakes);
  uint8_t fail_count =
      retry_pending ? (uint8_t)(counter - sleepProf->heartbeat_wakes) : 0;
  bool need_send = true;

  // Пращаме само при промяна >= TEMP_CHANGE или при heartbeat (напр. на 2 часа).
  // Ако не трябва - само вдигаме брояча и заспиваме веднага (пести батерия).
  if (has_stored && !retry_pending) {
    uint8_t next_counter = (uint8_t)((counter + 1) & sleepProf->counter_mask);
    bool changed = fabsf(temp - stored) >= cfg.temp_change;
    heartbeat = next_counter >= sleepProf->heartbeat_wakes;
    need_send = changed || heartbeat;

    if (!need_send) {
      deepSleepRetain(packState(stored, next_counter), sleepProf->sleep_ms);
      return;
    }
  }

  current_temp = temp;
  current_hum = hum;

  bool sent = false;
  if (wifiConnect()) {
    bool connected = false;
    for (int i = 0; i < 3; i++) {
      connected = mqttConnect();
      if (connected) break;
      delay(1000);
    }
    if (connected) {
      // Батерията се мери СЛЕД свързване: при boot/радиото има спад на напрежението
      delay(300);
      float bat = 0;
      for (int i = 0; i < 8; i++) {
        bat += adcScanBatteryVoltage();
        delay(10);
      }
      current_bat = bat / 8.0f;

      // Discovery при първо стартиране и при retry; по желание и на heartbeat
      bool discovery = initial || retry_pending;
#if DISCOVERY_REFRESH_ON_HEARTBEAT
      if (heartbeat) discovery = true;
#endif
      if (discovery) publishDiscovery();

      sent = publishSensors();
      mqttQos1Disconnect(wifiClient);
    }
  }

  WiFi.disconnect();
  delay(100);
  ledSet(false);

  // Успех -> пазим новата температура и нулираме брояча
  if (sent) {
    deepSleepRetain(packState(temp, 0), sleepProf->sleep_ms);
    return;
  }

  // Неуспех -> броячът се мести в retry зоната (heartbeat_wakes + fail_count).
  // След 3 последователни неуспеха спим RETRY_BACKOFF_MS вместо нормалния интервал.
  int max_fail = (int)(sleepProf->counter_mask - sleepProf->heartbeat_wakes);
  if (fail_count < max_fail) fail_count++;
  uint8_t retry_counter = (uint8_t)(sleepProf->heartbeat_wakes + fail_count);
  uint16_t retry_raw = packState(has_stored ? stored : temp, retry_counter);
  uint32_t sleep_ms =
      (fail_count >= 3) ? RETRY_BACKOFF_MS : sleepProf->sleep_ms;
  deepSleepRetain(retry_raw, sleep_ms);
}

void loop() {
  delay(1000);
}
