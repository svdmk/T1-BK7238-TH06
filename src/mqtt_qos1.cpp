#include "mqtt_qos1.h"

#include <string.h>

extern "C" {
#include "MQTTPacket.h"
}

static Client *s_client = nullptr;
static uint16_t s_packetId = 0;
static uint32_t s_deadline = 0;

static int transportGet(uint8_t *buf, int count) {
  if (!s_client) return -1;

  int got = 0;
  while (got < count) {
    int avail = s_client->available();
    if (avail > 0) {
      int n = s_client->read(buf + got, count - got);
      if (n <= 0) break;
      got += n;
    } else {
      if (!s_client->connected()) break;
      if ((int32_t)(millis() - s_deadline) >= 0) break;
      delay(1);
    }
  }
  return got;
}

static uint16_t nextPacketId() {
  if (++s_packetId == 0) s_packetId = 1;
  return s_packetId;
}

bool mqttQos1Connect(Client &client, const char *host, uint16_t port,
                     const char *clientId, const char *user, const char *pass,
                     uint16_t keepAlive) {
  s_client = &client;
  s_packetId = 0;

  if (!client.connect(host, port)) {
    s_client = nullptr;
    return false;
  }

  unsigned char buf[256];
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.clientID.cstring = (char *)clientId;
  data.keepAliveInterval = keepAlive;
  data.cleansession = 1;
  data.username.cstring = (char *)user;
  data.password.cstring = (char *)pass;

  int len = MQTTSerialize_connect(buf, sizeof(buf), &data);
  if (len <= 0 || client.write(buf, len) != (size_t)len) {
    s_client = nullptr;
    client.stop();
    return false;
  }

  s_deadline = millis() + 5000;
  int rc = MQTTPacket_read(buf, sizeof(buf), transportGet);
  if (rc != CONNACK) {
    s_client = nullptr;
    client.stop();
    return false;
  }

  unsigned char sessionPresent = 0;
  unsigned char connackRc = 0;
  if (MQTTDeserialize_connack(&sessionPresent, &connackRc, buf, sizeof(buf)) != 1 ||
      connackRc != 0) {
    s_client = nullptr;
    client.stop();
    return false;
  }

  return true;
}

bool mqttQos1Publish(Client &client, const char *topic, const char *payload,
                     uint8_t qos, bool retain, uint32_t timeoutMs) {
  s_client = &client;

  unsigned char buf[512];
  MQTTString topicString = MQTTString_initializer;
  topicString.cstring = (char *)topic;

  uint16_t packetId = nextPacketId();
  int payloadLen = strlen(payload);

  if (qos == 0) {
    int len = MQTTSerialize_publish(buf, sizeof(buf), 0, 0, retain ? 1 : 0, 0,
                                    topicString, (unsigned char *)payload,
                                    payloadLen);
    if (len <= 0) return false;
    return client.write(buf, len) == (size_t)len;
  }

  for (int attempt = 0; attempt < 3; attempt++) {
    unsigned char dup = (attempt == 0) ? 0 : 1;
    int len = MQTTSerialize_publish(buf, sizeof(buf), dup, qos,
                                    retain ? 1 : 0, packetId, topicString,
                                    (unsigned char *)payload, payloadLen);
    if (len <= 0) return false;
    if (client.write(buf, len) != (size_t)len) return false;

    s_deadline = millis() + timeoutMs;
    int rc = MQTTPacket_read(buf, sizeof(buf), transportGet);

    if (rc == PUBACK) {
      unsigned char type = 0;
      unsigned char ackDup = 0;
      unsigned short ackId = 0;
      if (MQTTDeserialize_ack(&type, &ackDup, &ackId, buf, sizeof(buf)) == 1 &&
          ackId == packetId) {
        return true;
      }
    }
  }

  return false;
}

void mqttQos1Disconnect(Client &client) {
  unsigned char buf[2];
  int len = MQTTSerialize_disconnect(buf, sizeof(buf));
  if (len > 0) client.write(buf, len);
  client.flush();
  client.stop();
  s_client = nullptr;
}
