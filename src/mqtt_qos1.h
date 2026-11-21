#pragma once

#include <Arduino.h>
#include <Client.h>

bool mqttQos1Connect(Client &client, const char *host, uint16_t port,
                     const char *clientId, const char *user, const char *pass,
                     uint16_t keepAlive);
bool mqttQos1Publish(Client &client, const char *topic, const char *payload,
                     uint8_t qos, bool retain, uint32_t timeoutMs);
void mqttQos1Disconnect(Client &client);
