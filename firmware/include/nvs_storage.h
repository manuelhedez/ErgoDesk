#pragma once
#include <Arduino.h>

extern String   wifiSSID;
extern String   wifiPass;
extern String   mqttHost;
extern uint16_t mqttPort;
extern String   mqttUser;
extern String   mqttPass;
extern float    maxAngleDeg;
extern float    minLightPct;
extern float    angleOffset;
extern uint32_t pauseLimitMs;

void loadConfigFromNVS();
void saveStringNVS(const char* key, const String& val);
void saveFloatNVS(const char* key, float val);
void saveUShortNVS(const char* key, uint16_t val);