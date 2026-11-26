#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

// ============================================================================
// CLIENTES DE RED, TÓPICOS MQTT Y RING BUFFER (240 REGISTROS)
// ============================================================================
WiFiClientSecure espClientTLS;
WiFiClient espClientPlain;
PubSubClient mqttClient;

const char* TOPIC_TELEMETRY = "ergodesk/telemetria";
const char* TOPIC_ALERTS    = "ergodesk/alertas";
const char* TOPIC_CONFIG    = "ergodesk/config";

unsigned long lastWifiRetryMs   = 0;
unsigned long lastMqttRetryMs   = 0;
bool wifiConnectedLogged        = false;
bool ntpConfigured              = false;
bool ntpSynced                  = false;

struct TelemetryRecord {
  unsigned long uptimeSec;
  float angle;
  float light;
  uint8_t stateCode;
};

constexpr size_t BUFFER_CAPACITY = 240;
TelemetryRecord offlineBuffer[BUFFER_CAPACITY];
size_t bufHead  = 0;
size_t bufTail  = 0;
size_t bufCount = 0;

void pushOfflineRecord(unsigned long uptimeSec, float angle, float light, uint8_t stateCode) {
  offlineBuffer[bufHead] = {uptimeSec, angle, light, stateCode};
  bufHead = (bufHead + 1) % BUFFER_CAPACITY;
  if (bufCount < BUFFER_CAPACITY) {
    bufCount++;
  } else {
    bufTail = (bufTail + 1) % BUFFER_CAPACITY;
  }
}

bool popOfflineRecord(TelemetryRecord &record) {
  if (bufCount == 0) return false;
  record = offlineBuffer[bufTail];
  bufTail = (bufTail + 1) % BUFFER_CAPACITY;
  bufCount--;
  return true;
}

// ============================================================================
// CONECTIVIDAD WI-FI + MQTTS + NTP
// ============================================================================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  Serial.printf("[MQTT RX] %s -> %s\n", topic, msg.c_str());

  if (msg.startsWith("MAX_ANGLE:")) {
    float val = msg.substring(10).toFloat();
    if (val >= 5.0f && val <= 45.0f) {
      maxAngleDeg = val;
      saveFloatNVS("maxAng", maxAngleDeg);
    }
  } else if (msg.startsWith("MIN_LIGHT:")) {
    float val = msg.substring(10).toFloat();
    if (val >= 5.0f && val <= 95.0f) {
      minLightPct = val;
      saveFloatNVS("minLit", minLightPct);
    }
  } else if (msg.startsWith("PAUSE_MIN:")) {
    uint32_t mins = msg.substring(10).toInt();
    if (mins >= 1 && mins <= 120) {
      pauseLimitMs = mins * 60000UL;
    }
  } else if (msg == "CALIBRATE") {
    angleOffset = currentPitch;
    saveFloatNVS("offset", angleOffset);
  }
}

void syncTimeNTPBlocking() {
  if (ntpSynced) return;
  if (!ntpConfigured) {
    configTime(-6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    ntpConfigured = true;
    Serial.print("[NTP] Sincronizando hora para TLS");
  }
  unsigned long start = millis();
  while (time(nullptr) < 1700000000UL && (millis() - start < 8000)) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  if (time(nullptr) >= 1700000000UL) {
    ntpSynced = true;
    Serial.println("[NTP] Hora sincronizada correctamente.");
  } else {
    Serial.println("[NTP] Aviso: no se obtuvo hora en 8s (revisar acceso a internet).");
  }
}

void ensureConnectivity() {
  if (bleActive) return;
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    wifiConnectedLogged = false;
    if (lastWifiRetryMs == 0 || (now - lastWifiRetryMs >= 6000)) {
      lastWifiRetryMs = now;
      Serial.printf("[WIFI] Conectando a '%s'... (FreeHeap: %u)\n",
                    wifiSSID.c_str(), ESP.getFreeHeap());
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    }
    return;
  }

  if (!wifiConnectedLogged) {
    wifiConnectedLogged = true;
    Serial.printf("[WIFI] Conectado! IP: %s | FreeHeap: %u\n",
                  WiFi.localIP().toString().c_str(), ESP.getFreeHeap());
    if (!USE_INSECURE_TLS && mqttPort == 8883) {
      syncTimeNTPBlocking();
    }
  }

  if (!mqttClient.connected()) {
    if (lastMqttRetryMs == 0 || (now - lastMqttRetryMs >= 6000)) {
      lastMqttRetryMs = now;
      if (mqttPort == 8883) {
        if (USE_INSECURE_TLS) {
          espClientTLS.setInsecure();
        } else {
          espClientTLS.setCACert(ROOT_CA_CERT);
        }
        espClientTLS.setHandshakeTimeout(20);
        mqttClient.setClient(espClientTLS);
      } else {
        mqttClient.setClient(espClientPlain);
      }
      mqttClient.setBufferSize(512);
      mqttClient.setServer(mqttHost.c_str(), mqttPort);
      mqttClient.setCallback(onMqttMessage);

      String clientId = "ErgoDesk_ESP32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
      Serial.printf("[MQTT] Conectando a %s:%u como '%s' (insecure=%s, FreeHeap=%u)...\n",
                    mqttHost.c_str(), mqttPort,
                    mqttUser.isEmpty() ? "<anon>" : mqttUser.c_str(),
                    USE_INSECURE_TLS ? "true" : "false",
                    ESP.getFreeHeap());

      bool connected = mqttUser.isEmpty()
        ? mqttClient.connect(clientId.c_str())
        : mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str());

      if (connected) {
        Serial.println("[MQTT] CONECTADO EXITOSAMENTE AL BROKER!");
        mqttClient.subscribe(TOPIC_CONFIG);
      } else {
        char tlsErrBuf[128] = {0};
        if (mqttPort == 8883) {
          espClientTLS.lastError(tlsErrBuf, sizeof(tlsErrBuf));
        }
        Serial.printf("[MQTT] Fallo de conexion. rc=%d | TLS: %s\n",
                      mqttClient.state(),
                      mqttPort == 8883 ? tlsErrBuf : "plain");
      }
    }
  }
}

void publishTelemetryJSON(unsigned long uptimeSec, float angle, float light,
                          ErgonomicState st, bool isBuffered) {
  char payload[240];
  snprintf(payload, sizeof(payload),
           "{\"uptime_s\":%lu,\"angulo_deg\":%.2f,\"luz_pct\":%.1f,"
           "\"estado\":\"%s\",\"umbral_ang\":%.1f,\"buffered\":%s,\"buf_count\":%u}",
           uptimeSec, angle, light, stateToString(st), maxAngleDeg,
           isBuffered ? "true" : "false", (unsigned)bufCount);
  mqttClient.publish(TOPIC_TELEMETRY, payload);
}
