#include "mqtt_mosquitto.h"
#include "nvs_storage.h"
#include "ble_provisioning.h"
#include "actuators.h"
#include "sensors.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#if __has_include("certs.h")
  #include "certs.h"
#else
  #include "certs.example.h"
#endif

static WiFiClientSecure espClientTLS;
static WiFiClient       espClientPlain;
static PubSubClient     mqttClient;
static unsigned long    lastReconnectMs = 0;

const char* stateToString(ErgonomicState s) {
  switch (s) {
    case STATE_OPTIMAL:      return "OPTIMO";
    case STATE_LOW_LIGHT:    return "POCA_LUZ";
    case STATE_BAD_POSTURE:  return "MALA_POSTURA";
    case STATE_ACTIVE_BREAK: return "PAUSA_ACTIVA";
    default:                 return "PROVISIONING";
  }
}

static void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

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

void ensureConnectivity(ErgonomicState& currentState) {
  if (wifiSSID.isEmpty() || mqttHost.isEmpty()) {
    currentState = STATE_PROVISIONING;
    updateMultimodalActuators(currentState, false);
    return;
  }

  if (shouldReconnect) {
    WiFi.disconnect();
    mqttClient.disconnect();
    shouldReconnect = false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnectMs > 5000) {
      lastReconnectMs = millis();
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    }
    return;
  }

  if (!mqttClient.connected()) {
    if (millis() - lastReconnectMs > 5000) {
      lastReconnectMs = millis();
      if (mqttPort == 8883) {
        espClientTLS.setCACert(ROOT_CA_CERT);
        mqttClient.setClient(espClientTLS);
      } else {
        mqttClient.setClient(espClientPlain);
      }

      mqttClient.setServer(mqttHost.c_str(), mqttPort);
      mqttClient.setCallback(onMqttMessage);

      String clientId = "ErgoDesk_ESP32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
      bool connected = mqttUser.isEmpty()
        ? mqttClient.connect(clientId.c_str())
        : mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str());

      if (connected) {
        mqttClient.subscribe(TOPIC_CONFIG);
      }
    }
  }
}

bool isMqttConnected() {
  return mqttClient.connected();
}

void loopMqtt() {
  mqttClient.loop();
}

void publishTelemetryJSON(unsigned long uptimeSec, float angle, float light,
                          ErgonomicState st, bool isBuffered) {
  char payload[220];
  snprintf(payload, sizeof(payload),
           "{\"uptime_s\":%lu,\"angulo_deg\":%.2f,\"luz_pct\":%.1f,"
           "\"estado\":\"%s\",\"umbral_ang\":%.1f,\"buffered\":%s}",
           uptimeSec, angle, light, stateToString(st), maxAngleDeg,
           isBuffered ? "true" : "false");
  mqttClient.publish(TOPIC_TELEMETRY, payload);
}

void publishStateChangeAlert(ErgonomicState prevSt, ErgonomicState newSt, float angle) {
  char alertMsg[128];
  snprintf(alertMsg, sizeof(alertMsg),
           "{\"evento\":\"CAMBIO_ESTADO\",\"de\":\"%s\",\"a\":\"%s\",\"angulo\":%.1f}",
           stateToString(prevSt), stateToString(newSt), angle);
  mqttClient.publish(TOPIC_ALERTS, alertMsg);
}S