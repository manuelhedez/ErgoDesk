#include <Arduino.h>
#include <math.h>
#include "app_types.h"
#include "sensors.h"
#include "actuators.h"
#include "nvs_storage.h"
#include "ble_provisioning.h"
#include "ring_buffer.h"
#include "mqtt_mosquitto.h"

ErgonomicState currentState  = STATE_OPTIMAL;
ErgonomicState previousState = STATE_OPTIMAL;

unsigned long lastSampleMs      = 0;
unsigned long badPostureStartMs = 0;
unsigned long sessionStartMs    = 0;

void setup() {
  Serial.begin(115200);
  initActuators();
  initMPU6050();
  loadConfigFromNVS();
  initBLEProvisioning();

  sessionStartMs = millis();
  Serial.println("[ErgoDesk IoT] Sistema iniciado. Listo para BLE y Edge.");
}

void loop() {
  // 1. Filtro complementario del IMU a ~50 Hz
  static unsigned long lastImuLoop = 0;
  if (millis() - lastImuLoop >= 20) {
    lastImuLoop = millis();
    updateIMUPitch();
  }

  // 2. Conectividad Wi-Fi / Mosquitto sin bloquear sensores
  ensureConnectivity(currentState);
  if (isMqttConnected()) {
    loopMqtt();
    TelemetryRecord cached;
    while (popOfflineRecord(cached)) {
      publishTelemetryJSON(cached.uptimeSec, cached.angle, cached.light,
                           (ErgonomicState)cached.stateCode, true);
      delay(30);
    }
  }

  // 3. Ciclo principal de decisión Edge cada 2 segundos
  if (millis() - lastSampleMs >= 2000) {
    lastSampleMs = millis();

    if (wifiSSID.isEmpty() || mqttHost.isEmpty()) {
      return;
    }

    float netAngle    = fabsf(currentPitch - angleOffset);
    currentLight      = readLightPercentage();
    bool isBadPosture = (netAngle > maxAngleDeg);
    bool isLowLight   = (currentLight < minLightPct);
    bool isBreakTime  = ((millis() - sessionStartMs) >= pauseLimitMs);
    bool triggerBuzzer = false;

    if (isBreakTime) {
      currentState  = STATE_ACTIVE_BREAK;
      triggerBuzzer = true;
    } else if (isBadPosture) {
      currentState = STATE_BAD_POSTURE;
      if (badPostureStartMs == 0) {
        badPostureStartMs = millis();
      } else if (millis() - badPostureStartMs >= 10000) {
        triggerBuzzer = true;
      }
    } else if (isLowLight) {
      badPostureStartMs = 0;
      currentState      = STATE_LOW_LIGHT;
    } else {
      badPostureStartMs = 0;
      currentState      = STATE_OPTIMAL;
    }

    updateMultimodalActuators(currentState, triggerBuzzer);

    unsigned long uptimeSec = millis() / 1000UL;
    if (isMqttConnected()) {
      publishTelemetryJSON(uptimeSec, netAngle, currentLight, currentState, false);
      if (currentState != previousState) {
        publishStateChangeAlert(previousState, currentState, netAngle);
      }
    } else {
      pushOfflineRecord(uptimeSec, netAngle, currentLight, (uint8_t)currentState);
    }

    previousState = currentState;
    Serial.printf("[EDGE] Angulo: %.1f° (Max: %.1f°) | Luz: %.1f%% | Estado: %s | Buffer: %u\n",
                  netAngle, maxAngleDeg, currentLight, stateToString(currentState),
                  (unsigned)getOfflineBufferCount());
  }
}