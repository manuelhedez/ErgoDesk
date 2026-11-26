#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// ============================================================================
// CONSTANTES DE TIEMPO Y UUIDs BLE GATT
// ============================================================================
constexpr unsigned long BLE_BOOT_WINDOW_MS = 30000;
constexpr unsigned long BLE_AFTER_DISC_MS  = 10000;

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_WIFI_SSID_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_WIFI_PASS_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_MQTT_HOST_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define CHAR_MQTT_USER_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ab"
#define CHAR_MQTT_PASS_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ac"
#define CHAR_MAX_ANGLE_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ad"
#define CHAR_MIN_LIGHT_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ae"
#define CHAR_CALIBRATE_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26af"
#define CHAR_APPLY_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26b0"

// ============================================================================
// INSTANCIA NVS, CREDENCIALES Y BANDERAS BLE
// ============================================================================
Preferences preferences;

String wifiSSID   = "";
String wifiPass   = "";
String mqttHost   = "";
uint16_t mqttPort = 8883;
String mqttUser   = "";
String mqttPass   = "";

unsigned long bleWindowStartMs  = 0;
unsigned long bleLastDiscMs     = 0;

volatile bool bleActive            = false;
volatile bool bleClientConnected   = false;
volatile bool bleNeedAdvertise     = false;
volatile bool bleFinishRequested   = false;
volatile bool bleHadClientOnce     = false;

bool hasRequiredCredentials() {
  return (!wifiSSID.isEmpty() && !mqttHost.isEmpty());
}

// ============================================================================
// MEMORIA NVS (PREFERENCES)
// ============================================================================
void loadConfigFromNVS() {
  preferences.begin("ergodesk", true);
  wifiSSID    = preferences.getString("ssid", "");
  wifiPass    = preferences.getString("wpass", "");
  mqttHost    = preferences.getString("mhost", "");
  mqttPort    = preferences.getUShort("mport", 8883);
  mqttUser    = preferences.getString("muser", "");
  mqttPass    = preferences.getString("mpass", "");
  maxAngleDeg = preferences.getFloat("maxAng", 18.0f);
  minLightPct = preferences.getFloat("minLit", 35.0f);
  angleOffset = preferences.getFloat("offset", 0.0f);
  preferences.end();
}

void saveStringNVS(const char* key, const String& val) {
  preferences.begin("ergodesk", false);
  preferences.putString(key, val);
  preferences.end();
}

void saveUShortNVS(const char* key, uint16_t val) {
  preferences.begin("ergodesk", false);
  preferences.putUShort(key, val);
  preferences.end();
}

void saveFloatNVS(const char* key, float val) {
  preferences.begin("ergodesk", false);
  preferences.putFloat(key, val);
  preferences.end();
}

String sanitizeMqttHost(String raw) {
  raw.trim();
  raw.replace("mqtts://", "");
  raw.replace("mqtt://", "");
  raw.replace("ssl://", "");
  raw.replace("wss://", "");
  raw.replace("https://", "");
  int slashIdx = raw.indexOf('/');
  if (slashIdx > 0) {
    raw = raw.substring(0, slashIdx);
  }
  raw.trim();
  return raw;
}

// ============================================================================
// SERVIDOR BLE GATT
// ============================================================================
class ErgoServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
    bleHadClientOnce = true;
    Serial.println("[BLE] Celular conectado. Ingresa tus datos tranquilamente.");
  }
  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    bleLastDiscMs = millis();
    Serial.println("[BLE] Celular desconectado.");
    if (bleActive && !bleFinishRequested) {
      bleNeedAdvertise = true;
    }
  }
};

class ConfigCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String uuid = pChar->getUUID().toString().c_str();
    uuid.toLowerCase();
    String val = pChar->getValue().c_str();
    val.trim();

    if (uuid == CHAR_WIFI_SSID_UUID) {
      wifiSSID = val;
      saveStringNVS("ssid", wifiSSID);
      Serial.printf("[BLE] SSID guardado: '%s'\n", wifiSSID.c_str());
    } else if (uuid == CHAR_WIFI_PASS_UUID) {
      wifiPass = val;
      saveStringNVS("wpass", wifiPass);
      Serial.println("[BLE] Password Wi-Fi guardado.");
    } else if (uuid == CHAR_MQTT_HOST_UUID) {
      String clean = sanitizeMqttHost(val);
      int colonIdx = clean.indexOf(':');
      if (colonIdx > 0) {
        mqttHost = clean.substring(0, colonIdx);
        mqttPort = (uint16_t)clean.substring(colonIdx + 1).toInt();
      } else {
        mqttHost = clean;
        mqttPort = 8883;
      }
      mqttHost.trim();
      saveStringNVS("mhost", mqttHost);
      saveUShortNVS("mport", mqttPort);
      Serial.printf("[BLE] MQTT Host guardado: '%s:%u'\n", mqttHost.c_str(), mqttPort);
    } else if (uuid == CHAR_MQTT_USER_UUID) {
      mqttUser = val;
      saveStringNVS("muser", mqttUser);
      Serial.printf("[BLE] MQTT User guardado: '%s'\n", mqttUser.c_str());
    } else if (uuid == CHAR_MQTT_PASS_UUID) {
      mqttPass = val;
      saveStringNVS("mpass", mqttPass);
      Serial.println("[BLE] MQTT Password guardado.");
    } else if (uuid == CHAR_MAX_ANGLE_UUID) {
      float newAngle = val.toFloat();
      if (newAngle >= 5.0f && newAngle <= 45.0f) {
        maxAngleDeg = newAngle;
        saveFloatNVS("maxAng", maxAngleDeg);
        Serial.printf("[BLE] Max Angle guardado: %.1f\n", maxAngleDeg);
      }
    } else if (uuid == CHAR_MIN_LIGHT_UUID) {
      float newLight = val.toFloat();
      if (newLight >= 5.0f && newLight <= 95.0f) {
        minLightPct = newLight;
        saveFloatNVS("minLit", minLightPct);
        Serial.printf("[BLE] Min Light guardado: %.1f\n", minLightPct);
      }
    } else if (uuid == CHAR_CALIBRATE_UUID && val == "1") {
      angleOffset = currentPitch;
      saveFloatNVS("offset", angleOffset);
      Serial.printf("[BLE] Postura calibrada. Offset: %.2f\n", angleOffset);
    } else if (uuid == CHAR_APPLY_UUID && val == "1") {
      Serial.println("[BLE] Orden recibida: cerrando BLE para conectar a Wi-Fi y HiveMQ...");
      bleFinishRequested = true;
    }
  }
};

void initBLEProvisioning() {
  BLEDevice::init("ErgoDesk-IoT-Node");
  BLEDevice::setMTU(185);
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ErgoServerCallbacks());
  BLEService* pService = pServer->createService(BLEUUID(SERVICE_UUID), 36);
  ConfigCharacteristicCallback* cb = new ConfigCharacteristicCallback();

  const char* writeChars[] = {
    CHAR_WIFI_SSID_UUID, CHAR_WIFI_PASS_UUID, CHAR_MQTT_HOST_UUID,
    CHAR_MQTT_USER_UUID, CHAR_MQTT_PASS_UUID, CHAR_CALIBRATE_UUID,
    CHAR_APPLY_UUID
  };
  for (const char* uuid : writeChars) {
    BLECharacteristic* c = pService->createCharacteristic(
      uuid, BLECharacteristic::PROPERTY_WRITE
    );
    c->setCallbacks(cb);
  }

  BLECharacteristic* cAngle = pService->createCharacteristic(
    CHAR_MAX_ANGLE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  cAngle->setValue(String(maxAngleDeg, 1).c_str());
  cAngle->setCallbacks(cb);

  BLECharacteristic* cLight = pService->createCharacteristic(
    CHAR_MIN_LIGHT_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  cLight->setValue(String(minLightPct, 1).c_str());
  cLight->setCallbacks(cb);

  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  bleActive = true;
  bleWindowStartMs = millis();
  Serial.printf("[BLE] Activo como 'ErgoDesk-IoT-Node'. FreeHeap con BLE: %u bytes\n", ESP.getFreeHeap());
}

void shutdownBLEAndStartNetwork() {
  Serial.println("[BLE] Apagando Bluetooth para liberar memoria RAM para TLS 1.2...");
  bleActive = false;
  bleNeedAdvertise = false;
  delay(250);
  BLEDevice::deinit(true);
  delay(250);
  Serial.printf("[NET] Bluetooth apagado. FreeHeap liberado: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("[NET] Config cargada -> SSID: '%s' | Broker: '%s:%u' | User: '%s'\n",
                wifiSSID.c_str(), mqttHost.c_str(), mqttPort, mqttUser.c_str());
}