#include "ble_provisioning.h"
#include "pins_and_uuids.h"
#include "nvs_storage.h"
#include "sensors.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

bool shouldReconnect = false;

class ConfigCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String uuid = pChar->getUUID().toString().c_str();
    String val  = pChar->getValue().c_str();
    val.trim();

    if (uuid == CHAR_WIFI_SSID_UUID) {
      wifiSSID = val;
      saveStringNVS("ssid", wifiSSID);
      shouldReconnect = true;
    } else if (uuid == CHAR_WIFI_PASS_UUID) {
      wifiPass = val;
      saveStringNVS("wpass", wifiPass);
      shouldReconnect = true;
    } else if (uuid == CHAR_MQTT_HOST_UUID) {
      // Acepta "192.168.1.50:1883" (Mosquitto local) o "host:8883" (TLS)
      int colonIdx = val.indexOf(':');
      if (colonIdx > 0) {
        mqttHost = val.substring(0, colonIdx);
        mqttPort = val.substring(colonIdx + 1).toInt();
      } else {
        mqttHost = val;
        mqttPort = DEFAULT_MQTT_PORT;
      }
      saveStringNVS("mhost", mqttHost);
      saveUShortNVS("mport", mqttPort);
      shouldReconnect = true;
    } else if (uuid == CHAR_MQTT_USER_UUID) {
      mqttUser = val;
      saveStringNVS("muser", mqttUser);
      shouldReconnect = true;
    } else if (uuid == CHAR_MQTT_PASS_UUID) {
      mqttPass = val;
      saveStringNVS("mpass", mqttPass);
      shouldReconnect = true;
    } else if (uuid == CHAR_MAX_ANGLE_UUID) {
      float newAngle = val.toFloat();
      if (newAngle >= 5.0f && newAngle <= 45.0f) {
        maxAngleDeg = newAngle;
        saveFloatNVS("maxAng", maxAngleDeg);
      }
    } else if (uuid == CHAR_MIN_LIGHT_UUID) {
      float newLight = val.toFloat();
      if (newLight >= 5.0f && newLight <= 95.0f) {
        minLightPct = newLight;
        saveFloatNVS("minLit", minLightPct);
      }
    } else if (uuid == CHAR_CALIBRATE_UUID && val == "1") {
      angleOffset = currentPitch;
      saveFloatNVS("offset", angleOffset);
    }
  }
};

void initBLEProvisioning() {
  BLEDevice::init("ErgoDesk-IoT-Node");
  BLEServer* pServer = BLEDevice::createServer();
  BLEService* pService = pServer->createService(BLEUUID(SERVICE_UUID), 30);

  ConfigCharacteristicCallback* cb = new ConfigCharacteristicCallback();

  const char* writeChars[] = {
    CHAR_WIFI_SSID_UUID, CHAR_WIFI_PASS_UUID, CHAR_MQTT_HOST_UUID,
    CHAR_MQTT_USER_UUID, CHAR_MQTT_PASS_UUID, CHAR_CALIBRATE_UUID
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
}