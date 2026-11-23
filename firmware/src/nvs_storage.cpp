#include "nvs_storage.h"
#include "pins_and_uuids.h"
#include <Preferences.h>

static Preferences preferences;

String   wifiSSID     = "";
String   wifiPass     = "";
String   mqttHost     = "";
uint16_t mqttPort     = DEFAULT_MQTT_PORT;
String   mqttUser     = "";
String   mqttPass     = "";
float    maxAngleDeg  = DEFAULT_MAX_ANGLE_DEG;
float    minLightPct  = DEFAULT_MIN_LIGHT_PCT;
float    angleOffset  = 0.0f;
uint32_t pauseLimitMs = DEFAULT_PAUSE_LIMIT_MS;

void loadConfigFromNVS() {
  preferences.begin("ergodesk", true);
  wifiSSID    = preferences.getString("ssid", "");
  wifiPass    = preferences.getString("wpass", "");
  mqttHost    = preferences.getString("mhost", "");
  mqttPort    = preferences.getUShort("mport", DEFAULT_MQTT_PORT);
  mqttUser    = preferences.getString("muser", "");
  mqttPass    = preferences.getString("mpass", "");
  maxAngleDeg = preferences.getFloat("maxAng", DEFAULT_MAX_ANGLE_DEG);
  minLightPct = preferences.getFloat("minLit", DEFAULT_MIN_LIGHT_PCT);
  angleOffset = preferences.getFloat("offset", 0.0f);
  preferences.end();
}

void saveStringNVS(const char* key, const String& val) {
  preferences.begin("ergodesk", false);
  preferences.putString(key, val);
  preferences.end();
}

void saveFloatNVS(const char* key, float val) {
  preferences.begin("ergodesk", false);
  preferences.putFloat(key, val);
  preferences.end();
}

void saveUShortNVS(const char* key, uint16_t val) {
  preferences.begin("ergodesk", false);
  preferences.putUShort(key, val);
  preferences.end();
}