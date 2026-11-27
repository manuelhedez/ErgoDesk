/**
 * ============================================================================
 * ErgoDesk IoT — Monitor Ergonómico y de Fatiga Visual (Firmware Edge ESP32)
 * ============================================================================
 * Arquitectura implementada:
 *  - Capa 1 (Percepción y Edge): Adquisición cada 2 s de sensor IMU (I2C) con
 *    filtro complementario y fotorresistencia LDR (ADC 12 bits). Evaluación
 *    local de umbrales posturales/lumínicos y control de actuadores
 *    multimodales (LED RGB, servomotor SG90 y buzzer) conforme a EN 301 549.
 *  - Capa 2 (Red y Conectividad): Aprovisionamiento inalámbrico mediante
 *    servidor BLE GATT con ventana temporal de apagado y posterior enlace
 *    Wi-Fi STA con cifrado TLS 1.2 (MQTTS puerto 8883).
 *  - Capa 3 (Middleware): Publicación en tópicos estructurados
 *    (ergodesk/telemetria, ergodesk/alertas) y suscripción a ergodesk/config.
 *  - Seguridad (Unidad 4): Almacenamiento persistente de credenciales en
 *    partición NVS (Preferences) sin secretos en código y validación de
 *    certificado raíz X.509 (CA Root) con sincronización NTP.
 *  - Resiliencia (Unidad 5): Cola circular offline (Ring Buffer) de 240
 *    registros con drenado ordenado FIFO (250 ms/paquete) tras reconexión.
 *
 * Asignación de pines de hardware:
 *  - IMU MPU6050/GY-6500 (I2C): SDA = GPIO 21, SCL = GPIO 22, VCC = 3.3 V
 *  - Divisor LDR + 10 kOhm:     GPIO 34 (ADC1, entrada analógica 12 bits)
 *  - Servomotor SG90 (PWM):     GPIO 32, alimentación 5 V
 *  - LED RGB (cátodo común):    R = GPIO 25, G = GPIO 26, B = GPIO 27 (330 Ohm)
 *  - Buzzer activo (+):         GPIO 14
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ESP32Servo.h>
#include <time.h>
#include <math.h>

// ============================================================================
// 1. DEFINICIÓN DE PINES, PARÁMETROS ERGONÓMICOS Y CONSTANTES DE OPERACIÓN
// ============================================================================
constexpr uint8_t PIN_SDA    = 21;   // Línea de datos del bus I2C para el IMU
constexpr uint8_t PIN_SCL    = 22;   // Línea de reloj del bus I2C para el IMU
constexpr uint8_t PIN_LDR    = 34;   // Entrada ADC1 de 12 bits para la LDR
constexpr uint8_t PIN_SERVO  = 32;   // Señal PWM para el indicador mecánico SG90
constexpr uint8_t PIN_LED_R  = 25;   // Canal rojo del LED RGB
constexpr uint8_t PIN_LED_G  = 26;   // Canal verde del LED RGB
constexpr uint8_t PIN_LED_B  = 27;   // Canal azul del LED RGB
constexpr uint8_t PIN_BUZZER = 14;   // Salida digital para aviso acústico
constexpr uint8_t IMU_ADDR   = 0x68; // Dirección I2C por defecto del sensor IMU

// Parámetros base de referencia postural (eje roll respecto a postura neutra)
constexpr float DEFAULT_ANGLE_OFFSET  = -85.0f; // Ángulo neutro de referencia (°)
constexpr float DEFAULT_MAX_ANGLE_DEG = 15.0f;  // Desviación angular máxima permitida (°)
constexpr uint8_t CALIBRATION_VERSION = 2;      // Control de versión de calibración en NVS

// Política TLS 1.2: false habilita la validación estricta mediante ROOT_CA_CERT
constexpr bool USE_INSECURE_TLS = false;

// Ventanas temporales de gestión BLE para limitar exposición de publicidad (OWASP I2)
constexpr unsigned long BLE_BOOT_WINDOW_MS = 30000; // Ventana inicial con credenciales en NVS
constexpr unsigned long BLE_AFTER_DISC_MS  = 10000; // Espera tras desconexión de cliente BLE

// ============================================================================
// 2. IDENTIFICADORES UUID DEL PERFIL BLE GATT (ERGO-DESK PROVISIONING SERVICE)
// ============================================================================
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // Servicio GATT principal
#define CHAR_WIFI_SSID_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8" // Escritura de SSID Wi-Fi
#define CHAR_WIFI_PASS_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9" // Escritura de contraseña Wi-Fi
#define CHAR_MQTT_HOST_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa" // Escritura de host/puerto MQTT
#define CHAR_MQTT_USER_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ab" // Escritura de usuario MQTTS
#define CHAR_MQTT_PASS_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ac" // Escritura de contraseña/token MQTTS
#define CHAR_MAX_ANGLE_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ad" // Lectura/escritura de umbral angular
#define CHAR_MIN_LIGHT_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ae" // Lectura/escritura de umbral lumínico
#define CHAR_CALIBRATE_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26af" // Disparo de calibración postural neutra
#define CHAR_APPLY_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26b0" // Cierre inmediato de BLE e inicio de red

// ============================================================================
// 3. CERTIFICADO RAÍZ TLS 1.2 (ISRG ROOT X1 — AUTENTICACIÓN DE BROKER CLOUD)
// ============================================================================
const char* ROOT_CA_CERT = R"EOF(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----)EOF";

// ============================================================================
// 4. INSTANCIAS DE COMUNICACIÓN, ESTADO GLOBAL Y RING BUFFER OFFLINE
// ============================================================================
Preferences preferences;        // Interfaz de persistencia en partición NVS
WiFiClientSecure espClientTLS;  // Cliente TCP seguro sobre TLS 1.2 (puerto 8883)
WiFiClient espClientPlain;      // Cliente TCP sin cifrado para broker local (1883)
PubSubClient mqttClient;        // Cliente MQTT (patrón Publish/Subscribe)
Servo indicatorServo;           // Controlador PWM del servomotor SG90

// Parámetros dinámicos cargados desde NVS (cero credenciales fijas en firmware)
String wifiSSID       = "";
String wifiPass       = "";
String mqttHost       = "";
uint16_t mqttPort     = 8883;
String mqttUser       = "";
String mqttPass       = "";
float maxAngleDeg     = DEFAULT_MAX_ANGLE_DEG;
float minLightPct     = 35.0f;
float angleOffset     = DEFAULT_ANGLE_OFFSET;
uint32_t pauseLimitMs = 2700000; // Intervalo de pausa activa por defecto (45 min)

// Tópicos MQTT estructurados según la arquitectura de Middleware
const char* TOPIC_TELEMETRY = "ergodesk/telemetria";
const char* TOPIC_ALERTS    = "ergodesk/alertas";
const char* TOPIC_CONFIG    = "ergodesk/config";

// Estados operativos de la máquina ergonómica local (Edge)
enum ErgonomicState {
  STATE_PROVISIONING, // Ventana activa de configuración BLE GATT
  STATE_OPTIMAL,      // Postura e iluminación dentro de umbrales saludables
  STATE_LOW_LIGHT,    // Déficit de iluminación ambiental detectado
  STATE_BAD_POSTURE,  // Desviación angular sostenida por encima del umbral
  STATE_ACTIVE_BREAK  // Tiempo de jornada continua excedido; requiere pausa activa
};

ErgonomicState currentState      = STATE_PROVISIONING;
ErgonomicState previousState     = STATE_PROVISIONING;
ErgonomicState lastActuatorState = (ErgonomicState)255;

// Variables de muestreo sensorial y temporización no bloqueante (millis)
float currentRoll               = DEFAULT_ANGLE_OFFSET;
float currentLight              = 100.0f;
unsigned long lastSampleMs      = 0;
unsigned long lastImuUpdateMs   = 0;
unsigned long badPostureStartMs = 0;
unsigned long sessionStartMs    = 0;
unsigned long lastWifiRetryMs   = 0;
unsigned long lastMqttRetryMs   = 0;
unsigned long bleWindowStartMs  = 0;
unsigned long bleLastDiscMs     = 0;

// Indicadores de estado de hardware y sincronización de red
bool imuAvailable        = false;
bool imuAngleInitialized = false;
bool wifiConnectedLogged = false;
bool ntpConfigured       = false;
bool ntpSynced           = false;

// Banderas volátiles de sincronización entre callbacks BLE y el bucle principal
volatile bool bleActive          = false;
volatile bool bleClientConnected = false;
volatile bool bleNeedAdvertise   = false;
volatile bool bleFinishRequested = false;
volatile bool bleHadClientOnce   = false;

// Estructura de registro de telemetría para almacenamiento local ante desconexión
struct TelemetryRecord {
  unsigned long uptimeSec; // Marca temporal relativa desde arranque (segundos)
  float angle;             // Desviación angular neta respecto a postura neutra (°)
  float rawRoll;           // Ángulo roll filtrado absoluto del IMU (°)
  float light;             // Nivel de iluminación ambiental normalizado (%)
  uint8_t stateCode;       // Código numérico del estado ergonómico
};

// Cola circular (Ring Buffer) con capacidad para 240 registros en SRAM
constexpr size_t BUFFER_CAPACITY = 240;
TelemetryRecord offlineBuffer[BUFFER_CAPACITY];
size_t bufHead  = 0; // Índice de inserción de nuevos registros
size_t bufTail  = 0; // Índice de extracción en orden estricto FIFO
size_t bufCount = 0; // Cantidad actual de registros almacenados

/**
 * Inserta un registro en la cola circular offline.
 * Si el buffer alcanza su capacidad máxima (240), sobrescribe el registro más antiguo.
 */
void pushOfflineRecord(unsigned long uptimeSec, float angle, float rawRoll, float light, uint8_t stateCode) {
  offlineBuffer[bufHead] = {uptimeSec, angle, rawRoll, light, stateCode};
  bufHead = (bufHead + 1) % BUFFER_CAPACITY;
  if (bufCount < BUFFER_CAPACITY) {
    bufCount++;
  } else {
    bufTail = (bufTail + 1) % BUFFER_CAPACITY;
  }
}

/**
 * Extrae el registro más antiguo del Ring Buffer en orden FIFO.
 * Retorna true si existía un elemento disponible para drenado.
 */
bool popOfflineRecord(TelemetryRecord &record) {
  if (bufCount == 0) return false;
  record = offlineBuffer[bufTail];
  bufTail = (bufTail + 1) % BUFFER_CAPACITY;
  bufCount--;
  return true;
}

/**
 * Verifica que existan los parámetros mínimos de red (SSID y host MQTT) en NVS.
 */
bool hasRequiredCredentials() {
  return (!wifiSSID.isEmpty() && !mqttHost.isEmpty());
}

// ============================================================================
// 5. ACTUADORES MULTIMODALES (ACCESIBILIDAD UNIVERSAL — NORMA EN 301 549)
// ============================================================================

/**
 * Establece el estado lógico de los canales R, G y B del LED de señalización visual.
 */
void setRGBColor(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

/**
 * Coordina la retroalimentación visual (LED RGB), mecánica/cinética (servomotor)
 * y acústica progresiva (buzzer) según el estado ergonómico evaluado en el Edge.
 */
void updateMultimodalActuators(ErgonomicState state, bool triggerAuditory) {
  if (state != lastActuatorState) {
    lastActuatorState = state;
    switch (state) {
      case STATE_PROVISIONING:
        setRGBColor(false, false, true); // Azul: modo aprovisionamiento BLE activo
        indicatorServo.write(0);
        digitalWrite(PIN_BUZZER, LOW);
        break;
      case STATE_OPTIMAL:
        setRGBColor(false, true, false); // Verde: postura e iluminación óptimas
        indicatorServo.write(0);
        digitalWrite(PIN_BUZZER, LOW);
        break;
      case STATE_LOW_LIGHT:
        setRGBColor(true, true, false);  // Amarillo: déficit de iluminación ambiental
        indicatorServo.write(45);        // Posición intermedia de aviso mecánico
        digitalWrite(PIN_BUZZER, LOW);
        break;
      case STATE_BAD_POSTURE:
      case STATE_ACTIVE_BREAK:
        setRGBColor(true, false, false); // Rojo: alerta postural o pausa activa
        indicatorServo.write(90);        // Elevación completa del indicador físico
        break;
    }
  }

  // El aviso acústico se activa únicamente ante alerta persistente o pausa activa
  if ((state == STATE_BAD_POSTURE || state == STATE_ACTIVE_BREAK) && triggerAuditory) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(120);
    digitalWrite(PIN_BUZZER, LOW);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// ============================================================================
// 6. ADQUISICIÓN Y FUSIÓN SENSORIAL (IMU I2C + LDR ADC 12 BITS)
// ============================================================================

/**
 * Inicializa el bus I2C y despierta el sensor IMU escribiendo 0x00 en el
 * registro de gestión de energía (PWR_MGMT_1 = 0x6B). Gestiona fallos sin bloqueo.
 */
void initIMU() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  uint8_t error = Wire.endTransmission(true);
  imuAvailable = (error == 0);
  imuAngleInitialized = false;
  lastImuUpdateMs = millis();

  if (!imuAvailable) {
    Serial.println("[WARN] Sensor IMU no detectado en direccion I2C 0x68.");
  }
}

/**
 * Lee acelerómetro y giroscopio por I2C (14 bytes desde registro 0x3B) y calcula
 * el ángulo roll mediante un filtro complementario (96% giroscopio, 4% acelerómetro)
 * para amortiguar vibraciones transitorias y evitar deriva acumulativa.
 */
void updateIMURoll() {
  if (!imuAvailable) return;

  unsigned long now = millis();
  float dt = (now - lastImuUpdateMs) / 1000.0f;
  lastImuUpdateMs = now;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.02f;

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;

  Wire.requestFrom(IMU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Descarte de bytes de temperatura interna
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Descarte de eje Gy
  Wire.read(); Wire.read(); // Descarte de eje Gz

  // Conversión a unidades físicas (±2g -> 16384 LSB/g; ±250°/s -> 131 LSB/°/s)
  float ax = rawAx / 16384.0f;
  float ay = rawAy / 16384.0f;
  float az = rawAz / 16384.0f;
  float gx = rawGx / 131.0f;

  // Cálculo trigonométrico de inclinación roll desde el vector de gravedad
  float accelRoll = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;

  if (!imuAngleInitialized) {
    currentRoll = accelRoll;
    imuAngleInitialized = true;
  } else {
    currentRoll = 0.96f * (currentRoll + gx * dt) + 0.04f * accelRoll;
  }
}

/**
 * Promedia 8 muestras consecutivas del ADC de 12 bits conectado al divisor
 * de voltaje de la LDR y normaliza la lectura al rango porcentual 0.0 - 100.0%.
 */
float readLightPercentage() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_LDR);
  }
  return constrain(((sum / 8.0f) / 4095.0f) * 100.0f, 0.0f, 100.0f);
}

// ============================================================================
// 7. GESTIÓN DE PERSISTENCIA EN PARTICIÓN NVS (PREFERENCES)
// ============================================================================

/**
 * Recupera credenciales Wi-Fi, configuración MQTT y parámetros ergonómicos
 * almacenados en la partición no volátil (NVS) del ESP32.
 */
void loadConfigFromNVS() {
  preferences.begin("ergodesk", false);
  wifiSSID    = preferences.getString("ssid", "");
  wifiPass    = preferences.getString("wpass", "");
  mqttHost    = preferences.getString("mhost", "");
  mqttPort    = preferences.getUShort("mport", 8883);
  mqttUser    = preferences.getString("muser", "");
  mqttPass    = preferences.getString("mpass", "");
  minLightPct = preferences.getFloat("minLit", 35.0f);

  // Migración única de parámetros de calibración cuando incrementa CALIBRATION_VERSION
  uint8_t savedCalVer = preferences.getUChar("calVer", 0);
  if (savedCalVer < CALIBRATION_VERSION) {
    angleOffset = DEFAULT_ANGLE_OFFSET;
    maxAngleDeg = DEFAULT_MAX_ANGLE_DEG;
    preferences.putFloat("offset", angleOffset);
    preferences.putFloat("maxAng", maxAngleDeg);
    preferences.putUChar("calVer", CALIBRATION_VERSION);
    Serial.printf("[NVS] Calibracion inicializada -> Offset: %.1f° | MaxAngle: %.1f°\n",
                  angleOffset, maxAngleDeg);
  } else {
    maxAngleDeg = preferences.getFloat("maxAng", DEFAULT_MAX_ANGLE_DEG);
    angleOffset = preferences.getFloat("offset", DEFAULT_ANGLE_OFFSET);
  }
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

/**
 * Sanitiza la cadena del host MQTT eliminando prefijos de protocolo y rutas
 * sobrantes antes de persistirla en NVS (validación de entrada OWASP I3).
 */
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
// 8. SERVIDOR BLE GATT PARA APROVISIONAMIENTO Y CALIBRACIÓN INICIAL
// ============================================================================

/**
 * Manejador de eventos de conexión y desconexión del servidor BLE GATT.
 */
class ErgoServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
    bleHadClientOnce = true;
    Serial.println("[BLE] Cliente GATT conectado.");
  }

  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    bleLastDiscMs = millis();
    Serial.println("[BLE] Cliente GATT desconectado.");
    if (bleActive && !bleFinishRequested) {
      bleNeedAdvertise = true;
    }
  }
};

/**
 * Manejador de escritura en características GATT. Valida rangos, persiste
 * configuraciones en NVS y permite calibrar postura o finalizar aprovisionamiento.
 */
class ConfigCharacteristicCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String uuid = pChar->getUUID().toString().c_str();
    uuid.toLowerCase();
    String val = pChar->getValue().c_str();
    val.trim();

    if (uuid == CHAR_WIFI_SSID_UUID) {
      wifiSSID = val;
      saveStringNVS("ssid", wifiSSID);
      Serial.printf("[BLE] SSID almacenado en NVS: '%s'\n", wifiSSID.c_str());
    } else if (uuid == CHAR_WIFI_PASS_UUID) {
      wifiPass = val;
      saveStringNVS("wpass", wifiPass);
      Serial.println("[BLE] Credencial Wi-Fi almacenada en NVS.");
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
      Serial.printf("[BLE] Host MQTT almacenado: '%s:%u'\n", mqttHost.c_str(), mqttPort);
    } else if (uuid == CHAR_MQTT_USER_UUID) {
      mqttUser = val;
      saveStringNVS("muser", mqttUser);
      Serial.printf("[BLE] Usuario MQTT almacenado: '%s'\n", mqttUser.c_str());
    } else if (uuid == CHAR_MQTT_PASS_UUID) {
      mqttPass = val;
      saveStringNVS("mpass", mqttPass);
      Serial.println("[BLE] Credencial MQTT almacenada en NVS.");
    } else if (uuid == CHAR_MAX_ANGLE_UUID) {
      float newAngle = val.toFloat();
      if (newAngle >= 5.0f && newAngle <= 45.0f) {
        maxAngleDeg = newAngle;
        saveFloatNVS("maxAng", maxAngleDeg);
        Serial.printf("[BLE] Umbral angular actualizado: %.1f°\n", maxAngleDeg);
      }
    } else if (uuid == CHAR_MIN_LIGHT_UUID) {
      float newLight = val.toFloat();
      if (newLight >= 5.0f && newLight <= 95.0f) {
        minLightPct = newLight;
        saveFloatNVS("minLit", minLightPct);
        Serial.printf("[BLE] Umbral luminico actualizado: %.1f%%\n", minLightPct);
      }
    } else if (uuid == CHAR_CALIBRATE_UUID && val == "1") {
      angleOffset = currentRoll;
      saveFloatNVS("offset", angleOffset);
      Serial.printf("[BLE] Calibracion postural aplicada. Nuevo offset: %.2f°\n", angleOffset);
    } else if (uuid == CHAR_APPLY_UUID && val == "1") {
      Serial.println("[BLE] Solicitud de transicion a modo red recibida.");
      bleFinishRequested = true;
    }
  }
};

/**
 * Inicializa la pila BLE, configura MTU extendido (185 bytes) para soportar
 * nombres de dominio cloud y expone el servicio GATT de aprovisionamiento.
 */
void initBLEProvisioning() {
  BLEDevice::init("ErgoDesk-IoT-Node");
  BLEDevice::setMTU(185);

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ErgoServerCallbacks());

  BLEService* pService = pServer->createService(BLEUUID(SERVICE_UUID), 36);
  ConfigCharacteristicCallback* cb = new ConfigCharacteristicCallback();

  // Características de solo escritura (las credenciales no son legibles por GATT)
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

  // Características de lectura y escritura para consulta y ajuste de umbrales
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
  Serial.printf("[BLE] Servicio GATT activo ('ErgoDesk-IoT-Node'). FreeHeap: %u bytes\n", ESP.getFreeHeap());
}

/**
 * Libera completamente la pila Bluetooth para recuperar memoria SRAM necesaria
 * para el handshake criptográfico TLS 1.2 y evitar interferencia de radio con Wi-Fi.
 */
void shutdownBLEAndStartNetwork() {
  Serial.println("[BLE] Desactivando Bluetooth y liberando memoria SRAM para TLS 1.2.");
  bleActive = false;
  bleNeedAdvertise = false;
  delay(250);
  BLEDevice::deinit(true);
  delay(250);
  Serial.printf("[NET] Stack BLE liberado. FreeHeap disponible: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("[NET] Parametros activos -> SSID: '%s' | Broker: '%s:%u' | Usuario: '%s'\n",
                wifiSSID.c_str(), mqttHost.c_str(), mqttPort, mqttUser.c_str());
}

// ============================================================================
// 9. CONECTIVIDAD WI-FI, SINCRONIZACIÓN NTP Y TRANSPORTE MQTTS (TLS 1.2)
// ============================================================================

/**
 * Convierte el código de estado ergonómico a etiqueta textual explícita
 * compatible con accesibilidad WCAG 2.1 AA (Criterio 1.4.1).
 */
const char* stateToString(ErgonomicState s) {
  switch (s) {
    case STATE_OPTIMAL:      return "OPTIMO";
    case STATE_LOW_LIGHT:    return "POCA_LUZ";
    case STATE_BAD_POSTURE:  return "MALA_POSTURA";
    case STATE_ACTIVE_BREAK: return "PAUSA_ACTIVA";
    default:                 return "PROVISIONING";
  }
}

/**
 * Procesa consignas remotas recibidas por el tópico ergodesk/config
 * (retroalimentación Cloud -> Edge para ajuste adaptativo de pausas y umbrales).
 */
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
    angleOffset = currentRoll;
    saveFloatNVS("offset", angleOffset);
    Serial.printf("[MQTT] Calibracion remota aplicada. Nuevo offset: %.2f°\n", angleOffset);
  }
}

/**
 * Sincroniza el reloj del sistema vía NTP (UTC-6) para permitir la verificación
 * de fechas de vigencia del certificado raíz X.509 en conexiones TLS 1.2.
 */
void syncTimeNTPBlocking() {
  if (ntpSynced) return;
  if (!ntpConfigured) {
    configTime(-6 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    ntpConfigured = true;
    Serial.print("[NTP] Sincronizando reloj para validacion TLS");
  }
  unsigned long start = millis();
  while (time(nullptr) < 1700000000UL && (millis() - start < 8000)) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  if (time(nullptr) >= 1700000000UL) {
    ntpSynced = true;
    Serial.println("[NTP] Reloj sincronizado correctamente.");
  } else {
    Serial.println("[NTP] Tiempo de espera agotado; validacion de certificado sujeta a hora del sistema.");
  }
}

/**
 * Máquina de estados no bloqueante para mantener la conectividad Wi-Fi STA
 * y la sesión MQTT/MQTTS sin interrumpir el muestreo sensorial en el Edge.
 */
void ensureConnectivity() {
  if (bleActive) return;

  unsigned long now = millis();

  // Gestión de reconexión Wi-Fi con reintento periódico cada 6 s
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnectedLogged = false;
    if (lastWifiRetryMs == 0 || (now - lastWifiRetryMs >= 6000)) {
      lastWifiRetryMs = now;
      Serial.printf("[WIFI] Iniciando enlace con '%s' (FreeHeap: %u bytes)\n",
                    wifiSSID.c_str(), ESP.getFreeHeap());
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    }
    return;
  }

  if (!wifiConnectedLogged) {
    wifiConnectedLogged = true;
    Serial.printf("[WIFI] Enlace establecido. IP: %s | FreeHeap: %u bytes\n",
                  WiFi.localIP().toString().c_str(), ESP.getFreeHeap());
    if (!USE_INSECURE_TLS && mqttPort == 8883) {
      syncTimeNTPBlocking();
    }
  }

  // Gestión de conexión MQTT/MQTTS (puerto 8883 usa TLS 1.2 con ROOT_CA_CERT)
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
      Serial.printf("[MQTT] Conectando a %s:%u como '%s' (TLS_CA=%s, FreeHeap=%u)...\n",
                    mqttHost.c_str(), mqttPort,
                    mqttUser.isEmpty() ? "<anon>" : mqttUser.c_str(),
                    (!USE_INSECURE_TLS && mqttPort == 8883) ? "activo" : "inactivo",
                    ESP.getFreeHeap());

      bool connected = mqttUser.isEmpty()
        ? mqttClient.connect(clientId.c_str())
        : mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPass.c_str());

      if (connected) {
        Serial.println("[MQTT] Sesion establecida con el broker.");
        mqttClient.subscribe(TOPIC_CONFIG);
      } else {
        char tlsErrBuf[128] = {0};
        if (mqttPort == 8883) {
          espClientTLS.lastError(tlsErrBuf, sizeof(tlsErrBuf));
        }
        Serial.printf("[MQTT] Error de conexion. rc=%d | TLS: %s\n",
                      mqttClient.state(),
                      mqttPort == 8883 ? tlsErrBuf : "plain");
      }
    }
  }
}

/**
 * Serializa y publica un registro de telemetría JSON en ergodesk/telemetria,
 * indicando si corresponde a muestreo en vivo o drenado desde el Ring Buffer.
 */
void publishTelemetryJSON(unsigned long uptimeSec, float angle, float rawRoll, float light,
                          ErgonomicState st, bool isBuffered) {
  char payload[280];
  snprintf(payload, sizeof(payload),
           "{\"uptime_s\":%lu,\"angulo_deg\":%.2f,\"roll_raw\":%.1f,\"luz_pct\":%.1f,"
           "\"estado\":\"%s\",\"umbral_ang\":%.1f,\"buffered\":%s,\"buf_count\":%u}",
           uptimeSec, angle, rawRoll, light, stateToString(st), maxAngleDeg,
           isBuffered ? "true" : "false", (unsigned int)bufCount);
  mqttClient.publish(TOPIC_TELEMETRY, payload);
}

/**
 * Publica eventos discretos de transición de estado ergonómico en ergodesk/alertas.
 */
void publishAlertJSON(ErgonomicState fromState, ErgonomicState toState, float angle) {
  if (!mqttClient.connected()) return;
  char payload[180];
  snprintf(payload, sizeof(payload),
           "{\"evento\":\"CAMBIO_ESTADO\",\"de\":\"%s\",\"a\":\"%s\",\"angulo\":%.1f}",
           stateToString(fromState), stateToString(toState), angle);
  mqttClient.publish(TOPIC_ALERTS, payload);
}

/**
 * Ejecuta el drenado controlado (Flush) del Ring Buffer en orden FIFO a una
 * cadencia de 250 ms por mensaje para evitar saturar el broker y Node-RED.
 */
void flushOfflineBuffer() {
  if (!mqttClient.connected() || bufCount == 0) return;

  static unsigned long lastFlushMs = 0;
  unsigned long now = millis();
  if (now - lastFlushMs >= 250) {
    lastFlushMs = now;
    TelemetryRecord rec;
    if (popOfflineRecord(rec)) {
      publishTelemetryJSON(rec.uptimeSec, rec.angle, rec.rawRoll, rec.light,
                           (ErgonomicState)rec.stateCode, true);
      Serial.printf("[FLUSH] Registro FIFO sincronizado. Pendientes en buffer: %u\n",
                    (unsigned int)bufCount);
    }
  }
}

// ============================================================================
// 10. INICIALIZACIÓN DEL SISTEMA (SETUP)
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== ErgoDesk IoT — Inicio de Firmware ===");

  // Configuración de pines de salida y periféricos de hardware
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  analogReadResolution(12);
  indicatorServo.attach(PIN_SERVO);
  indicatorServo.write(0);

  // Inicialización del bus I2C del IMU y carga de configuración desde NVS
  initIMU();
  loadConfigFromNVS();
  sessionStartMs = millis();

  // Arranque inicial en modo BLE GATT para permitir aprovisionamiento o recalibración
  initBLEProvisioning();
  currentState = STATE_PROVISIONING;
  updateMultimodalActuators(currentState, false);

  if (hasRequiredCredentials()) {
    Serial.printf("[BOOT] Credenciales detectadas en NVS (SSID: '%s', Broker: '%s:%u').\n",
                  wifiSSID.c_str(), mqttHost.c_str(), mqttPort);
    Serial.printf("[BOOT] Ventana BLE activa por %lu s antes de iniciar enlace Wi-Fi/MQTTS.\n",
                  BLE_BOOT_WINDOW_MS / 1000UL);
  } else {
    Serial.println("[BOOT] Sin credenciales en NVS. En espera de aprovisionamiento BLE GATT.");
  }
}

// ============================================================================
// 11. BUCLE PRINCIPAL DE CONTROL EDGE, RED Y RESILIENCIA (LOOP)
// ============================================================================
void loop() {
  unsigned long now = millis();

  // Fusión sensorial continua del ángulo roll del IMU (~50 Hz)
  static unsigned long lastImuTick = 0;
  if (now - lastImuTick >= 20) {
    lastImuTick = now;
    updateIMURoll();
  }

  // Gestión del ciclo de vida de la ventana BLE GATT antes de activar Wi-Fi
  if (bleActive) {
    if (bleNeedAdvertise) {
      bleNeedAdvertise = false;
      delay(100);
      BLEDevice::startAdvertising();
    }

    bool readyToSwitch = false;
    if (bleFinishRequested && hasRequiredCredentials()) {
      readyToSwitch = true;
    } else if (hasRequiredCredentials() && !bleClientConnected) {
      if (bleHadClientOnce && (now - bleLastDiscMs >= BLE_AFTER_DISC_MS)) {
        readyToSwitch = true;
      } else if (!bleHadClientOnce && (now - bleWindowStartMs >= BLE_BOOT_WINDOW_MS)) {
        readyToSwitch = true;
      }
    }

    if (readyToSwitch) {
      shutdownBLEAndStartNetwork();
      currentState = STATE_OPTIMAL;
      previousState = STATE_OPTIMAL;
      updateMultimodalActuators(currentState, false);
    } else {
      return;
    }
  }

  // Gestión de conectividad Wi-Fi/MQTTS y drenado FIFO del buffer offline
  ensureConnectivity();
  if (mqttClient.connected()) {
    mqttClient.loop();
    flushOfflineBuffer();
  }

  // Ciclo principal de evaluación ergonómica Edge y telemetría cada 2 s
  if (now - lastSampleMs >= 2000) {
    lastSampleMs = now;

    // Desviación angular neta respecto al ángulo neutro calibrado
    float netAngle = fabs(currentRoll - angleOffset);
    currentLight = readLightPercentage();

    bool triggerBuzzer = false;
    ErgonomicState nextState = STATE_OPTIMAL;

    // Evaluación de umbrales ergonómicos con prioridad: Pausa -> Postura -> Luz
    if (now - sessionStartMs >= pauseLimitMs) {
      nextState = STATE_ACTIVE_BREAK;
      triggerBuzzer = true;
    } else if (netAngle > maxAngleDeg) {
      if (badPostureStartMs == 0) {
        badPostureStartMs = now;
      }
      // Temporizador de tolerancia de 10 s antes de escalar alerta postural
      if (now - badPostureStartMs >= 10000) {
        nextState = STATE_BAD_POSTURE;
        triggerBuzzer = ((now - badPostureStartMs) >= 16000); // Aviso sonoro progresivo
      } else {
        nextState = (currentLight < minLightPct) ? STATE_LOW_LIGHT : STATE_OPTIMAL;
      }
    } else {
      badPostureStartMs = 0;
      if (currentLight < minLightPct) {
        nextState = STATE_LOW_LIGHT;
      } else {
        nextState = STATE_OPTIMAL;
      }
    }

    // Actualización determinista local de actuadores multimodales (Edge)
    currentState = nextState;
    updateMultimodalActuators(currentState, triggerBuzzer);

    // Publicación de alerta discreta ante transición de estado
    if (currentState != previousState) {
      publishAlertJSON(previousState, currentState, netAngle);
      previousState = currentState;
    }

    // Publicación de telemetría en tiempo real o encolado en Ring Buffer offline
    unsigned long uptimeSec = now / 1000;
    if (mqttClient.connected() && bufCount == 0) {
      publishTelemetryJSON(uptimeSec, netAngle, currentRoll, currentLight, currentState, false);
    } else {
      pushOfflineRecord(uptimeSec, netAngle, currentRoll, currentLight, (uint8_t)currentState);
    }

    // Registro de diagnóstico en consola serie
    Serial.printf("[EDGE] Roll:%.1f° | Offset:%.1f° | Desv:%.1f° (Max:%.1f°) | Luz:%.1f%% | Estado:%s | MQTT:%s | Buf:%u\n",
                  currentRoll, angleOffset, netAngle, maxAngleDeg, currentLight,
                  stateToString(currentState),
                  mqttClient.connected() ? "ON" : "OFF",
                  (unsigned int)bufCount);
  }
}