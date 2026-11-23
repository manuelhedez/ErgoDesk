#pragma once
#include <Arduino.h>

// Definición exacta de pines (Protoboard)
constexpr uint8_t PIN_SDA    = 21;  // MPU6050 SDA (I2C)
constexpr uint8_t PIN_SCL    = 22;  // MPU6050 SCL (I2C)
constexpr uint8_t PIN_LDR    = 34;  // Divisor LDR + 10kΩ (ADC1)
constexpr uint8_t PIN_SERVO  = 13;  // Servomotor SG90 (PWM)
constexpr uint8_t PIN_LED_R  = 25;  // LED RGB - Rojo (330Ω)
constexpr uint8_t PIN_LED_G  = 26;  // LED RGB - Verde (330Ω)
constexpr uint8_t PIN_LED_B  = 27;  // LED RGB - Azul (330Ω)
constexpr uint8_t PIN_BUZZER = 14;  // Buzzer Activo (+)

constexpr uint8_t MPU6050_ADDR = 0x68;

// Valores por defecto (Mosquitto local 1883 / TLS 8883)
constexpr uint16_t DEFAULT_MQTT_PORT      = 1883;
constexpr float    DEFAULT_MAX_ANGLE_DEG  = 18.0f;
constexpr float    DEFAULT_MIN_LIGHT_PCT  = 35.0f;
constexpr uint32_t DEFAULT_PAUSE_LIMIT_MS = 2700000UL; // 45 min

// Perfil GATT Propio (BLE Provisioning)
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_WIFI_SSID_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_WIFI_PASS_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_MQTT_HOST_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define CHAR_MQTT_USER_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ab"
#define CHAR_MQTT_PASS_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ac"
#define CHAR_MAX_ANGLE_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ad"
#define CHAR_MIN_LIGHT_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ae"
#define CHAR_CALIBRATE_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26af"