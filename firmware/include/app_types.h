#pragma once
#include <Arduino.h>

constexpr const char* TOPIC_TELEMETRY = "ergodesk/telemetria";
constexpr const char* TOPIC_ALERTS    = "ergodesk/alertas";
constexpr const char* TOPIC_CONFIG    = "ergodesk/config";

enum ErgonomicState : uint8_t {
  STATE_PROVISIONING = 0, // Azul: Esperando configuración BLE
  STATE_OPTIMAL,          // Verde: Postura e iluminación óptimas
  STATE_LOW_LIGHT,        // Amarillo: Fatiga visual (poca luz)
  STATE_BAD_POSTURE,      // Rojo: Mala postura sostenida
  STATE_ACTIVE_BREAK      // Rojo + Servo + Buzzer: Pausa activa requerida
};

struct TelemetryRecord {
  unsigned long uptimeSec;
  float angle;
  float light;
  uint8_t stateCode;
};

constexpr size_t BUFFER_CAPACITY = 150; // ~5 min de respaldo (1 muestra c/2s)