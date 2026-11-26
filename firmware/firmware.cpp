#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================================
// PINES Y DIRECCIONES DE HARDWARE
// ============================================================================
constexpr uint8_t PIN_SDA    = 21;
constexpr uint8_t PIN_SCL    = 22;
constexpr uint8_t PIN_LDR    = 34;
constexpr uint8_t PIN_SERVO  = 32;
constexpr uint8_t PIN_LED_R  = 25;
constexpr uint8_t PIN_LED_G  = 26;
constexpr uint8_t PIN_LED_B  = 27;
constexpr uint8_t PIN_BUZZER = 14;
constexpr uint8_t IMU_ADDR   = 0x68;

Servo indicatorServo;

// ============================================================================
// ESTADOS ERGONÓMICOS Y VARIABLES DE BORDE (EDGE)
// ============================================================================
enum ErgonomicState {
  STATE_PROVISIONING,
  STATE_OPTIMAL,
  STATE_LOW_LIGHT,
  STATE_BAD_POSTURE,
  STATE_ACTIVE_BREAK
};

ErgonomicState currentState      = STATE_PROVISIONING;
ErgonomicState previousState     = STATE_PROVISIONING;
ErgonomicState lastActuatorState = (ErgonomicState)255;

float maxAngleDeg   = 18.0f;
float minLightPct   = 35.0f;
float angleOffset   = 0.0f;
uint32_t pauseLimitMs = 2700000; // 45 min

float currentPitch  = 0.0f;
float currentLight  = 100.0f;

unsigned long lastSampleMs      = 0;
unsigned long lastImuUpdateMs   = 0;
unsigned long badPostureStartMs = 0;
unsigned long sessionStartMs    = 0;
bool imuAvailable               = false;

const char* stateToString(ErgonomicState s) {
  switch (s) {
    case STATE_OPTIMAL:      return "OPTIMO";
    case STATE_LOW_LIGHT:    return "POCA_LUZ";
    case STATE_BAD_POSTURE:  return "MALA_POSTURA";
    case STATE_ACTIVE_BREAK: return "PAUSA_ACTIVA";
    default:                 return "PROVISIONING";
  }
}

// ============================================================================
// ACTUADORES MULTIMODALES (LED RGB, SERVO SG90, BUZZER)
// ============================================================================
void setRGBColor(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

void updateMultimodalActuators(ErgonomicState state, bool triggerAuditory) {
  if (state != lastActuatorState) {
    lastActuatorState = state;
    switch (state) {
      case STATE_PROVISIONING:
        setRGBColor(false, false, true); // Azul
        indicatorServo.write(0);
        digitalWrite(PIN_BUZZER, LOW);
        break;
      case STATE_OPTIMAL:
        setRGBColor(false, true, false); // Verde
        indicatorServo.write(0);
        digitalWrite(PIN_BUZZER, LOW);
        break;
      case STATE_LOW_LIGHT:
        setRGBColor(true, true, false);  // Amarillo
        indicatorServo.write(45);
        digitalWrite(PIN_BUZZER, LOW);
        break;
      case STATE_BAD_POSTURE:
      case STATE_ACTIVE_BREAK:
        setRGBColor(true, false, false); // Rojo
        indicatorServo.write(90);
        break;
    }
  }
  if ((state == STATE_BAD_POSTURE || state == STATE_ACTIVE_BREAK) && triggerAuditory) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(120);
    digitalWrite(PIN_BUZZER, LOW);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// ============================================================================
// SENSORES (GY-6500 I2C + LDR ADC)
// ============================================================================
void initIMU() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  uint8_t error = Wire.endTransmission(true);
  imuAvailable = (error == 0);
  lastImuUpdateMs = millis();
  if (!imuAvailable) {
    Serial.println("[WARN] No se detecto el GY-6500 en 0x68 (revisar SDA=21, SCL=22, VCC=3V3, GND).");
  }
}

void updateIMUPitch() {
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
  Wire.read(); Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();

  float ax = rawAx / 16384.0f;
  float ay = rawAy / 16384.0f;
  float az = rawAz / 16384.0f;
  float gx = rawGx / 131.0f;

  float accelPitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;
  currentPitch = 0.96f * (currentPitch + gx * dt) + 0.04f * accelPitch;
}

float readLightPercentage() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_LDR);
  }
  return constrain(((sum / 8.0f) / 4095.0f) * 100.0f, 0.0f, 100.0f);
}