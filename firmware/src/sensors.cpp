#include "sensors.h"
#include "pins_and_uuids.h"
#include <Wire.h>
#include <math.h>

float currentPitch = 0.0f;
float currentLight = 100.0f;
bool imuAvailable  = false;

static unsigned long lastImuUpdateMs = 0;

void initMPU6050() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Despertar MPU6050
  uint8_t error = Wire.endTransmission(true);
  imuAvailable = (error == 0);
  lastImuUpdateMs = millis();
}

void updateIMUPitch() {
  if (!imuAvailable) return;

  unsigned long now = millis();
  float dt = (now - lastImuUpdateMs) / 1000.0f;
  lastImuUpdateMs = now;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.02f;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return;

  Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Temperatura
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Gyro Y
  Wire.read(); Wire.read(); // Gyro Z

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
  float rawAvg = sum / 8.0f;
  return constrain((rawAvg / 4095.0f) * 100.0f, 0.0f, 100.0f);
}