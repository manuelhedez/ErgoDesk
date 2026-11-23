#pragma once
#include <Arduino.h>

extern float currentPitch;
extern float currentLight;
extern bool imuAvailable;

void initMPU6050();
void updateIMUPitch();
float readLightPercentage();