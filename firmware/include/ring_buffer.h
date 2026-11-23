#pragma once
#include "app_types.h"

void pushOfflineRecord(unsigned long uptimeSec, float angle, float light, uint8_t stateCode);
bool popOfflineRecord(TelemetryRecord& record);
size_t getOfflineBufferCount();