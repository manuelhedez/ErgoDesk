#include "ring_buffer.h"

static TelemetryRecord offlineBuffer[BUFFER_CAPACITY];
static size_t bufHead  = 0;
static size_t bufTail  = 0;
static size_t bufCount = 0;

void pushOfflineRecord(unsigned long uptimeSec, float angle, float light, uint8_t stateCode) {
  offlineBuffer[bufHead] = {uptimeSec, angle, light, stateCode};
  bufHead = (bufHead + 1) % BUFFER_CAPACITY;
  if (bufCount < BUFFER_CAPACITY) {
    bufCount++;
  } else {
    bufTail = (bufTail + 1) % BUFFER_CAPACITY;
  }
}

bool popOfflineRecord(TelemetryRecord& record) {
  if (bufCount == 0) return false;
  record = offlineBuffer[bufTail];
  bufTail = (bufTail + 1) % BUFFER_CAPACITY;
  bufCount--;
  return true;
}

size_t getOfflineBufferCount() {
  return bufCount;
}