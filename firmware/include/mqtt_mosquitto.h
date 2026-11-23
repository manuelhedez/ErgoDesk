#pragma once
#include "app_types.h"

const char* stateToString(ErgonomicState s);
void ensureConnectivity(ErgonomicState& currentState);
bool isMqttConnected();
void loopMqtt();
void publishTelemetryJSON(unsigned long uptimeSec, float angle, float light,
                          ErgonomicState st, bool isBuffered);
void publishStateChangeAlert(ErgonomicState prevSt, ErgonomicState newSt, float angle);