#pragma once
#include "app_types.h"

void initActuators();
void setRGBColor(bool r, bool g, bool b);
void updateMultimodalActuators(ErgonomicState state, bool triggerAuditory);