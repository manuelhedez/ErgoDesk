#include "actuators.h"
#include "pins_and_uuids.h"
#include <Arduino.h>
#include <ESP32Servo.h>

static Servo indicatorServo;

void initActuators() {
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  analogReadResolution(12);
  indicatorServo.attach(PIN_SERVO);
  indicatorServo.write(0);
}

void setRGBColor(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

void updateMultimodalActuators(ErgonomicState state, bool triggerAuditory) {
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
      if (triggerAuditory) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(120);
        digitalWrite(PIN_BUZZER, LOW);
      } else {
        digitalWrite(PIN_BUZZER, LOW);
      }
      break;
  }
}