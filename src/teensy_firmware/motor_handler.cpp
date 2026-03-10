#include "motor_handler.h"

void MotorHandler::init() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
  }
  stopAll();
}

void MotorHandler::setVelocity(int wheel, int16_t cmd) {
  if (locked_) return;
  digitalWrite(DIR_PINS[wheel], cmd >= 0 ? HIGH : LOW);
  analogWrite(PWM_PINS[wheel], map(abs(cmd), 0, 32767, 0, 255));
}

void MotorHandler::stopAll() {
  for (int i = 0; i < NUM_WHEELS; i++) analogWrite(PWM_PINS[i], 0);
}
