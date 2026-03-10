#include "motor_handler.h"

void MotorHandler::init() {
  // Enable both TB6612 chips via STBY pins
  for (int i = 0; i < 2; i++) {
    pinMode(STBY_PINS[i], OUTPUT);
    digitalWrite(STBY_PINS[i], HIGH);   // HIGH = active (not standby)
  }

  // Set all motor pins as outputs
  for (int i = 0; i < NUM_WHEELS; i++) {
    pinMode(IN1_PINS[i], OUTPUT);
    pinMode(IN2_PINS[i], OUTPUT);
    pinMode(PWM_PINS[i], OUTPUT);
  }

  brakeAll();
}

void MotorHandler::setVelocity(int wheel, int16_t cmd) {
  if (locked_) return;
  if (wheel < 0 || wheel >= NUM_WHEELS) return;

  uint8_t pwm = (uint8_t)map(abs(cmd), 0, 32767, 0, 255);

  if (cmd > 0) {
    // Forward
    digitalWrite(IN1_PINS[wheel], HIGH);
    digitalWrite(IN2_PINS[wheel], LOW);
  } else if (cmd < 0) {
    // Reverse
    digitalWrite(IN1_PINS[wheel], LOW);
    digitalWrite(IN2_PINS[wheel], HIGH);
  } else {
    // Zero command → brake
    digitalWrite(IN1_PINS[wheel], HIGH);
    digitalWrite(IN2_PINS[wheel], HIGH);
    pwm = 0;
  }

  analogWrite(PWM_PINS[wheel], pwm);
}

// Coast — let motors spin freely (low power draw, slower stop)
void MotorHandler::stopAll() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    digitalWrite(IN1_PINS[i], LOW);
    digitalWrite(IN2_PINS[i], LOW);
    analogWrite(PWM_PINS[i], 0);
  }
}

// Short-brake — actively resists rotation (faster stop, more current)
void MotorHandler::brakeAll() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    digitalWrite(IN1_PINS[i], HIGH);
    digitalWrite(IN2_PINS[i], HIGH);
    analogWrite(PWM_PINS[i], 0);
  }
}
