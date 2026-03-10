#pragma once
#include <Arduino.h>

// TB6612FNG motor driver
// Each channel: IN1, IN2, PWM
// Shared:       STBY (pull HIGH to enable)
//
// Truth table:
//   IN1=H IN2=L PWM → forward
//   IN1=L IN2=H PWM → reverse
//   IN1=L IN2=L  x  → coast (free spin)
//   IN1=H IN2=H  x  → brake (short brake)
//
// Wheel order: [front_left, front_right, rear_left, rear_right]
// Two TB6612 chips assumed (each handles 2 motors):
//   Chip A: front_left, front_right
//   Chip B: rear_left,  rear_right

class MotorHandler {
public:
  void init();
  void setVelocity(int wheel, int16_t cmd);
  void stopAll();
  void brakeAll();
  void lock()   { locked_ = true;  brakeAll(); }
  void unlock() { locked_ = false; }
  bool isLocked() const { return locked_; }

private:
  static const int NUM_WHEELS = 4;

  // ── Pin assignments ── UPDATE to match your wiring ───────────────────
  // Wheel:               FL   FR   RL   RR
  const int IN1_PINS[NUM_WHEELS] = { 30,  35, 24, 28};
  const int IN2_PINS[NUM_WHEELS] = { 32,  34, 26, 29};
  const int PWM_PINS[NUM_WHEELS] = { 31,  33, 25, 27};

  // STBY pin for each TB6612 chip (tie together if using one chip for all 4)
  // If you have 2 chips, wire each STBY separately
  const int STBY_PINS[2] = {21, 22};   // chip A (FL/FR), chip B (RL/RR)
  // ─────────────────────────────────────────────────────────────────────

  bool locked_ = true;
};
