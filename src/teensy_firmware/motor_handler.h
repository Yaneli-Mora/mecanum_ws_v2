#pragma once
#include <Arduino.h>

class MotorHandler {
public:
  void init();
  void setVelocity(int wheel, int16_t cmd);
  void stopAll();
  void lock()   { locked_ = true;  stopAll(); }
  void unlock() { locked_ = false; }
  bool isLocked() const { return locked_; }

private:
  static const int NUM_WHEELS = 4;
  // PWM + direction pin pairs per wheel
  const int PWM_PINS[NUM_WHEELS] = {2,  6,  10, 14};
  const int DIR_PINS[NUM_WHEELS] = {3,  7,  11, 15};
  bool locked_ = true;
};
