#pragma once
#include <Arduino.h>

class CrankHandler {
public:
  void init(int in1, int in2, int pwm, int enc_a, int enc_b);
  bool turn(int turns = 3);  // spin N full turns, returns true on success
  void stop();

  // Called from ISR — must be public
  void encoderISR();

  // ⚠️ Set to your motor's actual counts per revolution
  static const int CPR   = 1200;
  static const int SPEED = 180;   // PWM 0-255
  static const unsigned long WATCHDOG_MS = 10000;  // 10s timeout

private:
  int in1_, in2_, pwm_;
  volatile long ticks_ = 0;
};
