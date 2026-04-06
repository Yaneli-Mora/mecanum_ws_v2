#pragma once
#include <Arduino.h>

// CrankHandler — simple DC motor via L298N (no encoder)
// Runs for a fixed duration to complete the crank task
// L298N wiring:
//   IN1 → pin 20
//   IN2 → pin 21
//   ENA → pin 22  (PWM speed control)

class CrankHandler {
public:
  void init(int in1, int in2, int pwm);
  bool turn();   // runs motor for CRANK_DURATION_MS, returns true when done
  void stop();

  static const int     SPEED           = 200;    // PWM 0-255 ⚠️ tune on field
  static const unsigned long CRANK_DURATION_MS = 4000;  // ⚠️ tune — ms to complete task
};
