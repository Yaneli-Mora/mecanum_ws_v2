#pragma once
#include <Arduino.h>

class UltrasonicHandler {
public:
  void     init();
  uint16_t readMM(int idx);
  void     update();
  uint16_t getDistanceMM(int i) const { return distances_mm_[i]; }

private:
  static const int NUM    = 4;
  const int TRIG[NUM]     = {2, 4, 6, 8};   // FL, FR, RL, RR
  const int ECHO[NUM]     = {3, 5, 7, 9};
  uint16_t distances_mm_[NUM] = {9999,9999,9999,9999};
};
