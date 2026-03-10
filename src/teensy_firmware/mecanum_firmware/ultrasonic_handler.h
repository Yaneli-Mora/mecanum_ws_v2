#pragma once
#include <Arduino.h>

class UltrasonicHandler {
public:
  void     init();
  void     update();
  uint16_t getDistanceMM(int i) const { return distances_mm_[i]; }

private:
  static const int  NUM = 4;
  const int TRIG[NUM]   = {20, 22, 24, 26};
  const int ECHO[NUM]   = {21, 23, 25, 27};
  uint16_t  distances_mm_[NUM] = {9999,9999,9999,9999};
};
