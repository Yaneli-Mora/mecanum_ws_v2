#pragma once
#include <Wire.h>
#include <VL53L1X.h>

class ToFHandler {
public:
  bool     init();
  void     update();
  uint16_t getDistanceMM(int i) const { return distances_mm_[i]; }

private:
  static const int NUM = 2;
  const int  XSHUT[NUM]    = {28, 29};
  const uint8_t ADDR[NUM]  = {0x30, 0x31};
  VL53L1X    sensors_[NUM];
  uint16_t   distances_mm_[NUM] = {9999, 9999};
};
