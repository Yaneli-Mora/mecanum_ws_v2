#pragma once
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// Three VL53L1X sensors on Teensy I2C (pins 18/19)
// XSHUT pins used to reassign addresses at boot
// Sensor 0 = left, Sensor 1 = right, Sensor 2 = front

class ToFHandler {
public:
  bool     init();
  void     update();
  uint16_t getDistanceMM(int i) const { return distances_mm_[i]; }

private:
  static const int NUM = 3;
  const int     XSHUT[NUM] = {41, 40, 39};
  const uint8_t ADDR[NUM]  = {0x30, 0x31, 0x32};
  Adafruit_VL53L1X sensors_[NUM];
  uint16_t distances_mm_[NUM] = {9999, 9999, 9999};
};
