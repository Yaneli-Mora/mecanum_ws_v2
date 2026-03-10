#pragma once
#include <Arduino.h>
#include <Encoder.h>

class EncoderHandler {
public:
  void init();
  void update();
  int32_t getDeltaTicks(int wheel) const { return delta_ticks_[wheel]; }
  int32_t getTotalTicks(int wheel) const { return last_counts_[wheel]; }

private:
  static const int NUM_WHEELS = 4;
  // Interrupt-capable pins on Teensy 4.1 (all pins support interrupts)
  const int PINS_A[NUM_WHEELS] = {2, 4, 6,  8};
  const int PINS_B[NUM_WHEELS] = {3, 5, 7,  9};
  Encoder*  encoders_[NUM_WHEELS];
  int32_t   last_counts_[NUM_WHEELS] = {0,0,0,0};
  int32_t   delta_ticks_[NUM_WHEELS] = {0,0,0,0};
};
