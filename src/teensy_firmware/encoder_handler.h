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
  // Interrupt-capable pins on Teensy 4.1
  // ⚠️ Pins 12 and 13 conflict with SPI (PMW3901 MISO/SCK) — RR encoder moved to 36/37
  //   FL: A=0  B=1
  //   FR: A=4  B=5
  //   RL: A=8  B=9
  //   RR: A=36 B=37   ← moved from 12/13 to avoid SPI conflict
  const int PINS_A[NUM_WHEELS] = {0, 4,  8, 36};
  const int PINS_B[NUM_WHEELS] = {1, 5,  9, 37};
  Encoder*  encoders_[NUM_WHEELS];
  int32_t   last_counts_[NUM_WHEELS] = {0,0,0,0};
  int32_t   delta_ticks_[NUM_WHEELS] = {0,0,0,0};
};
