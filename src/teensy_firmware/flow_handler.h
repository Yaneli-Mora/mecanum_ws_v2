#pragma once
#include <Arduino.h>
#include <SPI.h>

// CJMCU-3901 optical flow sensor (PMW3901 chip)
// Uses SPI interface
//
// Wiring to Teensy 4.1:
//   MOSI → pin 11 (SPI MOSI)
//   MISO → pin 12 (SPI MISO)
//   SCLK → pin 13 (SPI SCK)
//   CS   → pin 10 (chip select — update below if different)
//   VCC  → 3.3V
//   GND  → GND
//
// Outputs deltaX, deltaY — accumulated pixel displacement since last read
// Positive X = motion to the right
// Positive Y = motion forward (toward sensor)

class FlowHandler {
public:
  bool init();
  void update();

  int16_t getDeltaX() const { return delta_x_; }
  int16_t getDeltaY() const { return delta_y_; }
  bool    isMotionDetected() const { return motion_; }

private:
  // ── Update CS pin to match your wiring ───────────────────────────────
  static const int CS_PIN = 10;
  // ─────────────────────────────────────────────────────────────────────

  void     writeRegister(uint8_t reg, uint8_t val);
  uint8_t  readRegister(uint8_t reg);
  bool     checkSignature();

  int16_t delta_x_ = 0;
  int16_t delta_y_ = 0;
  bool    motion_  = false;
};
