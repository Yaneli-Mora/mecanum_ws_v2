#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "ir_transmitter_handler.h"  // for color code defines

class RGBSensorHandler {
public:
  bool        init();
  uint8_t     read();          // returns color code (COLOR_RED/GREEN/BLUE/PURPLE/UNKNOWN)
  const char* colorStr(uint8_t code) const;

private:
  Adafruit_TCS34725 tcs_;   // initialized in init() after Wire.begin()
  bool ready_ = false;

  uint8_t classify(uint16_t r, uint16_t g, uint16_t b);
};
