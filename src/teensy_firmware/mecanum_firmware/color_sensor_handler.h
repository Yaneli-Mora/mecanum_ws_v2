#pragma once
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "serial_protocol.h"

class ColorSensorHandler {
public:
  bool      init();
  void      update();
  ColorZone getZone(int i) const { return zones_[i]; }

private:
  Adafruit_TCS34725 sensors_[2];
  ColorZone         zones_[2] = {COLOR_BLACK, COLOR_BLACK};
  ColorZone classify(float r, float g, float b, float c);
};
