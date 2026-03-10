#include "color_sensor_handler.h"

bool ColorSensorHandler::init() {
  sensors_[0] = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  sensors_[1] = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  return sensors_[0].begin(0x29) && sensors_[1].begin(0x28);
}

void ColorSensorHandler::update() {
  for (int i = 0; i < 2; i++) {
    uint16_t r, g, b, c;
    sensors_[i].getRawData(&r, &g, &b, &c);
    zones_[i] = classify(r, g, b, c);
  }
}

ColorZone ColorSensorHandler::classify(float r, float g, float b, float c) {
  if (c < 50)   return COLOR_BLACK;
  float t = r + g + b;
  float rn = r/t, gn = g/t, bn = b/t;
  if (c > 800 && rn > 0.30f && gn > 0.30f) return COLOR_WHITE;
  if (gn > 0.45f && gn > rn && gn > bn)    return COLOR_GREEN;
  if (bn > 0.40f && bn > rn && bn > gn)    return COLOR_BLUE;
  return COLOR_BLACK;
}
