#include "rgb_sensor_handler.h"

bool RGBSensorHandler::init() {
  ready_ = tcs_.begin();
  return ready_;
}

uint8_t RGBSensorHandler::read() {
  if (!ready_) return COLOR_UNKNOWN;
  uint16_t r, g, b, c;
  tcs_.getRawData(&r, &g, &b, &c);
  return classify(r, g, b);
}

uint8_t RGBSensorHandler::classify(uint16_t r, uint16_t g, uint16_t b) {
  float total = r + g + b;
  if (total < 100) return COLOR_UNKNOWN;  // too dark / no light

  float rf = r / total;
  float gf = g / total;
  float bf = b / total;

  // Purple = high red + high blue, low green
  if (rf > 0.35f && bf > 0.35f && gf < 0.25f) return COLOR_PURPLE;

  // Red = dominant red
  if (rf > 0.55f)                              return COLOR_RED;

  // Green = dominant green
  if (gf > 0.50f)                              return COLOR_GREEN;

  // Blue = dominant blue
  if (bf > 0.50f)                              return COLOR_BLUE;

  return COLOR_UNKNOWN;
}

const char* RGBSensorHandler::colorStr(uint8_t code) const {
  switch (code) {
    case COLOR_RED:    return "RED";
    case COLOR_GREEN:  return "GREEN";
    case COLOR_BLUE:   return "BLUE";
    case COLOR_PURPLE: return "PURPLE";
    default:           return "UNKNOWN";
  }
}
