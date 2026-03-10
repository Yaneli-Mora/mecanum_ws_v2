#include "tof_handler.h"

bool ToFHandler::init() {
  Wire.begin();
  for (int i = 0; i < NUM; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  delay(10);
  for (int i = 0; i < NUM; i++) {
    digitalWrite(XSHUT[i], HIGH); delay(10);
    if (!sensors_[i].init()) return false;
    sensors_[i].setAddress(ADDR[i]);
    sensors_[i].startContinuous(10);
  }
  return true;
}

void ToFHandler::update() {
  for (int i = 0; i < NUM; i++)
    distances_mm_[i] = sensors_[i].read(false);
}
