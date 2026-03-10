#include "tof_handler.h"

bool ToFHandler::init() {
  Wire.begin();

  // Hold all sensors in reset
  for (int i = 0; i < NUM; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  delay(10);

  // Bring up one sensor at a time, assign unique address
  for (int i = 0; i < NUM; i++) {
    digitalWrite(XSHUT[i], HIGH);
    delay(10);

    if (!sensors_[i].begin(0x29, &Wire)) {
      return false;
    }
    if (!sensors_[i].startRanging()) {
      return false;
    }
    sensors_[i].VL53L1X_SetI2CAddress(ADDR[i]);
    sensors_[i].setTimingBudget(50);
  }
  return true;
}

void ToFHandler::update() {
  for (int i = 0; i < NUM; i++) {
    if (sensors_[i].dataReady()) {
      int16_t d = sensors_[i].distance();
      distances_mm_[i] = (d < 0) ? 9999 : (uint16_t)d;
      sensors_[i].clearInterrupt();
    }
  }
}
