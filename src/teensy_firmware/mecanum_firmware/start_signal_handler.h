#pragma once
#include <Arduino.h>

class StartSignalHandler {
public:
  explicit StartSignalHandler(int pin, int threshold = 700);
  void init();
  bool checkForStart();
  bool hasStarted() const { return started_; }

private:
  int  pin_, threshold_;
  bool started_        = false;
  int  sustained_      = 0;
  static const int REQUIRED = 50;
  static const int SAMPLES  = 10;
  int  buf_[SAMPLES]   = {};
  int  idx_            = 0;
  int  smoothed();
};
