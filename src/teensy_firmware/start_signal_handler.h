#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// StartSignalHandler — reads photodiode via ADS1115 on Wire2 (pins 24/25)
// ADS1115 I2C address: 0x48 (ADDR pin to GND)
// Photodiode connected to ADS1115 channel A0
// Signal detected when ADC reading drops below threshold
// (photodiode conducts more in light → voltage drops)
// ⚠️ If your circuit inverts this, change >= to <= in checkForStart()

class StartSignalHandler {
public:
  StartSignalHandler(int threshold = 10000);  // ⚠️ tune on field
  void init();
  bool checkForStart();
  bool hasStarted() const { return started_; }

private:
  int  threshold_;
  bool started_   = false;
  int  sustained_ = 0;

  static const int REQUIRED = 50;   // sustained samples needed (~1s at 50Hz)
  static const int SAMPLES  = 10;   // smoothing window
  int  buf_[SAMPLES] = {};
  int  idx_          = 0;

  Adafruit_ADS1115 ads_;
  int smoothed();
};
