#include "start_signal_handler.h"

StartSignalHandler::StartSignalHandler(int threshold)
  : threshold_(threshold) {}

void StartSignalHandler::init() {
  // ADS1115 on Wire2 (Teensy 1 pins 24=SDA, 25=SCL)
  Wire2.begin();
  ads_.setDataRate(RATE_ADS1115_860SPS);  // fastest rate
  if (!ads_.begin(0x48, &Wire2)) {
    // ADS1115 not found — fail gracefully, never trigger start
    return;
  }
  ads_.setGain(GAIN_ONE);  // ±4.096V range, 0.125mV/bit

  // Prime the smoothing buffer
  for (int i = 0; i < SAMPLES; i++) {
    buf_[i] = (int)ads_.readADC_SingleEnded(0);
    delay(5);
  }
}

int StartSignalHandler::smoothed() {
  buf_[idx_] = (int)ads_.readADC_SingleEnded(0);  // channel A0
  idx_ = (idx_ + 1) % SAMPLES;
  int s = 0;
  for (int i = 0; i < SAMPLES; i++) s += buf_[i];
  return s / SAMPLES;
}

bool StartSignalHandler::checkForStart() {
  if (started_) return false;
  // Signal detected when reading drops below threshold
  // (light hits photodiode → resistance drops → voltage drops)
  // ⚠️ If signal goes HIGH instead, change < to >
  if (smoothed() < threshold_) {
    if (++sustained_ >= REQUIRED) {
      started_ = true;
      return true;
    }
  } else {
    sustained_ = 0;
  }
  return false;
}
