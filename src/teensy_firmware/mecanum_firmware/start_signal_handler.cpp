#include "start_signal_handler.h"

StartSignalHandler::StartSignalHandler(int pin, int threshold)
  : pin_(pin), threshold_(threshold) {}

void StartSignalHandler::init() {
  pinMode(pin_, INPUT);
  for (int i = 0; i < SAMPLES; i++) { buf_[i] = analogRead(pin_); delay(5); }
}

int StartSignalHandler::smoothed() {
  buf_[idx_] = analogRead(pin_);
  idx_ = (idx_ + 1) % SAMPLES;
  int s = 0; for (int i = 0; i < SAMPLES; i++) s += buf_[i];
  return s / SAMPLES;
}

bool StartSignalHandler::checkForStart() {
  if (started_) return false;
  if (smoothed() >= threshold_) { if (++sustained_ >= REQUIRED) { started_ = true; return true; } }
  else sustained_ = 0;
  return false;
}
