#include "line_tracker_handler.h"

void LineTrackerHandler::init(int pin) {
  pin_   = pin;
  state_ = read();
}

LineState LineTrackerHandler::read() {
  int val = analogRead(pin_);
  LineState newState;
  if (val < THRESH_WHITE) newState = LINE_WHITE;
  else if (val > THRESH_BLACK) newState = LINE_BLACK;
  else newState = LINE_COLORED;

  if (newState != state_) {
    state_   = newState;
    changed_ = true;
  }
  return state_;
}

const char* LineTrackerHandler::stateStr() const {
  switch (state_) {
    case LINE_WHITE:   return "WHITE";
    case LINE_COLORED: return "COLORED";
    case LINE_BLACK:   return "BLACK";
    default:           return "UNKNOWN";
  }
}
