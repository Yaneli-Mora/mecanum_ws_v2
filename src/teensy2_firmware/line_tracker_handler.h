#pragma once
#include <Arduino.h>

enum LineState { LINE_WHITE, LINE_COLORED, LINE_BLACK };

class LineTrackerHandler {
public:
  void      init(int pin);
  LineState read();
  LineState getState()   const { return state_; }
  bool      hasChanged() const { return changed_; }
  void      clearChanged()     { changed_ = false; }
  const char* stateStr() const;

  // ⚠️ Calibrate these on the actual field surface
  static const int THRESH_WHITE = 200;  // below = white
  static const int THRESH_BLACK = 800;  // above = black

private:
  int       pin_    = A0;
  LineState state_  = LINE_WHITE;
  bool      changed_= false;
};
