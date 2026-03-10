#pragma once
#include <Arduino.h>
#include <Servo.h>

class KeypadPresser {
public:
  void init(int pin_x, int pin_y);
  bool pressKey(int row, int col);
  void home();
private:
  Servo sx_, sy_;
  // Calibrated angles per key — adjust after physical testing
  const int AX[4][3] = {{30,60,90},{30,60,90},{30,60,90},{45,75,105}};
  const int AY[4][3] = {{30,30,30},{60,60,60},{90,90,90},{120,120,120}};
};
