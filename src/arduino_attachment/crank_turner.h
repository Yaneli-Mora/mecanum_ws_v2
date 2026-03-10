#pragma once
#include <Arduino.h>
#include <Servo.h>

class CrankTurner {
public:
  void init(int pin);
  bool turn(int degrees);
  void home();
private:
  Servo servo_;
  int   angle_ = 90;
};
