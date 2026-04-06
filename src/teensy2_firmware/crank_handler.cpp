#include "crank_handler.h"

static int _in1, _in2, _pwm;

void CrankHandler::init(int in1, int in2, int pwm) {
  _in1 = in1; _in2 = in2; _pwm = pwm;
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_pwm, OUTPUT);
  stop();
}

bool CrankHandler::turn() {
  // Spin motor forward for CRANK_DURATION_MS then stop
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, LOW);
  analogWrite(_pwm, SPEED);
  delay(CRANK_DURATION_MS);
  stop();
  return true;
}

void CrankHandler::stop() {
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, LOW);
  analogWrite(_pwm, 0);
}
