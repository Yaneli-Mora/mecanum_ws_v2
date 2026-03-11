#include "crank_handler.h"

static CrankHandler* _instance = nullptr;
static void crankISR() { if (_instance) _instance->encoderISR(); }

void CrankHandler::init(int in1, int in2, int pwm, int enc_a, int enc_b) {
  in1_ = in1; in2_ = in2; pwm_ = pwm;
  pinMode(in1_, OUTPUT); pinMode(in2_, OUTPUT); pinMode(pwm_, OUTPUT);
  stop();
  pinMode(enc_a, INPUT_PULLUP);
  pinMode(enc_b, INPUT_PULLUP);
  _instance = this;
  attachInterrupt(digitalPinToInterrupt(enc_a), crankISR, RISING);
}

void CrankHandler::encoderISR() {
  ticks_++;
}

bool CrankHandler::turn(int turns) {
  long target = (long)CPR * turns;
  ticks_ = 0;

  digitalWrite(in1_, HIGH);
  digitalWrite(in2_, LOW);
  analogWrite(pwm_, SPEED);

  unsigned long start = millis();
  while (ticks_ < target) {
    if (millis() - start > WATCHDOG_MS) {
      stop();
      return false;  // timeout — motor stalled or encoder fault
    }
    delayMicroseconds(100);
  }
  stop();
  return true;
}

void CrankHandler::stop() {
  digitalWrite(in1_, LOW);
  digitalWrite(in2_, LOW);
  analogWrite(pwm_, 0);
}
