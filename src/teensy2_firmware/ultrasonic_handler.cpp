#include "ultrasonic_handler.h"

void UltrasonicHandler::init() {
  for (int i = 0; i < NUM; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    digitalWrite(TRIG[i], LOW);
  }
}

uint16_t UltrasonicHandler::readMM(int idx) {
  digitalWrite(TRIG[idx], LOW);  delayMicroseconds(2);
  digitalWrite(TRIG[idx], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[idx], LOW);
  long dur = pulseIn(ECHO[idx], HIGH, 30000);  // 30ms timeout
  return dur ? (uint16_t)(dur * 0.171f) : 9999;  // mm = us * 0.171
}

void UltrasonicHandler::update() {
  for (int i = 0; i < NUM; i++)
    distances_mm_[i] = readMM(i);
}
