#include "ultrasonic_handler.h"

void UltrasonicHandler::init() {
  for (int i = 0; i < NUM; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
  }
}

void UltrasonicHandler::update() {
  for (int i = 0; i < NUM; i++) {
    digitalWrite(TRIG[i], LOW);  delayMicroseconds(2);
    digitalWrite(TRIG[i], HIGH); delayMicroseconds(10);
    digitalWrite(TRIG[i], LOW);
    long dur = pulseIn(ECHO[i], HIGH, 30000);
    distances_mm_[i] = dur ? (uint16_t)((dur * 0.343f) / 2.0f) : 9999;
  }
}
