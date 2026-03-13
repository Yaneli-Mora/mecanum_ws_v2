#include "keypad_handler.h"

void KeypadHandler::init() {
  pinMode(SOL_A, OUTPUT); digitalWrite(SOL_A, LOW);
  pinMode(SOL_B, OUTPUT); digitalWrite(SOL_B, LOW);
  pinMode(SOL_C, OUTPUT); digitalWrite(SOL_C, LOW);
  pinMode(SOL_D, OUTPUT); digitalWrite(SOL_D, LOW);
}

void KeypadHandler::firePin(int pin) {
  digitalWrite(pin, HIGH);
  delay(FIRE_MS);
  digitalWrite(pin, LOW);
  delay(GAP_MS);
}

bool KeypadHandler::press() {
  // Sequence: 7 → 3 → 7 → 3 → 8 → #
  firePin(SOL_A);   // key 7
  firePin(SOL_B);   // key 3
  firePin(SOL_A);   // key 7
  firePin(SOL_B);   // key 3
  firePin(SOL_C);   // key 8
  firePin(SOL_D);   // key #
  return true;
}
