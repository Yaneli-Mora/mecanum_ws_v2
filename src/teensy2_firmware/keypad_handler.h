#pragma once
#include <Arduino.h>

// KeypadHandler — fires 4 intermittent solenoids in sequence to enter keypad code
//
// Solenoid wiring (Teensy 2):
//   Solenoid A (key 7) → pin 25
//   Solenoid B (key 3) → pin 26
//   Solenoid C (key 8) → pin 27
//   Solenoid D (key #) → pin 28
//
// Sequence: 7 → 3 → 7 → 3 → 8 → #
//   i.e.:   A → B → A → B → C → D
//
// Each solenoid fires for FIRE_MS then releases for GAP_MS before next fires

class KeypadHandler {
public:
  void init();
  bool press();   // blocking — fires full sequence, returns true when done

private:
  static const int SOL_A = 25;   // key 7
  static const int SOL_B = 26;   // key 3
  static const int SOL_C = 27;   // key 8
  static const int SOL_D = 28;   // key #

  static const int FIRE_MS = 80;   // ⚠️ tune on field — solenoid strike duration
  static const int GAP_MS  = 200;  // ⚠️ tune on field — release gap between keys

  void firePin(int pin);
};
