#pragma once
#include <Arduino.h>
#define IR_SEND_PIN 10          // must be defined before include in IRremote v4
#define NO_LED_FEEDBACK_CODE    // disable feedback LED to save flash
#include <IRremote.hpp>

// ── Antenna address codes ─────────────────────────────────────────────────
#define ANTENNA_1_ADDR  0x00
#define ANTENNA_2_ADDR  0x30
#define ANTENNA_3_ADDR  0x50
#define ANTENNA_4_ADDR  0x60

// ── LED color codes ───────────────────────────────────────────────────────
#define COLOR_RED     0x09
#define COLOR_GREEN   0x0A
#define COLOR_BLUE    0x0C
#define COLOR_PURPLE  0x0F
#define COLOR_UNKNOWN 0xFF

class IRTransmitterHandler {
public:
  void    init(int pin);
  void    recordColor(int antenna, uint8_t color_code);  // antenna 1-4
  void    transmitAll();   // send all 4 recorded antenna+color codes via NEC IR

  // Combined code = antenna_addr | color_code
  // e.g. antenna_2 + GREEN = 0x30 | 0x0A = 0x3A
  static uint8_t antennaAddr(int antenna);

private:
  // Stored as combined byte: antenna_addr | color_code
  uint8_t records_[4] = {
    ANTENNA_1_ADDR | COLOR_UNKNOWN,
    ANTENNA_2_ADDR | COLOR_UNKNOWN,
    ANTENNA_3_ADDR | COLOR_UNKNOWN,
    ANTENNA_4_ADDR | COLOR_UNKNOWN
  };
};
