#pragma once
#include <stdint.h>

// ── Teensy → Pi ───────────────────────────────────────────────────────────
// Ultrasonics removed — now handled by Arduino, published by attachment_node
struct __attribute__((packed)) TeensyToPiPacket {
  uint8_t  start_byte       = 0xFF;
  int16_t  enc_ticks[4];          // delta ticks since last packet per wheel
  uint16_t tof_mm[3];             // VL53L1X: rear-left(0) rear-right(1) right-side(2)
  int16_t  flow_vx;               // PMW3901 optical flow x
  int16_t  flow_vy;               // PMW3901 optical flow y
  uint8_t  color_left;            // 0=black 1=white 2=green 3=blue
  uint8_t  color_right;
  uint8_t  start_detected;        // 1 once photodiode threshold sustained
  uint8_t  checksum;
};

// ── Pi → Teensy ───────────────────────────────────────────────────────────
struct __attribute__((packed)) PiToTeensyPacket {
  uint8_t  start_byte       = 0xFE;
  int16_t  motor_cmd[4];
  uint8_t  checksum;
};

// ── Color zone enum ───────────────────────────────────────────────────────
enum ColorZone : uint8_t {
  COLOR_BLACK = 0,
  COLOR_WHITE = 1,
  COLOR_GREEN = 2,
  COLOR_BLUE  = 3
};

static const size_t TEENSY_PKT_SIZE = sizeof(TeensyToPiPacket);
static const size_t PI_PKT_SIZE     = sizeof(PiToTeensyPacket);

inline uint8_t computeChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= data[i];
  return cs;
}
