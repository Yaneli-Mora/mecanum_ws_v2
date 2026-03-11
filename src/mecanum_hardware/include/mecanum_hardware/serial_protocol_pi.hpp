#pragma once
#include <cstdint>
#include <cstddef>

// ── Teensy 1 → Pi binary packet ──────────────────────────────────────────────
// Ultrasonics moved to Teensy 2 — no longer in this packet
struct __attribute__((packed)) TeensyToPiPacket {
  uint8_t  start_byte;
  int16_t  enc_ticks[4];    // FL, FR, RL, RR encoder deltas
  uint16_t tof_mm[3];       // ToF sensors 0=rear-L, 1=rear-R, 2=right-side
  int16_t  flow_vx;
  int16_t  flow_vy;
  uint8_t  color_left;      // reserved
  uint8_t  color_right;     // reserved
  uint8_t  start_detected;
  uint8_t  checksum;
};

struct __attribute__((packed)) PiToTeensyPacket {
  uint8_t  start_byte = 0xFE;
  int16_t  motor_cmd[4];
  uint8_t  checksum;
};

static const uint8_t TEENSY_START_BYTE = 0xFF;
static const uint8_t PI_START_BYTE     = 0xFE;
static const size_t  TEENSY_PKT_SIZE   = sizeof(TeensyToPiPacket);
static const size_t  PI_PKT_SIZE       = sizeof(PiToTeensyPacket);

inline uint8_t computeChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= data[i];
  return cs;
}
