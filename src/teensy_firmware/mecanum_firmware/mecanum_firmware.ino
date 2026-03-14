// mecanum_firmware.ino — Teensy 4.1 (board 1)
// Handles: encoders, motors (TB6612FNG), ToF x3 (VL53L1X),
//          optical flow (PMW3901), start signal, Plank A servo, Plank B servo
//
// Color sensors (TCS34725) and ultrasonics (HC-SR04) are on Teensy 2.
//
// Text commands received from Pi via /teensy_command:
//   EXTEND_PLANK_A      — Plank A to 90°  (pin 31)
//   RETRACT_PLANK_A     — Plank A to 0°   (pin 31)
//   EXTEND_PLANK_B_90   — Plank B to 90°  (antennas 1, 4)
//   EXTEND_PLANK_B_120  — Plank B to 120° (IR transmit)
//   EXTEND_PLANK_B_150  — Plank B to 150° (antenna 2, antenna 3 — shorter antennas)
//   RETRACT_PLANK_B     — Plank B to 0°

#include <Servo.h>
#include "serial_protocol.h"
#include "encoder_handler.h"
#include "motor_handler.h"
#include "tof_handler.h"
#include "flow_handler.h"
#include "start_signal_handler.h"

EncoderHandler     enc;
MotorHandler       mot;
ToFHandler         tof;
FlowHandler        flow;
StartSignalHandler sig(10000);  // ⚠️ tune threshold on field — ADC value when light hits photodiode

// ── Plank A — servo on pin 31 ────────────────────────────────────────────
Servo plankA;
const int PLANK_A_PIN       = 31;
const int PLANK_A_RETRACTED = 0;
const int PLANK_A_EXTENDED  = 90;

// ── Plank B — LED reading ────────────────────────────────────────────────
Servo plankB;
const int PLANK_B_PIN       = 32;
const int PLANK_B_RETRACTED = 0;
const int PLANK_B_POS_90    = 90;
const int PLANK_B_POS_120   = 120;
const int PLANK_B_POS_150   = 150;

// ── Command buffer ───────────────────────────────────────────────────────
String cmd_buffer = "";

bool          mission_active = false;
unsigned long last_rx_ms     = 0;
const unsigned long WD_MS    = 500;   // watchdog: stop motors if no cmd for 500ms
const int     LOOP_MS        = 10;    // 100Hz loop

void setup() {
  Serial.begin(115200);
  enc.init();
  mot.init();
  tof.init();
  sig.init();

  plankA.attach(PLANK_A_PIN);
  plankA.write(PLANK_A_RETRACTED);

  plankB.attach(PLANK_B_PIN);
  plankB.write(PLANK_B_RETRACTED);

  if (!flow.init()) {
    // Flow sensor not found — check SPI wiring: MOSI=11, MISO=12, SCK=13, CS=10
  }
}

void loop() {
  unsigned long t0 = millis();

  // Activate on start signal
  if (!mission_active && sig.checkForStart()) {
    mission_active = true;
    mot.unlock();
  }

  // Watchdog — stop motors if Pi goes quiet
  if (mission_active && (millis() - last_rx_ms > WD_MS)) {
    mot.brakeAll();
  }

  enc.update();
  tof.update();
  flow.update();

  sendPacket();
  receiveCommands();

  long elapsed = millis() - t0;
  if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}

void sendPacket() {
  TeensyToPiPacket p;
  for (int i = 0; i < 4; i++)
    p.enc_ticks[i] = (int16_t)enc.getDeltaTicks(i);
  p.tof_mm[0]      = tof.getDistanceMM(0);   // rear-left  (keypad squareness L)
  p.tof_mm[1]      = tof.getDistanceMM(1);   // rear-right (keypad squareness R)
  p.tof_mm[2]      = tof.getDistanceMM(2);   // left-side  (antenna_2 crank approach)
  p.flow_vx        = flow.getDeltaX();
  p.flow_vy        = flow.getDeltaY();
  p.start_detected = sig.hasStarted() ? 1 : 0;
  p.checksum       = computeChecksum((uint8_t*)&p, TEENSY_PKT_SIZE - 1);
  Serial.write((uint8_t*)&p, TEENSY_PKT_SIZE);
}

void receiveCommands() {
  // Binary motor commands — start byte 0xFE
  if (Serial.available() >= (int)PI_PKT_SIZE && Serial.peek() == 0xFE) {
    PiToTeensyPacket cmd;
    Serial.readBytes((char*)&cmd, PI_PKT_SIZE);
    uint8_t cs = computeChecksum((uint8_t*)&cmd, PI_PKT_SIZE - 1);
    if (cmd.start_byte == 0xFE && cs == cmd.checksum && mission_active) {
      for (int i = 0; i < 4; i++) mot.setVelocity(i, cmd.motor_cmd[i]);
      last_rx_ms = millis();
    }
    return;
  }

  // Text commands from Pi via /teensy_command
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmd_buffer.trim();
      if (cmd_buffer.length() > 0) handleTextCommand(cmd_buffer);
      cmd_buffer = "";
    } else if (c != '\r') {
      cmd_buffer += c;
    }
  }
}

void handleTextCommand(const String & cmd) {
  if      (cmd == "EXTEND_PLANK_A")    plankA.write(PLANK_A_EXTENDED);
  else if (cmd == "RETRACT_PLANK_A")   plankA.write(PLANK_A_RETRACTED);
  else if (cmd == "EXTEND_PLANK_B_90")  plankB.write(PLANK_B_POS_90);
  else if (cmd == "EXTEND_PLANK_B_120") plankB.write(PLANK_B_POS_120);
  else if (cmd == "EXTEND_PLANK_B_150") plankB.write(PLANK_B_POS_150);
  else if (cmd == "RETRACT_PLANK_B")    plankB.write(PLANK_B_RETRACTED);
}
