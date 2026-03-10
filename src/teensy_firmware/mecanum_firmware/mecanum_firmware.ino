 // mecanum_firmware.ino — Teensy 4.1
// Handles: encoders, motors, ToF (3x), optical flow, start signal
// NOTE: Ultrasonics moved to Arduino — not handled here

#include "serial_protocol.h"
#include "encoder_handler.h"
#include "motor_handler.h"
#include "tof_handler.h"
#include "flow_handler.h"
#include "color_sensor_handler.h"
#include "start_signal_handler.h"

EncoderHandler     enc;
MotorHandler       mot;
ToFHandler         tof;
FlowHandler        flow;
ColorSensorHandler col;
StartSignalHandler sig(A0, 700);

bool mission_active       = false;
unsigned long last_rx_ms  = 0;
const unsigned long WD_MS = 500;
const int LOOP_MS         = 10;    // 100 Hz

void setup() {
  Serial.begin(115200);
  enc.init();
  mot.init();
  tof.init();
  col.init();
  sig.init();
  if (!flow.init()) {
    // Flow sensor not found — check wiring: MOSI=11, MISO=12, SCK=13, CS=10
  }
}

void loop() {
  unsigned long t0 = millis();

  if (!mission_active && sig.checkForStart()) {
    mission_active = true;
    mot.unlock();
  }

  if (mission_active && (millis() - last_rx_ms > WD_MS)) {
    mot.brakeAll();
  }

  enc.update();
  tof.update();
  flow.update();
  col.update();

  sendPacket();
  receiveCommands();

  long elapsed = millis() - t0;
  if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}

void sendPacket() {
  TeensyToPiPacket p;
  for (int i = 0; i < 4; i++) {
    p.enc_ticks[i] = (int16_t)enc.getDeltaTicks(i);
  }
  p.tof_mm[0]      = tof.getDistanceMM(0);
  p.tof_mm[1]      = tof.getDistanceMM(1);
  p.tof_mm[2]      = tof.getDistanceMM(2);
  p.flow_vx        = flow.getDeltaX();
  p.flow_vy        = flow.getDeltaY();
  p.color_left     = (uint8_t)col.getZone(0);
  p.color_right    = (uint8_t)col.getZone(1);
  p.start_detected = sig.hasStarted() ? 1 : 0;
  p.checksum       = computeChecksum((uint8_t*)&p, TEENSY_PKT_SIZE - 1);
  Serial.write((uint8_t*)&p, TEENSY_PKT_SIZE);
}

void receiveCommands() {
  if (Serial.available() >= (int)PI_PKT_SIZE) {
    PiToTeensyPacket cmd;
    Serial.readBytes((char*)&cmd, PI_PKT_SIZE);
    uint8_t cs = computeChecksum((uint8_t*)&cmd, PI_PKT_SIZE - 1);
    if (cmd.start_byte == 0xFE && cs == cmd.checksum && mission_active) {
      for (int i = 0; i < 4; i++) mot.setVelocity(i, cmd.motor_cmd[i]);
      last_rx_ms = millis();
    }
  }
}
