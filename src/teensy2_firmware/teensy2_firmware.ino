
// teensy2_firmware.ino — Teensy 4.1 (board 2)
// Handles:
//   4x HC-SR04 ultrasonic sensors  — UltrasonicHandler
//   TCRT5000 line tracker          — LineTrackerHandler
//   TCS34725 RGB sensor            — RGBSensorHandler (on Plank B)
//   IR transmitter                 — IRTransmitterHandler (on Plank B)
//   Crank DC motor + encoder       — CrankHandler
//
// Serial protocol to Pi (115200 baud, newline terminated):
//
//   Teensy2 → Pi (spontaneous):
//     READY                         — on startup
//     LINE:WHITE / LINE:COLORED / LINE:BLACK  — on state change
//     US:<fl>,<fr>,<rl>,<rr>        — distances in mm at 50Hz
//
//   Pi → Teensy2 (commands):
//     TURN_CRANK                    — spin crank 3 full turns via encoder
//     PRESS_KEYPAD                  — fire solenoids: 7→3→7→3→8→#
//     EXTEND_PLANK_A                — Plank A servo to 90°
//     RETRACT_PLANK_A               — Plank A servo to 0°
//     READ_LED <1-4>                — read RGB, record + report color
//     TRANSMIT_IR                   — send all 4 recorded colors via IR
//
//   Teensy2 → Pi (responses):
//     DONE                          — command succeeded
//     ERROR                         — command failed
//     COLOR:<antenna>:<color>       — after READ_LED

#include <Wire.h>
#include "ultrasonic_handler.h"
#include "line_tracker_handler.h"
#include "rgb_sensor_handler.h"
#include "ir_transmitter_handler.h"
#include "crank_handler.h"
#include "keypad_handler.h"

// ── Pin assignments ───────────────────────────────────────────────────────
const int LINE_PIN    = A0;
const int IR_PIN      = 10;
const int CRANK_IN1   = 20;
const int CRANK_IN2   = 21;
const int CRANK_PWM   = 22;
const int CRANK_ENC_A = 23;
const int CRANK_ENC_B = 24;
// Solenoids (keypad) — pins 25-28
// SOL_A=25 (key 7), SOL_B=26 (key 3), SOL_C=27 (key 8), SOL_D=28 (key #)
// Plank A servo      — pin 29

// ── Handlers ─────────────────────────────────────────────────────────────
UltrasonicHandler    ultrasonics;
LineTrackerHandler   lineTracker;
RGBSensorHandler     rgbSensor;
IRTransmitterHandler irTx;
CrankHandler         crank;
KeypadHandler        keypad;
PlankAHandler        plankA;

// ── Timing ───────────────────────────────────────────────────────────────
const int US_INTERVAL_MS = 20;   // 50Hz ultrasonic publish
unsigned long us_last_ms = 0;

// ── Command buffer ────────────────────────────────────────────────────────
String cmd_buffer = "";

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  ultrasonics.init();
  lineTracker.init(LINE_PIN);
  rgbSensor.init();
  irTx.init(IR_PIN);
  crank.init(CRANK_IN1, CRANK_IN2, CRANK_PWM, CRANK_ENC_A, CRANK_ENC_B);
  keypad.init();

  Serial.println("READY");
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  // Publish ultrasonic distances at 50Hz
  if (millis() - us_last_ms >= US_INTERVAL_MS) {
    us_last_ms = millis();
    ultrasonics.update();
    Serial.print("US:");
    Serial.print(ultrasonics.getDistanceMM(0)); Serial.print(",");
    Serial.print(ultrasonics.getDistanceMM(1)); Serial.print(",");
    Serial.print(ultrasonics.getDistanceMM(2)); Serial.print(",");
    Serial.println(ultrasonics.getDistanceMM(3));
  }

  // Publish line state on change
  LineState ls = lineTracker.read();
  if (ls != lineTracker.getState()) {
    lineTracker.clearChanged();
    Serial.print("LINE:");
    Serial.println(lineTracker.stateStr());
  }

  // Read incoming commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmd_buffer.trim();
      if (cmd_buffer.length() > 0) handleCommand(cmd_buffer);
      cmd_buffer = "";
    } else if (c != '\r') {
      cmd_buffer += c;
    }
  }
}

// ── Command handler ───────────────────────────────────────────────────────
void handleCommand(const String & cmd) {

  // TURN_CRANK — spin 3 full turns via encoder feedback
  if (cmd == "TURN_CRANK") {
    bool ok = crank.turn(3);
    Serial.println(ok ? "DONE" : "ERROR");

  // PRESS_KEYPAD — fire solenoids in sequence: 7 → 3 → 7 → 3 → 8 → #
  } else if (cmd == "PRESS_KEYPAD") {
    bool ok = keypad.press();
    Serial.println(ok ? "DONE" : "ERROR");

  // EXTEND_PLANK_A — Plank A servo to 90°
  } else if (cmd == "EXTEND_PLANK_A") {
    plankA.extend();
    Serial.println("DONE");

  // RETRACT_PLANK_A — Plank A servo to 0°
  } else if (cmd == "RETRACT_PLANK_A") {
    plankA.retract();
    Serial.println("DONE");

  // READ_LED <antenna 1-4>
  } else if (cmd.startsWith("READ_LED")) {
    int ant = cmd.length() > 9 ? cmd.substring(9).toInt() : 0;
    if (ant < 1 || ant > 4) { Serial.println("ERROR"); return; }

    uint8_t color_code = rgbSensor.read();          // returns COLOR_RED/GREEN/BLUE/PURPLE/UNKNOWN
    irTx.recordColor(ant, color_code);               // stores antenna_addr | color_code

    Serial.print("COLOR:"); Serial.print(ant);
    Serial.print(":"); Serial.println(rgbSensor.colorStr(color_code));
    Serial.println("DONE");

  // TRANSMIT_IR — send all 4 recorded antenna colors
  } else if (cmd == "TRANSMIT_IR") {
    irTx.transmitAll();
    Serial.println("DONE");

  } else {
    Serial.println("ERROR");
  }
}
