// attachment_firmware.ino — Arduino Uno/Nano
// Handles:
//   - 4x HC-SR04 ultrasonic sensors (continuous streaming at ~20Hz)
//   - TCRT5000 line tracker (spontaneous change reports)
//   - TURN_CRANK, PRESS_KEY commands from Pi
//
// Serial protocol (115200 baud):
//
//   Continuous output (every 50ms):
//     US:<f0>,<f1>,<r0>,<r1>\n   distances in mm, e.g. US:245,312,180,420
//
//   Spontaneous output (on state change):
//     LINE:WHITE\n
//     LINE:COLORED\n
//     LINE:BLACK\n
//
//   Commands from Pi → response:
//     TURN_CRANK <deg>  → DONE or ERROR
//     PRESS_KEY <r> <c> → DONE or ERROR
//     HOME              → DONE
//     LINE_STATUS       → WHITE, COLORED, or BLACK
//
// HC-SR04 wiring:
//   Front-left:  TRIG=2  ECHO=3
//   Front-right: TRIG=4  ECHO=5
//   Rear-left:   TRIG=6  ECHO=7
//   Rear-right:  TRIG=8  ECHO=9
//
// TCRT5000: Digital OUT → pin 10

#include "crank_turner.h"
#include "keypad_presser.h"

CrankTurner   crank;
KeypadPresser keypad;

// ── HC-SR04 pins ──────────────────────────────────────────────────────────
const int TRIG[4] = {2, 4, 6, 8};
const int ECHO[4] = {3, 5, 7, 9};

// ── TCRT5000 ──────────────────────────────────────────────────────────────
const int TCRT_PIN        = 10;
const int THRESHOLD_BLACK = 800;
const int THRESHOLD_WHITE = 200;

// ── State ─────────────────────────────────────────────────────────────────
String lastLineState   = "";
unsigned long lastUs   = 0;
const unsigned long US_INTERVAL = 50;  // stream ultrasonics every 50ms

// ── Ultrasonic read ───────────────────────────────────────────────────────
uint16_t readUltrasonic(int idx) {
  digitalWrite(TRIG[idx], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG[idx], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG[idx], LOW);
  long duration = pulseIn(ECHO[idx], HIGH, 30000);  // 30ms timeout ~5m
  if (duration == 0) return 9999;
  return (uint16_t)(duration * 0.1715f);  // mm = duration(us) * 0.1715
}

// ── Line state read ───────────────────────────────────────────────────────
String readLineState() {
  int val = analogRead(TCRT_PIN);
  if (val > THRESHOLD_BLACK) return "BLACK";
  if (val < THRESHOLD_WHITE) return "WHITE";
  return "COLORED";
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
  }
  pinMode(TCRT_PIN, INPUT);

  crank.init(5);
  keypad.init(6, 9);

  lastLineState = readLineState();
  Serial.println("READY");
}

void loop() {
  unsigned long now = millis();

  // ── Stream ultrasonics every 50ms ─────────────────────────────────────
  if (now - lastUs >= US_INTERVAL) {
    lastUs = now;
    uint16_t d[4];
    for (int i = 0; i < 4; i++) d[i] = readUltrasonic(i);
    Serial.print("US:");
    Serial.print(d[0]); Serial.print(",");
    Serial.print(d[1]); Serial.print(",");
    Serial.print(d[2]); Serial.print(",");
    Serial.println(d[3]);
  }

  // ── Report line state changes ──────────────────────────────────────────
  String state = readLineState();
  if (state != lastLineState) {
    lastLineState = state;
    Serial.println("LINE:" + state);
  }

  // ── Handle incoming commands ───────────────────────────────────────────
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("TURN_CRANK")) {
    int deg = cmd.length() > 11 ? cmd.substring(11).toInt() : 90;
    Serial.println(crank.turn(deg) ? "DONE" : "ERROR");

  } else if (cmd.startsWith("PRESS_KEY")) {
    int row = cmd.substring(10, 11).toInt();
    int col = cmd.substring(12, 13).toInt();
    Serial.println(keypad.pressKey(row, col) ? "DONE" : "ERROR");

  } else if (cmd == "HOME") {
    crank.home();
    keypad.home();
    Serial.println("DONE");

  } else if (cmd == "LINE_STATUS") {
    Serial.println(readLineState());
  }
}
