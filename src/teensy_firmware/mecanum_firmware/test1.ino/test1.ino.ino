// motor_test.ino — TB6612 wheel direction test
// Moves: Forward → Backward → Strafe Left → Strafe Right
// Each movement lasts 3 seconds, then brakes for 1 second before next move
//
// Upload this separately from mecanum_firmware.ino to test wiring.
// Check wheel spin directions match expected mecanum behaviour.
// Adjust SPEED (0-255) if motors are too fast/slow for bench testing.

// ── Pin assignments — must match your motor_handler.h ─────────────────
//              FL   FR   RL   RR
const int IN1[] = { 34,  31, 27, 24};
const int IN2[] = { 33,  30, 26, 23};
const int PWM[] = { 32,  28, 25, 22};
const int STBY[]= { 29,  19};          // chip A (FL/FR), chip B (RL/RR)
// ──────────────────────────────────────────────────────────────────────

const int SPEED    = 150;  // 0-255, keep low for bench testing
const int MOVE_MS  = 3000;
const int BRAKE_MS = 1000;

// Mecanum wheel velocity matrix
// Each row = [FL, FR, RL, RR] as +1 forward, -1 reverse, 0 stop
const int8_t FORWARD []  = { 1,  1,  1,  1};
const int8_t BACKWARD[]  = {-1, -1, -1, -1};
const int8_t LEFT[]      = {-1,  1,  1, -1};  // strafe left
const int8_t RIGHT[]     = { 1, -1, -1,  1};  // strafe right

void setMotor(int wheel, int8_t dir) {
  int pwm_val = (dir == 0) ? 0 : SPEED;
  if (dir > 0) {
    digitalWrite(IN1[wheel], HIGH);
    digitalWrite(IN2[wheel], LOW);
  } else if (dir < 0) {
    digitalWrite(IN1[wheel], LOW);
    digitalWrite(IN2[wheel], HIGH);
  } else {
    digitalWrite(IN1[wheel], HIGH);  // brake
    digitalWrite(IN2[wheel], HIGH);
    pwm_val = 0;
  }
  analogWrite(PWM[wheel], pwm_val);
}

void runPattern(const int8_t* pattern, const char* label) {
  Serial.print("Moving: ");
  Serial.println(label);
  for (int i = 0; i < 4; i++) setMotor(i, pattern[i]);
  delay(MOVE_MS);

  // Brake between moves
  Serial.println("Braking...");
  for (int i = 0; i < 4; i++) setMotor(i, 0);
  delay(BRAKE_MS);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Motor test starting in 2 seconds...");

  // Enable TB6612 chips
  for (int i = 0; i < 2; i++) {
    pinMode(STBY[i], OUTPUT);
    digitalWrite(STBY[i], HIGH);
  }

  // Init all motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
    setMotor(i, 0);  // start braked
  }

  delay(2000);
}

void loop() {
  runPattern(FORWARD,  "FORWARD");
  runPattern(BACKWARD, "BACKWARD");
  runPattern(LEFT,     "STRAFE LEFT");
  runPattern(RIGHT,    "STRAFE RIGHT");

  Serial.println("Test complete — repeating in 3 seconds...");
  delay(3000);
}
