#include "keypad_presser.h"
void KeypadPresser::init(int px, int py) { sx_.attach(px); sy_.attach(py); home(); }
bool KeypadPresser::pressKey(int r, int c) {
  sx_.write(AX[r][c]); sy_.write(AY[r][c]); delay(400);
  delay(200); home(); return true;
}
void KeypadPresser::home() { sx_.write(90); sy_.write(90); }
