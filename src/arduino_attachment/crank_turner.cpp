#include "crank_turner.h"
void CrankTurner::init(int pin) { servo_.attach(pin); home(); }
bool CrankTurner::turn(int deg) {
  angle_ = constrain(angle_ + deg, 0, 180);
  servo_.write(angle_); delay(300); return true;
}
void CrankTurner::home() { servo_.write(90); angle_ = 90; }
