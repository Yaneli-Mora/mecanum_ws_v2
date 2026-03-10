#include "encoder_handler.h"

void EncoderHandler::init() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    encoders_[i]   = new Encoder(PINS_A[i], PINS_B[i]);
    last_counts_[i] = 0;
    delta_ticks_[i] = 0;
  }
}

void EncoderHandler::update() {
  for (int i = 0; i < NUM_WHEELS; i++) {
    int32_t cur      = encoders_[i]->read();
    delta_ticks_[i]  = cur - last_counts_[i];
    last_counts_[i]  = cur;
  }
}
