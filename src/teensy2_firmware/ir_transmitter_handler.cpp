#include "ir_transmitter_handler.h"

void IRTransmitterHandler::init(int pin) {
  IrSender.begin(pin, false);  // pin, enableLEDFeedback=false
}

uint8_t IRTransmitterHandler::antennaAddr(int antenna) {
  switch (antenna) {
    case 1: return ANTENNA_1_ADDR;
    case 2: return ANTENNA_2_ADDR;
    case 3: return ANTENNA_3_ADDR;
    case 4: return ANTENNA_4_ADDR;
    default: return 0xFF;
  }
}

void IRTransmitterHandler::recordColor(int antenna, uint8_t color_code) {
  if (antenna < 1 || antenna > 4) return;
  records_[antenna - 1] = antennaAddr(antenna) | color_code;
}

void IRTransmitterHandler::transmitAll() {
  // NEC protocol: sendNEC(address, command, repeats)
  for (int i = 0; i < 4; i++) {
    IrSender.sendNEC(0x00, records_[i], 0);
    delay(200);
  }
}
