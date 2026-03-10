#include "flow_handler.h"

// PMW3901 register addresses
#define REG_PRODUCT_ID        0x00
#define REG_REVISION_ID       0x01
#define REG_MOTION            0x02
#define REG_DELTA_X_L         0x03
#define REG_DELTA_X_H         0x04
#define REG_DELTA_Y_L         0x05
#define REG_DELTA_Y_H         0x06
#define REG_SQUAL             0x07  // surface quality
#define REG_POWER_UP_RESET    0x3A
#define REG_ORIENTATION       0x5B

// Expected product ID
#define PMW3901_PRODUCT_ID    0x49

bool FlowHandler::init() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  // Power-up reset sequence (datasheet requirement)
  writeRegister(REG_POWER_UP_RESET, 0x5A);
  delay(5);

  // Read and discard motion registers to clear them
  readRegister(REG_MOTION);
  readRegister(REG_DELTA_X_L);
  readRegister(REG_DELTA_X_H);
  readRegister(REG_DELTA_Y_L);
  readRegister(REG_DELTA_Y_H);

  // Verify we're talking to the right chip
  if (!checkSignature()) {
    return false;
  }

  // Performance optimisation registers (from Pixart application note)
  writeRegister(0x7F, 0x00);
  writeRegister(0x61, 0xAD);
  writeRegister(0x7F, 0x03);
  writeRegister(0x40, 0x00);
  writeRegister(0x7F, 0x05);
  writeRegister(0x41, 0xB3);
  writeRegister(0x43, 0xF1);
  writeRegister(0x45, 0x14);
  writeRegister(0x5B, 0x32);
  writeRegister(0x5F, 0x34);
  writeRegister(0x7B, 0x08);
  writeRegister(0x7F, 0x06);
  writeRegister(0x44, 0x1B);
  writeRegister(0x40, 0xBF);
  writeRegister(0x4E, 0x3F);
  writeRegister(0x7F, 0x08);
  writeRegister(0x65, 0x20);
  writeRegister(0x6A, 0x18);
  writeRegister(0x7F, 0x09);
  writeRegister(0x4F, 0xAF);
  writeRegister(0x5F, 0x40);
  writeRegister(0x48, 0x80);
  writeRegister(0x49, 0x80);
  writeRegister(0x57, 0x77);
  writeRegister(0x60, 0x78);
  writeRegister(0x61, 0x78);
  writeRegister(0x62, 0x08);
  writeRegister(0x63, 0x50);
  writeRegister(0x7F, 0x0A);
  writeRegister(0x45, 0x60);
  writeRegister(0x7F, 0x00);
  writeRegister(0x4D, 0x11);
  writeRegister(0x55, 0x80);
  writeRegister(0x74, 0x21);
  writeRegister(0x75, 0x1F);
  writeRegister(0x4A, 0x78);
  writeRegister(0x4B, 0x78);
  writeRegister(0x44, 0x08);
  writeRegister(0x45, 0x50);
  writeRegister(0x64, 0xFF);
  writeRegister(0x65, 0x1F);
  writeRegister(0x7F, 0x14);
  writeRegister(0x65, 0x67);
  writeRegister(0x66, 0x08);
  writeRegister(0x63, 0x70);
  writeRegister(0x7F, 0x15);
  writeRegister(0x48, 0x48);
  writeRegister(0x7F, 0x07);
  writeRegister(0x41, 0x0D);
  writeRegister(0x43, 0x14);
  writeRegister(0x4B, 0x0E);
  writeRegister(0x45, 0x0F);
  writeRegister(0x44, 0x42);
  writeRegister(0x4C, 0x80);
  writeRegister(0x7F, 0x10);
  writeRegister(0x5B, 0x02);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x41);
  writeRegister(0x70, 0x00);
  delay(10);
  writeRegister(0x32, 0x44);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x40);
  writeRegister(0x7F, 0x06);
  writeRegister(0x62, 0xF0);
  writeRegister(0x63, 0x00);
  writeRegister(0x7F, 0x0D);
  writeRegister(0x48, 0xC0);
  writeRegister(0x6F, 0xD5);
  writeRegister(0x7F, 0x00);
  writeRegister(0x5B, 0xA0);
  writeRegister(0x4E, 0xA8);
  writeRegister(0x5A, 0x50);
  writeRegister(0x40, 0x80);

  return true;
}

void FlowHandler::update() {
  uint8_t motion = readRegister(REG_MOTION);
  motion_ = (motion & 0x80) != 0;  // bit 7 = motion detected

  if (motion_) {
    uint8_t xl = readRegister(REG_DELTA_X_L);
    uint8_t xh = readRegister(REG_DELTA_X_H);
    uint8_t yl = readRegister(REG_DELTA_Y_L);
    uint8_t yh = readRegister(REG_DELTA_Y_H);

    delta_x_ = (int16_t)((xh << 8) | xl);
    delta_y_ = (int16_t)((yh << 8) | yl);
  } else {
    delta_x_ = 0;
    delta_y_ = 0;
  }
}

bool FlowHandler::checkSignature() {
  uint8_t pid = readRegister(REG_PRODUCT_ID);
  return (pid == PMW3901_PRODUCT_ID);
}

void FlowHandler::writeRegister(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.transfer(reg | 0x80);  // set MSB for write
  SPI.transfer(val);
  delayMicroseconds(50);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(200);
}

uint8_t FlowHandler::readRegister(uint8_t reg) {
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(50);
  SPI.transfer(reg & 0x7F);  // clear MSB for read
  delayMicroseconds(100);
  uint8_t val = SPI.transfer(0);
  delayMicroseconds(50);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(200);
  return val;
}
