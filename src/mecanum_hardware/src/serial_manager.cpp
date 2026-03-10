#include "mecanum_hardware/serial_manager.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

bool SerialManager::open(const std::string & port, int baud_rate) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;

  struct termios tty{};
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }

  setBaudRate(baud_rate);
  tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_lflag  = 0;
  tty.c_oflag  = 0;
  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 1;   // 100ms read timeout

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fd_ = -1; return false; }
  return true;
}

void SerialManager::close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

int SerialManager::readBytes(uint8_t * buf, size_t n) {
  size_t total = 0;
  while (total < n) {
    int r = ::read(fd_, buf + total, n - total);
    if (r <= 0) return (total > 0) ? (int)total : -1;
    total += r;
  }
  return (int)total;
}

int SerialManager::writeBytes(const uint8_t * buf, size_t n) {
  return (int)::write(fd_, buf, n);
}

bool SerialManager::syncToStartByte(uint8_t start_byte, int max_attempts) {
  uint8_t b = 0;
  for (int i = 0; i < max_attempts; i++) {
    if (readBytes(&b, 1) == 1 && b == start_byte) return true;
  }
  return false;
}

bool SerialManager::setBaudRate(int baud_rate) {
  (void)baud_rate;  // B115200 hardcoded above — extend if needed
  return true;
}
