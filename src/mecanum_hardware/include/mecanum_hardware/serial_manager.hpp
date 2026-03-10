#pragma once
#include <string>
#include <cstdint>
#include <cstddef>

// Shared serial port manager used by EncoderReader and MotorDriver
// Handles open/close/read/write to the Teensy USB serial port

class SerialManager {
public:
  bool open(const std::string & port, int baud_rate = 115200);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // Returns number of bytes read, -1 on error
  int readBytes(uint8_t * buf, size_t n);

  // Returns number of bytes written, -1 on error
  int writeBytes(const uint8_t * buf, size_t n);

  // Drain input buffer until start byte found
  bool syncToStartByte(uint8_t start_byte, int max_attempts = 256);

private:
  int fd_ = -1;
  bool setBaudRate(int baud_rate);
};
