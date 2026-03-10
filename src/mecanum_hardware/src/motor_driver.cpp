#include "mecanum_hardware/motor_driver.hpp"
#include "mecanum_hardware/serial_protocol_pi.hpp"   // Pi-side copy of serial_protocol.h
#include <algorithm>
#include <cmath>

namespace mecanum_hardware {

MotorDriver::MotorDriver(double max_vel, std::size_t num_wheels)
: max_vel_(max_vel), num_wheels_(num_wheels) {}

MotorDriver::~MotorDriver() { if (initialized_) close(); }

bool MotorDriver::init(std::shared_ptr<SerialManager> serial) {
  serial_ = serial;
  stopAll();
  initialized_ = true;
  RCLCPP_INFO(rclcpp::get_logger("MotorDriver"), "Initialized");
  return true;
}

void MotorDriver::close() {
  stopAll();
  initialized_ = false;
}

void MotorDriver::setVelocities(const std::vector<double> & velocities) {
  if (!initialized_ || velocities.size() != num_wheels_) return;

  PiToTeensyPacket pkt;
  for (std::size_t i = 0; i < num_wheels_; ++i) {
    double clamped  = clamp(velocities[i], max_vel_);
    pkt.motor_cmd[i] = static_cast<int16_t>(
      (clamped / max_vel_) * 32767.0);
  }
  pkt.checksum = computeChecksum((uint8_t*)&pkt, sizeof(pkt) - 1);
  serial_->writeBytes((uint8_t*)&pkt, sizeof(pkt));
}

void MotorDriver::stopAll() {
  setVelocities(std::vector<double>(num_wheels_, 0.0));
}

double MotorDriver::clamp(double v, double lim) {
  return std::max(-lim, std::min(v, lim));
}

}  // namespace mecanum_hardware
