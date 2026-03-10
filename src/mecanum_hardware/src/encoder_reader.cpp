#include "mecanum_hardware/encoder_reader.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace mecanum_hardware {

EncoderReader::EncoderReader(int ticks_per_rev, std::size_t num_wheels)
: ticks_per_rev_(ticks_per_rev), num_wheels_(num_wheels),
  accumulated_ticks_(num_wheels, 0),
  positions_(num_wheels, 0.0),
  velocities_(num_wheels, 0.0) {
  if (ticks_per_rev_ <= 0)
    throw std::invalid_argument("ticks_per_rev must be > 0");
}

EncoderReader::~EncoderReader() { if (initialized_) close(); }

bool EncoderReader::init(std::shared_ptr<SerialManager> serial) {
  (void)serial;  // serial managed by mec_system — shared
  resetCounters();
  initialized_ = true;
  RCLCPP_INFO(rclcpp::get_logger("EncoderReader"), "Initialized");
  return true;
}

void EncoderReader::close() {
  initialized_ = false;
}

// dt in seconds, delta_ticks array from parsed TeensyToPiPacket
void EncoderReader::update(double dt, const int16_t * delta_ticks) {
  if (!initialized_ || dt <= 0.0) return;
  const double ticks_to_rad = (2.0 * M_PI) / static_cast<double>(ticks_per_rev_);
  for (std::size_t i = 0; i < num_wheels_; ++i) {
    accumulated_ticks_[i] += delta_ticks[i];
    positions_[i]   = static_cast<double>(accumulated_ticks_[i]) * ticks_to_rad;
    velocities_[i]  = static_cast<double>(delta_ticks[i])        * ticks_to_rad / dt;
  }
}

double EncoderReader::getPosition(std::size_t i) const { return positions_.at(i); }
double EncoderReader::getVelocity(std::size_t i) const { return velocities_.at(i); }

void EncoderReader::resetCounters() {
  std::fill(accumulated_ticks_.begin(), accumulated_ticks_.end(), 0);
  std::fill(positions_.begin(),         positions_.end(),         0.0);
  std::fill(velocities_.begin(),        velocities_.end(),        0.0);
}

}  // namespace mecanum_hardware
