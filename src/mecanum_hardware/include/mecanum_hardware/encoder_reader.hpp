#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mecanum_hardware/serial_manager.hpp"

namespace mecanum_hardware {

class EncoderReader {
public:
  explicit EncoderReader(int ticks_per_rev, std::size_t num_wheels = 4);
  ~EncoderReader();

  bool   init(std::shared_ptr<SerialManager> serial);
  void   close();
  void   update(double dt, const int16_t * delta_ticks);  // takes parsed ticks from packet
  double getPosition(std::size_t i) const;
  double getVelocity(std::size_t i) const;
  void   resetCounters();

private:
  int         ticks_per_rev_;
  std::size_t num_wheels_;
  bool        initialized_ = false;

  std::vector<int32_t> accumulated_ticks_;
  std::vector<double>  positions_;
  std::vector<double>  velocities_;
};

}  // namespace mecanum_hardware
