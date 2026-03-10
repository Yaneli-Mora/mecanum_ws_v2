#pragma once
#include <cstddef>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mecanum_hardware/serial_manager.hpp"

namespace mecanum_hardware {

class MotorDriver {
public:
  explicit MotorDriver(double max_velocity_rad_s, std::size_t num_wheels = 4);
  ~MotorDriver();

  bool init(std::shared_ptr<SerialManager> serial);
  void close();
  void setVelocities(const std::vector<double> & velocities);
  void stopAll();

private:
  double      max_vel_;
  std::size_t num_wheels_;
  bool        initialized_ = false;
  std::shared_ptr<SerialManager> serial_;

  static double clamp(double v, double lim);
};

}  // namespace mecanum_hardware
