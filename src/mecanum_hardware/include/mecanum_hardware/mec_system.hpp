#pragma once
#include <cstddef>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mecanum_hardware/serial_manager.hpp"
#include "mecanum_hardware/encoder_reader.hpp"
#include "mecanum_hardware/motor_driver.hpp"
#include "mecanum_hardware/serial_protocol_pi.hpp"

namespace mecanum_hardware {

class MecanumSystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t NUM_WHEELS = 4;
  std::vector<std::string> joint_names_;
  std::vector<double>      hw_positions_;
  std::vector<double>      hw_velocities_;
  std::vector<double>      hw_commands_;

  std::shared_ptr<SerialManager> serial_;
  std::unique_ptr<EncoderReader> encoder_reader_;
  std::unique_ptr<MotorDriver>   motor_driver_;

  // Latest parsed packet — updated in read()
  TeensyToPiPacket latest_pkt_{};
  std::mutex pkt_mutex_;

  std::string serial_port_       = "/dev/teensy";
  int         ticks_per_rev_     = 4096;
  double      max_velocity_rad_s_ = 20.0;

  bool readPacket();
};

}  // namespace mecanum_hardware

PLUGINLIB_EXPORT_CLASS(
  mecanum_hardware::MecanumSystem,
  hardware_interface::SystemInterface)
