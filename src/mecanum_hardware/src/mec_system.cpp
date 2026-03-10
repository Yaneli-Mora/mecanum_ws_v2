#include "mecanum_hardware/mec_system.hpp"
#include <cstring>

namespace mecanum_hardware {

hardware_interface::CallbackReturn MecanumSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Jazzy: call parent on_init — suppress deprecation by using params form
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  if (info.hardware_parameters.count("serial_port"))
    serial_port_ = info.hardware_parameters.at("serial_port");
  if (info.hardware_parameters.count("ticks_per_rev"))
    ticks_per_rev_ = std::stoi(info.hardware_parameters.at("ticks_per_rev"));
  if (info.hardware_parameters.count("max_velocity_rad_s"))
    max_velocity_rad_s_ = std::stod(info.hardware_parameters.at("max_velocity_rad_s"));

  joint_names_.resize(NUM_WHEELS);
  hw_positions_.assign(NUM_WHEELS, 0.0);
  hw_velocities_.assign(NUM_WHEELS, 0.0);
  hw_commands_.assign(NUM_WHEELS, 0.0);

  for (std::size_t i = 0; i < NUM_WHEELS; ++i)
    joint_names_[i] = info.joints[i].name;

  serial_        = std::make_shared<SerialManager>();
  encoder_reader_ = std::make_unique<EncoderReader>(ticks_per_rev_, NUM_WHEELS);
  motor_driver_   = std::make_unique<MotorDriver>(max_velocity_rad_s_, NUM_WHEELS);

  RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"),
              "on_init: port=%s ticks=%d max_vel=%.1f",
              serial_port_.c_str(), ticks_per_rev_, max_velocity_rad_s_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!serial_->open(serial_port_)) {
    RCLCPP_ERROR(rclcpp::get_logger("MecanumSystem"),
                 "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  encoder_reader_->init(serial_);
  motor_driver_->init(serial_);
  RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"),
              "Serial port opened: %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  hw_positions_.assign(NUM_WHEELS, 0.0);
  hw_velocities_.assign(NUM_WHEELS, 0.0);
  hw_commands_.assign(NUM_WHEELS, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  motor_driver_->stopAll();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  motor_driver_->close();
  encoder_reader_->close();
  serial_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MecanumSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    v.emplace_back(joint_names_[i],
      hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    v.emplace_back(joint_names_[i],
      hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return v;
}

std::vector<hardware_interface::CommandInterface>
MecanumSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  for (std::size_t i = 0; i < NUM_WHEELS; ++i)
    v.emplace_back(joint_names_[i],
      hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  return v;
}

bool MecanumSystem::readPacket() {
  if (!serial_->syncToStartByte(TEENSY_START_BYTE)) return false;

  uint8_t buf[TEENSY_PKT_SIZE];
  buf[0] = TEENSY_START_BYTE;
  int r = serial_->readBytes(buf + 1, TEENSY_PKT_SIZE - 1);
  if (r != (int)(TEENSY_PKT_SIZE - 1)) return false;

  uint8_t cs = computeChecksum(buf, TEENSY_PKT_SIZE - 1);
  if (cs != buf[TEENSY_PKT_SIZE - 1]) return false;

  std::lock_guard<std::mutex> lock(pkt_mutex_);
  memcpy(&latest_pkt_, buf, TEENSY_PKT_SIZE);
  return true;
}

hardware_interface::return_type MecanumSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (readPacket()) {
    std::lock_guard<std::mutex> lock(pkt_mutex_);

    // ── FIX: copy packed int16 array to aligned local array before passing ──
    // Taking address of packed struct member causes unaligned pointer warning
    int16_t ticks[NUM_WHEELS];
    memcpy(ticks, latest_pkt_.enc_ticks, sizeof(ticks));

    encoder_reader_->update(period.seconds(), ticks);

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      hw_positions_[i]  = encoder_reader_->getPosition(i);
      hw_velocities_[i] = encoder_reader_->getVelocity(i);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  motor_driver_->setVelocities(hw_commands_);
  return hardware_interface::return_type::OK;
}

}  // namespace mecanum_hardware
