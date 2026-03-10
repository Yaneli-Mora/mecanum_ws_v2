#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

namespace mecanum_sensors {

class TeensyBridgeNode : public rclcpp::Node {
public:
  TeensyBridgeNode();
  ~TeensyBridgeNode();

private:
  void readLoop();
  std::string classifyColor(uint8_t code);

  // us_pub_ removed — ultrasonics on Arduino
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tof_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr   flow_pub_;  // NEW
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            color_left_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            color_right_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            color_zone_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              start_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  int         fd_          = -1;
  bool        start_sent_  = false;
  std::string serial_port_ = "/dev/teensy";

  bool openSerial();
  bool readPacket();
};

}  // namespace mecanum_sensors
