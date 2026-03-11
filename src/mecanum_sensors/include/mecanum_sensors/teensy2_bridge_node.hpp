#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <string>
#include <mutex>
#include <atomic>

namespace mecanum_sensors {

class Teensy2BridgeNode : public rclcpp::Node {
public:
  Teensy2BridgeNode();
  ~Teensy2BridgeNode();

private:
  void pollSerial();
  void handleIncomingLine(const std::string & line);
  void onCommand(const std_msgs::msg::String::SharedPtr msg);
  bool openSerial(const std::string & port);
  void closeSerial();
  std::string sendAndWait(const std::string & cmd, int timeout_ms = 5000);

  // ── Publishers ────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr us_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            line_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            task_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            antenna_colors_pub_;

  // ── Subscriptions ─────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         cmd_sub_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  int              fd_         = -1;
  std::string      port_;
  std::string      serial_buf_;
  std::mutex       serial_mtx_;
  std::atomic_bool busy_       = false;

  // Recorded antenna colors (index 0-3 = antenna 1-4)
  std::string antenna_colors_[4] = {"UNKNOWN","UNKNOWN","UNKNOWN","UNKNOWN"};
};

} // namespace mecanum_sensors
