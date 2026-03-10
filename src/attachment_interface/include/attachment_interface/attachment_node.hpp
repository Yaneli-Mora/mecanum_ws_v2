#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <string>
#include <mutex>
#include <atomic>

namespace attachment_interface {

class AttachmentNode : public rclcpp::Node {
public:
  AttachmentNode();
  ~AttachmentNode();

private:
  void onCommand(const std_msgs::msg::String::SharedPtr msg);
  void pollSerial();
  void dispatchLine(const std::string & line);
  std::string readLineBlocking(int timeout_ms);
  bool openSerial(const std::string & port);
  void closeSerial();

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           line_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr us_pub_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

  rclcpp::TimerBase::SharedPtr serial_timer_;

  int              fd_   = -1;
  std::mutex       serial_mtx_;
  std::string      port_;
  std::string      buf_;           // serial line buffer
  std::atomic_bool busy_ = false;  // true while waiting for command response
};

} // namespace attachment_interface
