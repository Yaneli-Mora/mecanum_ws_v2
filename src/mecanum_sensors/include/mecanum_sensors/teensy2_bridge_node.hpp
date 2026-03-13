
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/range.hpp"
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
  void publishUltrasonics(const std::vector<float> & distances_m);

  // ── Publishers ────────────────────────────────────────────────────────
  // Array publisher (for task_manager safety checks)
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr us_pub_;

  // Individual Range publishers (for Nav2 obstacle layer)
  // Order: FL=0, FR=1, RL=2, RR=3
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us_range_pub_[4];

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr line_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr antenna_colors_pub_;

  // ── Subscriptions ─────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  int              fd_         = -1;
  std::string      port_;
  std::string      serial_buf_;
  std::mutex       serial_mtx_;
  std::atomic_bool busy_       = false;

  // Sensor frame names — match URDF
  const std::string US_FRAMES[4] = {
    "ultrasonic_fl_link",
    "ultrasonic_fr_link",
    "ultrasonic_rl_link",
    "ultrasonic_rr_link"
  };

  const std::string US_TOPICS[4] = {
    "/ultrasonic_distances/fl",
    "/ultrasonic_distances/fr",
    "/ultrasonic_distances/rl",
    "/ultrasonic_distances/rr"
  };

  // Recorded antenna colors (index 0-3 = antenna 1-4)
  std::string antenna_colors_[4] = {"UNKNOWN","UNKNOWN","UNKNOWN","UNKNOWN"};
};

} // namespace mecanum_sensors
