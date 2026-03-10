#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace duck_operations {

enum class PlowState {
  IDLE,
  PUSHING,          // duck push — straight forward
  BUTTON_PRESSING,  // forward/reverse cycles against button
  RETREATING,
  DONE
};

class PlowControllerNode : public rclcpp::Node {
public:
  PlowControllerNode();

private:
  void controlLoop();
  void onCommand(const std_msgs::msg::String::SharedPtr msg);
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
  void buttonPressLoop();
  void stopRobot();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  PlowState state_  = PlowState::IDLE;

  // Button press state
  int    press_cycles_remaining_ = 0;
  bool   press_fwd_              = true;
  double dist_traveled_m_        = 0.0;
  double wheel_radius_           = 0.050;

  // Wheel positions for encoder-based distance tracking
  double last_wheel_pos_[4]    = {0,0,0,0};
  bool   wheel_pos_init_       = false;

  // Constants
  static constexpr double LOOP_DT          = 0.05;   // 50ms timer
  static constexpr double PUSH_SPEED       = 0.10;   // m/s duck push
  static constexpr double PRESS_SPEED      = 0.12;   // m/s button press
  static constexpr double REVERSE_SPEED    = 0.10;   // m/s retreat
  static constexpr double PRESS_DIST_M     = 0.08;   // forward stroke
  static constexpr double REVERSE_DIST_M   = 0.06;   // retreat stroke
  static constexpr int    PRESS_CYCLES     = 3;
};

} // namespace duck_operations
