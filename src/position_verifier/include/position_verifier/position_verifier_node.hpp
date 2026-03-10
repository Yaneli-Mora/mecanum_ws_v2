#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include <map>
#include <cmath>

namespace position_verifier {

struct StationProfile {
  std::string name;
  double x = 0, y = 0, yaw = 0;
  double tof_expected_m   = 0.0;
  std::string zone_color  = "";
  int    tof_sensor       = -1;  // -1=none, 0=L+R (keypad), 2=front (crank)
  double pose_tol_m       = 0.05;
  double tof_tol_m        = 0.010;
  double tof_square_tol_m = 0.008;
};

class PositionVerifierNode : public rclcpp::Node {
public:
  PositionVerifierNode();
  bool verifyAtStation(const std::string & name);

private:
  using NavToPose  = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavToPose>;

  void loadStationProfiles();
  bool checkEKF(const StationProfile & p);
  bool checkToF(const StationProfile & p);
  bool checkColor(const StationProfile & p);
  void nudgeCorrection(const StationProfile & p);

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             verified_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tof_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        color_sub_;
  rclcpp_action::Client<NavToPose>::SharedPtr                   nav_client_;

  nav_msgs::msg::Odometry          latest_odom_;
  std_msgs::msg::Float32MultiArray latest_tof_;
  std::string                      latest_color_;

  std::map<std::string, StationProfile> stations_;
  int max_retries_ = 2;
};

} // namespace position_verifier
