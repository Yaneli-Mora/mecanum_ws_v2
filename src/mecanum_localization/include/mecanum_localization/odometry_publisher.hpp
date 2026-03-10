#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace mecanum_localization {

class OdometryPublisher : public rclcpp::Node {
public:
  OdometryPublisher();

private:
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishOdomAndTF(double vx, double vy, double vtheta, double dt);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;

  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
  rclcpp::Time last_time_;
  bool first_ = true;

  // Params — set in ros2_control.xacro or node params
  double wheel_radius_  = 0.050;   // meters
  double wb_x_          = 0.150;   // half front-rear wheelbase
  double wb_y_          = 0.170;   // half left-right wheelbase
};

}  // namespace mecanum_localization
