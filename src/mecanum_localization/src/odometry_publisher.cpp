#include "mecanum_localization/odometry_publisher.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mecanum_localization {

OdometryPublisher::OdometryPublisher() : Node("odometry_publisher") {
  declare_parameter("wheel_radius",   0.050);
  declare_parameter("wheel_base_x",   0.150);
  declare_parameter("wheel_base_y",   0.170);

  wheel_radius_ = get_parameter("wheel_radius").as_double();
  wb_x_         = get_parameter("wheel_base_x").as_double();
  wb_y_         = get_parameter("wheel_base_y").as_double();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&OdometryPublisher::onJointStates, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Odometry publisher started (r=%.3fm bx=%.3f by=%.3f)",
              wheel_radius_, wb_x_, wb_y_);
}

void OdometryPublisher::onJointStates(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->velocity.size() < 4) return;

  rclcpp::Time now = msg->header.stamp;
  if (first_) { last_time_ = now; first_ = false; return; }

  double dt = (now - last_time_).seconds();
  last_time_ = now;
  if (dt <= 0.0 || dt > 0.5) return;

  // Joint order assumed: front_left, front_right, rear_left, rear_right
  // Adjust indices to match your joint_state_broadcaster output
  double v_fl = msg->velocity[0] * wheel_radius_;
  double v_fr = msg->velocity[1] * wheel_radius_;
  double v_rl = msg->velocity[2] * wheel_radius_;
  double v_rr = msg->velocity[3] * wheel_radius_;

  // Mecanum forward kinematics
  double vx     = ( v_fl + v_fr + v_rl + v_rr) / 4.0;
  double vy     = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
  double vtheta = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (wb_x_ + wb_y_));

  publishOdomAndTF(vx, vy, vtheta, dt);
}

void OdometryPublisher::publishOdomAndTF(
  double vx, double vy, double vtheta, double dt)
{
  // Integrate pose
  x_     += (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
  y_     += (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;
  theta_ += vtheta * dt;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);

  rclcpp::Time stamp = now();

  // ── Publish /odom ───────────────────────────────────────────────────────
  nav_msgs::msg::Odometry odom;
  odom.header.stamp            = stamp;
  odom.header.frame_id         = "odom";
  odom.child_frame_id          = "base_link";
  odom.pose.pose.position.x    = x_;
  odom.pose.pose.position.y    = y_;
  odom.pose.pose.orientation   = tf2::toMsg(q);
  odom.twist.twist.linear.x    = vx;
  odom.twist.twist.linear.y    = vy;
  odom.twist.twist.angular.z   = vtheta;

  // Covariance (diagonal) — tune after testing
  odom.pose.covariance[0]  = 0.01;   // x
  odom.pose.covariance[7]  = 0.01;   // y
  odom.pose.covariance[35] = 0.05;   // yaw
  odom.twist.covariance[0] = 0.01;
  odom.twist.covariance[7] = 0.01;
  odom.twist.covariance[35]= 0.05;

  odom_pub_->publish(odom);

  // ── Broadcast odom → base_link TF ──────────────────────────────────────
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp            = stamp;
  tf.header.frame_id         = "odom";
  tf.child_frame_id          = "base_link";
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation      = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(tf);
}

}  // namespace mecanum_localization

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_localization::OdometryPublisher>());
  rclcpp::shutdown();
}
