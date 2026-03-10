#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class InitialPosePublisher : public rclcpp::Node {
public:
  InitialPosePublisher() : Node("initial_pose_publisher") {
    declare_parameter("x",          0.15);
    declare_parameter("y",          0.15);
    declare_parameter("yaw",        1.5708);
    declare_parameter("cov_x",      0.01);
    declare_parameter("cov_y",      0.01);
    declare_parameter("cov_yaw",    0.05);
    declare_parameter("delay_ms",   2000);

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).transient_local());

    int delay = get_parameter("delay_ms").as_int();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(delay),
      [this]() { publish(); timer_->cancel(); });

    RCLCPP_INFO(get_logger(), "Will publish initial pose in %dms", delay);
  }

private:
  void publish() {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = get_parameter("x").as_double();
    msg.pose.pose.position.y = get_parameter("y").as_double();
    tf2::Quaternion q;
    q.setRPY(0, 0, get_parameter("yaw").as_double());
    msg.pose.pose.orientation = tf2::toMsg(q);
    msg.pose.covariance.fill(0.0);
    msg.pose.covariance[0]  = get_parameter("cov_x").as_double();
    msg.pose.covariance[7]  = get_parameter("cov_y").as_double();
    msg.pose.covariance[35] = get_parameter("cov_yaw").as_double();
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Initial pose published at (%.3f, %.3f, %.4f rad)",
                msg.pose.pose.position.x, msg.pose.pose.position.y,
                get_parameter("yaw").as_double());
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
}
