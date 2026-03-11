#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

struct WhiteLine {
  std::string name;
  char   axis;
  double pos_m;
  double start, end;
  double tol_m = 0.08;
};

class WhiteLineCorrectorNode : public rclcpp::Node {
public:
  WhiteLineCorrectorNode() : Node("white_line_corrector_node") {
    // Hardcoded from white_lines.yaml — update after field measurement
    lines_ = {
      {"vertical_center",  'x', 1.219, 0.05,  1.169, 0.08},
      {"horizontal_right", 'y', 0.609, 1.219, 2.388, 0.08},
    };

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr m) { latest_ = *m; });

    color_sub_ = create_subscription<std_msgs::msg::String>(
      "/line_state", 10,
      [this](const std_msgs::msg::String::SharedPtr m) { onColor(m->data); });

    RCLCPP_INFO(get_logger(), "White line corrector started (%zu lines)", lines_.size());
  }

private:
  void onColor(const std::string & color) {
    if (color != "WHITE") { was_white_ = false; return; }
    if (was_white_) return;  // leading edge only
    was_white_ = true;

    double rx = latest_.pose.pose.position.x;
    double ry = latest_.pose.pose.position.y;

    for (const auto & l : lines_) {
      double dist  = (l.axis == 'x') ? std::abs(rx - l.pos_m) : std::abs(ry - l.pos_m);
      double along = (l.axis == 'x') ? ry : rx;
      if (dist > l.tol_m || along < l.start || along > l.end) continue;

      geometry_msgs::msg::PoseWithCovarianceStamped corrected;
      corrected.header.stamp    = now();
      corrected.header.frame_id = "map";
      corrected.pose.pose.position.x = (l.axis == 'x') ? l.pos_m : rx;
      corrected.pose.pose.position.y = (l.axis == 'y') ? l.pos_m : ry;
      corrected.pose.pose.orientation = latest_.pose.pose.orientation;
      corrected.pose.covariance.fill(0.0);
      corrected.pose.covariance[0]  = (l.axis == 'x') ? 0.001 : 0.05;
      corrected.pose.covariance[7]  = (l.axis == 'y') ? 0.001 : 0.05;
      corrected.pose.covariance[35] = 0.05;
      pose_pub_->publish(corrected);

      RCLCPP_INFO(get_logger(), "Line correction: %s → %c=%.3f (was %.3fm off)",
        l.name.c_str(), l.axis, l.pos_m, dist);
      return;
    }
  }

  std::vector<WhiteLine> lines_;
  nav_msgs::msg::Odometry latest_;
  bool was_white_ = false;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr    color_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhiteLineCorrectorNode>());
  rclcpp::shutdown();
}
