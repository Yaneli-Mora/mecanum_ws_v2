#include "position_verifier/position_verifier_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace position_verifier {

PositionVerifierNode::PositionVerifierNode() : Node("position_verifier_node") {
  verified_pub_ = create_publisher<std_msgs::msg::Bool>("/position_verified", 10);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr m) { latest_odom_ = *m; });

  tof_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/tof_distances", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr m) { latest_tof_ = *m; });

  color_sub_ = create_subscription<std_msgs::msg::String>(
    "/color_sensor/zone", 10,
    [this](const std_msgs::msg::String::SharedPtr m) { latest_color_ = m->data; });

  nav_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");

  loadStationProfiles();
  RCLCPP_INFO(get_logger(), "Position verifier ready (%zu stations loaded)",
              stations_.size());
}

void PositionVerifierNode::loadStationProfiles() {
  // FIX: use insert() instead of brace-list assignment
  // StationProfile: name, x, y, yaw, tof_expected_m, zone_color,
  //                 pose_tol_m, tof_tol_m, tof_square_tol_m

  // tof_sensor: -1 = no ToF check (antennas trust EKF only)
  //              0 = use sensors 0+1 (left+right, keypad squareness check)
  //              2 = use sensor 2 (front, crank distance check)
  auto add = [&](const std::string & name,
                 double x, double y, double yaw,
                 double tof_m, const std::string & color,
                 int tof_sensor) {
    StationProfile p;
    p.name             = name;
    p.x                = x;
    p.y                = y;
    p.yaw              = yaw;
    p.tof_expected_m   = tof_m;
    p.zone_color       = color;
    p.tof_sensor       = tof_sensor;
    p.pose_tol_m       = 0.05;
    p.tof_tol_m        = 0.010;
    p.tof_square_tol_m = 0.008;
    stations_.insert({name, p});
  };

  // name           x      y      yaw     tof_m  color  tof_sensor
  add("antenna_1",    0.15,  0.75,  1.5708, 0.0,   "",    -1);  // EKF only
  add("antenna_2",    1.30,  0.75,  1.5708, 0.0,   "",    -1);  // EKF only
  add("antenna_3",    0.65,  0.40,  0.0,    0.0,   "",    -1);  // EKF only
  add("antenna_4",    0.65,  0.10,  1.5708, 0.0,   "",    -1);  // EKF only
  add("keypad_panel", 0.65,  0.80,  3.1416, 0.12,  "",     0);  // sensors 0+1 (L+R)
  add("crank",        0.10,  0.40,  0.0,    0.15,  "",     2);  // sensor 2 (front)
}

bool PositionVerifierNode::checkEKF(const StationProfile & p) {
  double dx   = latest_odom_.pose.pose.position.x - p.x;
  double dy   = latest_odom_.pose.pose.position.y - p.y;
  double dist = std::sqrt(dx*dx + dy*dy);
  bool ok = dist < p.pose_tol_m;
  if (!ok)
    RCLCPP_WARN(get_logger(), "EKF check FAIL: %.3fm from target (tol=%.3fm)",
                dist, p.pose_tol_m);
  return ok;
}

bool PositionVerifierNode::checkToF(const StationProfile & p) {
  // tof_sensor == -1 means no ToF check for this station (antennas use EKF only)
  if (p.tof_sensor < 0) return true;

  if ((int)latest_tof_.data.size() < 3) return false;

  if (p.tof_sensor == 0) {
    // Keypad: use sensors 0 (left) + 1 (right) — check distance AND squareness
    float l   = latest_tof_.data[0];
    float r   = latest_tof_.data[1];
    float avg = (l + r) / 2.0f;
    bool dist_ok   = std::abs(avg - p.tof_expected_m) < p.tof_tol_m;
    bool square_ok = std::abs(l - r)                  < p.tof_square_tol_m;
    if (!dist_ok)
      RCLCPP_WARN(get_logger(), "ToF dist FAIL: avg=%.3fm expected=%.3fm",
                  avg, p.tof_expected_m);
    if (!square_ok)
      RCLCPP_WARN(get_logger(), "ToF square FAIL: L=%.3fm R=%.3fm diff=%.3fm",
                  l, r, std::abs(l-r));
    return dist_ok && square_ok;
  }

  if (p.tof_sensor == 2) {
    // Crank: use sensor 2 (front) — check distance only
    float front = latest_tof_.data[2];
    bool dist_ok = std::abs(front - p.tof_expected_m) < p.tof_tol_m;
    if (!dist_ok)
      RCLCPP_WARN(get_logger(), "ToF front FAIL: %.3fm expected=%.3fm",
                  front, p.tof_expected_m);
    return dist_ok;
  }

  return true;
}

bool PositionVerifierNode::checkColor(const StationProfile & p) {
  if (p.zone_color.empty()) return true;
  bool ok = (latest_color_ == p.zone_color);
  if (!ok)
    RCLCPP_WARN(get_logger(), "Color check FAIL: got '%s' expected '%s'",
                latest_color_.c_str(), p.zone_color.c_str());
  return ok;
}

void PositionVerifierNode::nudgeCorrection(const StationProfile & p) {
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) return;
  auto goal = NavToPose::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp    = now();
  goal.pose.pose.position.x = p.x;
  goal.pose.pose.position.y = p.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, p.yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);
  nav_client_->async_send_goal(goal);
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
}

bool PositionVerifierNode::verifyAtStation(const std::string & name) {
  auto it = stations_.find(name);
  if (it == stations_.end()) {
    RCLCPP_ERROR(get_logger(), "Unknown station: %s", name.c_str());
    return false;
  }
  const auto & p = it->second;

  for (int attempt = 0; attempt <= max_retries_; ++attempt) {
    bool ekf_ok   = checkEKF(p);
    bool tof_ok   = checkToF(p);
    bool color_ok = checkColor(p);

    if (ekf_ok && tof_ok && color_ok) {
      RCLCPP_INFO(get_logger(), "Position verified at %s (attempt %d)",
                  name.c_str(), attempt + 1);
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      verified_pub_->publish(msg);
      return true;
    }

    if (attempt < max_retries_) {
      RCLCPP_INFO(get_logger(), "Nudging correction at %s (attempt %d)",
                  name.c_str(), attempt + 1);
      nudgeCorrection(p);
    }
  }

  RCLCPP_ERROR(get_logger(), "Position verification FAILED at %s after %d attempts",
               name.c_str(), max_retries_ + 1);
  auto msg = std_msgs::msg::Bool();
  msg.data = false;
  verified_pub_->publish(msg);
  return false;
}

}  // namespace position_verifier

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<position_verifier::PositionVerifierNode>());
  rclcpp::shutdown();
}
