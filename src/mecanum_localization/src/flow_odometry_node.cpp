// flow_odometry_node.cpp
// Converts raw optical flow pixel deltas from CJMCU-3901 (PMW3901)
// into a nav_msgs/Odometry message for the EKF to fuse.
//
// The key parameter is sensor_height_m — the height of the sensor
// above the floor. Measure this accurately for best results.
//
// Scale factor: velocity_m_s = (delta_pixels / dt) * pixel_scale
// where pixel_scale = sensor_height_m * tan(fov_rad / resolution)
// For PMW3901: resolution = 30x30 pixels, FOV = 42 degrees

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class FlowOdometryNode : public rclcpp::Node {
public:
  FlowOdometryNode() : Node("flow_odometry_node") {

    // ── Parameters ──────────────────────────────────────────────────────
    declare_parameter("sensor_height_m",  0.05);   // height above floor in meters
                                                    // MEASURE THIS accurately
    declare_parameter("loop_rate_hz",     100.0);  // must match Teensy loop rate
    declare_parameter("frame_id",         std::string("odom"));
    declare_parameter("child_frame_id",   std::string("base_link"));

    // PMW3901 sensor parameters (do not change unless you know what you're doing)
    declare_parameter("sensor_fov_deg",   42.0);   // field of view in degrees
    declare_parameter("sensor_res_px",    30.0);   // pixels across FOV

    sensor_height_m_ = get_parameter("sensor_height_m").as_double();
    loop_rate_hz_    = get_parameter("loop_rate_hz").as_double();
    frame_id_        = get_parameter("frame_id").as_string();
    child_frame_id_  = get_parameter("child_frame_id").as_string();

    double fov_rad  = get_parameter("sensor_fov_deg").as_double() * M_PI / 180.0;
    double res_px   = get_parameter("sensor_res_px").as_double();

    // pixel_scale: meters of real motion per pixel at given height
    // derived from pinhole camera model: scale = 2 * h * tan(fov/2) / resolution
    pixel_scale_ = (2.0 * sensor_height_m_ * std::tan(fov_rad / 2.0)) / res_px;

    RCLCPP_INFO(get_logger(),
      "Flow odometry: height=%.3fm pixel_scale=%.6f m/px",
      sensor_height_m_, pixel_scale_);

    // ── Publishers / Subscribers ─────────────────────────────────────────
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/flow", 10);

    flow_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "/flow_raw", 10,
      std::bind(&FlowOdometryNode::onFlow, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Flow odometry node ready");
  }

private:
  void onFlow(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;

    int16_t raw_vx = msg->data[0];  // pixel delta X (right = positive)
    int16_t raw_vy = msg->data[1];  // pixel delta Y (forward = positive)

    // Convert pixel deltas to velocity in m/s
    // delta is accumulated since last packet (at loop_rate_hz_)
    double dt = 1.0 / loop_rate_hz_;
    double vx = (raw_vx * pixel_scale_) / dt;  // m/s lateral
    double vy = (raw_vy * pixel_scale_) / dt;  // m/s forward

    // Accumulate position
    pos_x_ += vy * dt;  // forward = robot x axis
    pos_y_ += vx * dt;  // lateral = robot y axis

    // Build odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now();
    odom.header.frame_id = frame_id_;
    odom.child_frame_id  = child_frame_id_;

    // Position
    odom.pose.pose.position.x = pos_x_;
    odom.pose.pose.position.y = pos_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;  // no rotation from flow sensor alone

    // Velocity
    odom.twist.twist.linear.x = vy;   // forward
    odom.twist.twist.linear.y = vx;   // lateral
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = 0.0; // flow sensor doesn't measure rotation

    // Covariance — tune these after calibration
    // Higher values = less trust from EKF
    // Position covariance
    odom.pose.covariance[0]  = 0.01;  // x
    odom.pose.covariance[7]  = 0.01;  // y
    odom.pose.covariance[35] = 1e6;   // yaw (not measured, very high uncertainty)

    // Velocity covariance
    odom.twist.covariance[0]  = 0.005;  // vx — flow is quite accurate
    odom.twist.covariance[7]  = 0.005;  // vy
    odom.twist.covariance[35] = 1e6;    // angular z (not measured)

    odom_pub_->publish(odom);
  }

  // Parameters
  double sensor_height_m_ = 0.05;
  double loop_rate_hz_    = 100.0;
  double pixel_scale_     = 0.0;
  std::string frame_id_;
  std::string child_frame_id_;

  // Accumulated position
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr flow_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlowOdometryNode>());
  rclcpp::shutdown();
}