#include "duck_operations/duck_detector_node.hpp"
#include <cmath>

namespace duck_operations {

DuckDetectorNode::DuckDetectorNode() : Node("duck_detector_node") {
  declare_parameter("camera_height_m", 0.30);
  declare_parameter("fx", 554.0); declare_parameter("fy", 554.0);
  declare_parameter("cx", 320.0); declare_parameter("cy", 240.0);
  camera_height_m_ = get_parameter("camera_height_m").as_double();
  fx_ = get_parameter("fx").as_double(); fy_ = get_parameter("fy").as_double();
  cx_ = get_parameter("cx").as_double(); cy_ = get_parameter("cy").as_double();

  img_sub_  = create_subscription<sensor_msgs::msg::Image>(
    "/camera/forward/image_raw", 10,
    std::bind(&DuckDetectorNode::onImage, this, std::placeholders::_1));
  duck_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/duck_detections", 10);

  RCLCPP_INFO(get_logger(), "Duck detector started (HSV yellow threshold active)");
}

void DuckDetectorNode::onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try { cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); }
  catch (...) { return; }

  // Convert to HSV and threshold for yellow duck color
  cv::Mat hsv, mask, morph;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, HSV_LOW, HSV_HIGH, mask);

  // Remove noise
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
  cv::morphologyEx(mask, morph, cv::MORPH_OPEN,  kernel);
  cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, kernel);

  // Find contours of duck blobs
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  geometry_msgs::msg::PoseArray detections;
  detections.header = msg->header;

  for (auto & c : contours) {
    double area = cv::contourArea(c);
    if (area < 800) continue;  // filter small blobs

    cv::Moments m = cv::moments(c);
    if (m.m00 == 0) continue;

    double px = m.m10 / m.m00;  // pixel centroid x
    double py = m.m01 / m.m00;  // pixel centroid y

    // Project to floor plane using pinhole model + known camera height
    // Assumes camera points forward with slight downward pitch
    double dx_cam = (px - cx_) / fx_ * camera_height_m_;
    double dy_cam = (py - cy_) / fy_ * camera_height_m_;

    // dx_cam = forward distance from camera, dy_cam = lateral offset
    // These are in camera frame — task_manager transforms to map frame via TF
    geometry_msgs::msg::Pose pose;
    pose.position.x = dx_cam;
    pose.position.y = dy_cam;
    pose.position.z = 0.0;
    detections.poses.push_back(pose);
  }

  if (!detections.poses.empty())
    duck_pub_->publish(detections);
}

} // namespace duck_operations

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<duck_operations::DuckDetectorNode>());
  rclcpp::shutdown();
}
