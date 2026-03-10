#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

namespace duck_operations {

class DuckDetectorNode : public rclcpp::Node {
public:
  DuckDetectorNode();
private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr duck_pub_;

  // HSV thresholds for yellow rubber duck
  // Tune with: ros2 run rqt_image_view rqt_image_view
  cv::Scalar HSV_LOW  {8, 35, 150};   // yellow-orange lower
  cv::Scalar HSV_HIGH {25, 255, 255};   // yellow-orange upper

  // Camera intrinsics (from calibration file)
  double fx_ = 554.0, fy_ = 554.0;     // focal length px — update after calibration
  double cx_ = 320.0, cy_ = 240.0;     // principal point
  double camera_height_m_ = 0.30;      // height of camera above floor
};

} // namespace duck_operations
