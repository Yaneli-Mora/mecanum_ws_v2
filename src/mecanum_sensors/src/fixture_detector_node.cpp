#include "mecanum_sensors/fixture_detector_node.hpp"

namespace mecanum_sensors {

FixtureDetectorNode::FixtureDetectorNode() : Node("fixture_detector_node") {
  img_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/forward/image_raw", 10,
    std::bind(&FixtureDetectorNode::onImage, this, std::placeholders::_1));
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/fixture_pose", 10);
  RCLCPP_INFO(get_logger(), "Fixture detector started");
}

void FixtureDetectorNode::onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try { cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); }
  catch (cv_bridge::Exception & e) { return; }

  cv::Mat gray, blurred, edges;
  cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, blurred, cv::Size(5,5), 0);
  cv::Canny(blurred, edges, 50, 150);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (auto & c : contours) {
    double area = cv::contourArea(c);
    if (area < 500 || area > 50000) continue;
    cv::Moments m = cv::moments(c);
    if (m.m00 == 0) continue;
    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;

    // Publish centroid as a PoseStamped (pixel coords — convert to world in task_manager)
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position.x = cx;
    pose.pose.position.y = cy;
    pose_pub_->publish(pose);
    break;  // publish nearest/largest fixture
  }
}

}  // namespace mecanum_sensors

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_sensors::FixtureDetectorNode>());
  rclcpp::shutdown();
}
