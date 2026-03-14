// camera_node.cpp
// Captures frames from a USB/Pi camera via OpenCV (V4L2)
// and publishes them as sensor_msgs/Image on /camera/forward/image_raw
//
// Parameters:
//   device_id  (int)    — V4L2 device index, default 0 (/dev/video0)
//   width      (int)    — capture width,  default 640
//   height     (int)    — capture height, default 480
//   fps        (int)    — capture fps,    default 30
//   frame_id   (string) — TF frame,       default "camera_forward_link"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    declare_parameter("device_id", 0);
    declare_parameter("width",     640);
    declare_parameter("height",    480);
    declare_parameter("fps",       30);
    declare_parameter("frame_id",  std::string("camera_forward_link"));

    int device_id = get_parameter("device_id").as_int();
    width_        = get_parameter("width").as_int();
    height_       = get_parameter("height").as_int();
    int fps       = get_parameter("fps").as_int();
    frame_id_     = get_parameter("frame_id").as_string();

    cap_.open(device_id, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open camera device %d", device_id);
      return;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps);

    img_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/forward/image_raw", 10);

    int interval_ms = 1000 / fps;
    timer_ = create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      std::bind(&CameraNode::captureFrame, this));

    RCLCPP_INFO(get_logger(),
      "Camera node started: device=%d %dx%d @ %dfps",
      device_id, width_, height_, fps);
  }

private:
  void captureFrame() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Failed to capture frame");
      return;
    }

    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp    = now();
    msg->header.frame_id = frame_id_;
    img_pub_->publish(*msg);
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int         width_    = 640;
  int         height_   = 480;
  std::string frame_id_ = "camera_forward_link";
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
}
