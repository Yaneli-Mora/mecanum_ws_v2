#include "mecanum_sensors/teensy_bridge_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

struct __attribute__((packed)) TeensyToPiPacket {
  uint8_t  start_byte;
  int16_t  enc_ticks[4];
  uint16_t tof_mm[3];             // ultrasonics removed — on Arduino now
  int16_t  flow_vx, flow_vy;
  uint8_t  color_left, color_right;
  uint8_t  start_detected;
  uint8_t  checksum;
};
static const size_t PKT_SIZE = sizeof(TeensyToPiPacket);

namespace mecanum_sensors {

TeensyBridgeNode::TeensyBridgeNode() : Node("teensy_bridge_node") {
  declare_parameter("serial_port", "/dev/teensy");
  serial_port_ = get_parameter("serial_port").as_string();

  // us_pub_ removed — ultrasonics now on Arduino, published by attachment_node
  flow_pub_        = create_publisher<std_msgs::msg::Int16MultiArray>("/flow_raw", 10);
  tof_pub_         = create_publisher<std_msgs::msg::Float32MultiArray>("/tof_distances", 10);
  color_left_pub_  = create_publisher<std_msgs::msg::String>("/color_sensor/left",  10);
  color_right_pub_ = create_publisher<std_msgs::msg::String>("/color_sensor/right", 10);
  color_zone_pub_  = create_publisher<std_msgs::msg::String>("/color_sensor/zone",  10);
  start_pub_       = create_publisher<std_msgs::msg::Bool>("/start_signal", 10);

  if (!openSerial()) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s", serial_port_.c_str());
  }

  timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&TeensyBridgeNode::readLoop, this));

  RCLCPP_INFO(get_logger(), "Teensy bridge started on %s", serial_port_.c_str());
}

TeensyBridgeNode::~TeensyBridgeNode() { if (fd_ >= 0) ::close(fd_); }

bool TeensyBridgeNode::openSerial() {
  fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;
  struct termios tty{};
  memset(&tty, 0, sizeof(tty));
  cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
  tty.c_cflag  = CS8 | CLOCAL | CREAD;
  tty.c_iflag  = 0; tty.c_lflag = 0; tty.c_oflag = 0;
  tty.c_cc[VMIN] = 1; tty.c_cc[VTIME] = 1;
  tcsetattr(fd_, TCSANOW, &tty);
  return true;
}

bool TeensyBridgeNode::readPacket() {
  if (fd_ < 0) return false;

  // Sync to start byte
  uint8_t b = 0;
  for (int i = 0; i < 256; i++) {
    if (::read(fd_, &b, 1) == 1 && b == 0xFF) break;
    if (i == 255) return false;
  }

  uint8_t buf[PKT_SIZE];
  buf[0] = 0xFF;
  size_t got = 1;
  while (got < PKT_SIZE) {
    int r = ::read(fd_, buf + got, PKT_SIZE - got);
    if (r <= 0) return false;
    got += r;
  }

  // Verify checksum
  uint8_t cs = 0;
  for (size_t i = 0; i < PKT_SIZE - 1; i++) cs ^= buf[i];
  if (cs != buf[PKT_SIZE - 1]) return false;

  TeensyToPiPacket pkt;
  memcpy(&pkt, buf, PKT_SIZE);

  auto stamp = now();

  // Ultrasonics published by attachment_node (Arduino)

  // Publish ToF (m) — 3 sensors
  auto tof_msg = std_msgs::msg::Float32MultiArray();
  tof_msg.data.push_back(pkt.tof_mm[0] / 1000.0f);
  tof_msg.data.push_back(pkt.tof_mm[1] / 1000.0f);
  tof_msg.data.push_back(pkt.tof_mm[2] / 1000.0f);
  tof_pub_->publish(tof_msg);

  // Publish optical flow raw pixel deltas [vx, vy]
  auto flow_msg = std_msgs::msg::Int16MultiArray();
  flow_msg.data.push_back(pkt.flow_vx);
  flow_msg.data.push_back(pkt.flow_vy);
  flow_pub_->publish(flow_msg);

  // Publish color
  std::string cl = classifyColor(pkt.color_left);
  std::string cr = classifyColor(pkt.color_right);
  auto cl_msg = std_msgs::msg::String(); cl_msg.data = cl; color_left_pub_->publish(cl_msg);
  auto cr_msg = std_msgs::msg::String(); cr_msg.data = cr; color_right_pub_->publish(cr_msg);

  auto zone_msg = std_msgs::msg::String();
  zone_msg.data = (cl == cr && cl != "black") ? cl : (cl == "white" || cr == "white") ? "white" : "black";
  color_zone_pub_->publish(zone_msg);

  // Publish start signal once
  if (pkt.start_detected && !start_sent_) {
    auto s = std_msgs::msg::Bool(); s.data = true;
    start_pub_->publish(s);
    start_sent_ = true;
    RCLCPP_INFO(get_logger(), "START SIGNAL received from Teensy");
  }
  return true;
}

void TeensyBridgeNode::readLoop() { readPacket(); }

std::string TeensyBridgeNode::classifyColor(uint8_t code) {
  switch (code) {
    case 1: return "white";
    case 2: return "green";
    case 3: return "blue";
    default: return "black";
  }
}

}  // namespace mecanum_sensors

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_sensors::TeensyBridgeNode>());
  rclcpp::shutdown();
}
