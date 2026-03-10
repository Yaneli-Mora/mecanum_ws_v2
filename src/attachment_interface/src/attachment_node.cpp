#include "attachment_interface/attachment_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <sstream>

namespace attachment_interface {

AttachmentNode::AttachmentNode() : Node("attachment_node") {
  declare_parameter("serial_port", "/dev/arduino_attachment");
  port_ = get_parameter("serial_port").as_string();

  result_pub_    = create_publisher<std_msgs::msg::String>("/task_status",        10);
  line_pub_      = create_publisher<std_msgs::msg::String>("/line_state",         10);
  us_pub_        = create_publisher<std_msgs::msg::Float32MultiArray>("/ultrasonic_distances", 10);

  cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "/task_command", 10,
    std::bind(&AttachmentNode::onCommand, this, std::placeholders::_1));

  if (openSerial(port_)) {
    RCLCPP_INFO(get_logger(), "Arduino interface open on %s", port_.c_str());
    // Wait for READY
    auto resp = readLineBlocking(2000);
    RCLCPP_INFO(get_logger(), "Arduino says: %s", resp.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open %s", port_.c_str());
  }

  // Poll serial at 50Hz for streaming US: and LINE: messages
  serial_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&AttachmentNode::pollSerial, this));
}

AttachmentNode::~AttachmentNode() { closeSerial(); }

bool AttachmentNode::openSerial(const std::string & port) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;
  struct termios tty{};
  memset(&tty, 0, sizeof(tty));
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag  = CS8 | CLOCAL | CREAD;
  tty.c_iflag  = IGNPAR;
  tty.c_lflag  = 0;
  tty.c_oflag  = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;
  tcsetattr(fd_, TCSANOW, &tty);
  return true;
}

void AttachmentNode::closeSerial() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// Read characters into buffer, dispatch complete lines
void AttachmentNode::pollSerial() {
  if (fd_ < 0 || busy_) return;
  char c;
  while (::read(fd_, &c, 1) == 1) {
    if (c == '\n') {
      // Trim \r
      while (!buf_.empty() && buf_.back() == '\r') buf_.pop_back();
      dispatchLine(buf_);
      buf_.clear();
    } else {
      buf_ += c;
    }
  }
}

void AttachmentNode::dispatchLine(const std::string & line) {
  if (line.empty()) return;

  if (line.rfind("US:", 0) == 0) {
    // US:d0,d1,d2,d3  — ultrasonic distances in mm
    auto msg = std_msgs::msg::Float32MultiArray();
    std::string data = line.substr(3);
    std::stringstream ss(data);
    std::string token;
    while (std::getline(ss, token, ',')) {
      try {
        msg.data.push_back(std::stof(token) / 1000.0f);  // mm → m
      } catch (...) {
        msg.data.push_back(9.999f);
      }
    }
    if (msg.data.size() == 4) us_pub_->publish(msg);

  } else if (line.rfind("LINE:", 0) == 0) {
    // LINE:WHITE / LINE:BLACK / LINE:COLORED
    std::string state = line.substr(5);
    auto msg = std_msgs::msg::String();
    msg.data = state;
    line_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Line state: %s", state.c_str());
  }
  // DONE / ERROR / READY responses are handled by readLineBlocking()
}

// Blocking read for command responses — used while busy_ is true
std::string AttachmentNode::readLineBlocking(int timeout_ms) {
  std::string response;
  char c;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (::read(fd_, &c, 1) == 1) {
      if (c == '\n') break;
      if (c != '\r') response += c;
      // Skip streaming lines while waiting for command response
      if (response.rfind("US:", 0) == 0 || response.rfind("LINE:", 0) == 0) {
        // Dispatch it then keep waiting
        dispatchLine(response);
        response.clear();
      }
    }
  }
  return response.empty() ? "TIMEOUT" : response;
}

void AttachmentNode::onCommand(const std_msgs::msg::String::SharedPtr msg) {
  if (fd_ < 0) return;
  std::lock_guard<std::mutex> lock(serial_mtx_);
  busy_ = true;

  std::string line = msg->data + "\n";
  ::write(fd_, line.c_str(), line.size());
  RCLCPP_INFO(get_logger(), "Sent: %s", msg->data.c_str());

  std::string result = readLineBlocking(5000);
  RCLCPP_INFO(get_logger(), "Result: %s", result.c_str());

  auto out = std_msgs::msg::String();
  out.data = result;
  result_pub_->publish(out);

  busy_ = false;
}

} // namespace attachment_interface

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<attachment_interface::AttachmentNode>());
  rclcpp::shutdown();
}
