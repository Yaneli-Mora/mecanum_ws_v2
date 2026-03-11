#include "teensy2_interface/teensy2_bridge_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <sstream>

namespace teensy2_interface {

Teensy2BridgeNode::Teensy2BridgeNode() : Node("teensy2_bridge_node") {
  declare_parameter("serial_port", "/dev/teensy2");
  port_ = get_parameter("serial_port").as_string();

  // ── Publishers ─────────────────────────────────────────────────────────
  us_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>(
                         "/ultrasonic_distances", 10);
  line_pub_          = create_publisher<std_msgs::msg::String>("/line_state",      10);
  task_status_pub_   = create_publisher<std_msgs::msg::String>("/task_status",     10);
  antenna_colors_pub_= create_publisher<std_msgs::msg::String>("/antenna_colors",  10);

  // ── Subscriptions ──────────────────────────────────────────────────────
  // Listens for TURN_CRANK, PRESS_KEY, READ_LED, TRANSMIT_IR commands
  cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "/task_command", 10,
    std::bind(&Teensy2BridgeNode::onCommand, this, std::placeholders::_1));

  if (openSerial(port_)) {
    RCLCPP_INFO(get_logger(), "Teensy2 bridge open on %s", port_.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open %s", port_.c_str());
  }

  // Poll for spontaneous messages (LINE:*, US:*, COLOR:*) at 100Hz
  poll_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&Teensy2BridgeNode::pollSerial, this));
}

Teensy2BridgeNode::~Teensy2BridgeNode() { closeSerial(); }

bool Teensy2BridgeNode::openSerial(const std::string & port) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;
  struct termios tty{};
  memset(&tty, 0, sizeof(tty));
  cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
  tty.c_cflag  = CS8 | CLOCAL | CREAD;
  tty.c_iflag  = IGNPAR;
  tty.c_lflag  = 0; tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
  tcsetattr(fd_, TCSANOW, &tty);
  return true;
}

void Teensy2BridgeNode::closeSerial() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// ── Poll for spontaneous messages from Teensy2 ─────────────────────────────
void Teensy2BridgeNode::pollSerial() {
  if (fd_ < 0 || busy_) return;
  char c;
  while (::read(fd_, &c, 1) == 1) {
    if (c == '\n') {
      // Strip trailing \r
      while (!serial_buf_.empty() && serial_buf_.back() == '\r')
        serial_buf_.pop_back();
      if (!serial_buf_.empty())
        handleIncomingLine(serial_buf_);
      serial_buf_.clear();
    } else {
      serial_buf_ += c;
    }
  }
}

void Teensy2BridgeNode::handleIncomingLine(const std::string & line) {
  // ── LINE:* ─────────────────────────────────────────────────────────────
  if (line.rfind("LINE:", 0) == 0) {
    auto msg = std_msgs::msg::String();
    msg.data = line.substr(5);
    line_pub_->publish(msg);
    return;
  }

  // ── US:<f1>,<f2>,<r1>,<r2> ────────────────────────────────────────────
  if (line.rfind("US:", 0) == 0) {
    auto msg = std_msgs::msg::Float32MultiArray();
    std::stringstream ss(line.substr(3));
    std::string token;
    while (std::getline(ss, token, ',')) {
      try { msg.data.push_back(std::stof(token) / 1000.0f); }  // mm → m
      catch (...) { msg.data.push_back(9.999f); }
    }
    if (msg.data.size() == 4) us_pub_->publish(msg);
    return;
  }

  // ── COLOR:<antenna>:<color> ────────────────────────────────────────────
  if (line.rfind("COLOR:", 0) == 0) {
    // e.g. "COLOR:2:GREEN"
    auto msg = std_msgs::msg::String();
    msg.data = line;
    antenna_colors_pub_->publish(msg);

    // Parse and store locally
    auto p1 = line.find(':', 6);
    if (p1 != std::string::npos) {
      int ant_idx = std::stoi(line.substr(6, p1 - 6)) - 1;
      std::string color = line.substr(p1 + 1);
      if (ant_idx >= 0 && ant_idx < 4) {
        antenna_colors_[ant_idx] = color;
        RCLCPP_INFO(get_logger(), "Antenna %d LED = %s", ant_idx + 1, color.c_str());
      }
    }
    return;
  }
}

// ── Handle commands from task_manager ─────────────────────────────────────
void Teensy2BridgeNode::onCommand(const std_msgs::msg::String::SharedPtr msg) {
  const std::string & cmd = msg->data;
  RCLCPP_INFO(get_logger(), "Sending to Teensy2: %s", cmd.c_str());

  std::string result = sendAndWait(cmd, 8000);
  RCLCPP_INFO(get_logger(), "Teensy2 response: %s", result.c_str());

  auto out = std_msgs::msg::String();
  out.data = result;
  task_status_pub_->publish(out);
}

std::string Teensy2BridgeNode::sendAndWait(const std::string & cmd, int timeout_ms) {
  std::lock_guard<std::mutex> lock(serial_mtx_);
  if (fd_ < 0) return "ERROR_NO_PORT";
  busy_ = true;

  std::string line = cmd + "\n";
  ::write(fd_, line.c_str(), line.size());

  // Read response — skip spontaneous US:/LINE: messages, wait for DONE/ERROR/COLOR:
  std::string response;
  char c;
  std::string buf;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    int r = ::read(fd_, &c, 1);
    if (r == 1) {
      if (c == '\n') {
        while (!buf.empty() && buf.back() == '\r') buf.pop_back();
        // Skip spontaneous sensor messages
        if (buf.rfind("US:",    0) != 0 &&
            buf.rfind("LINE:",  0) != 0) {
          response = buf;
          break;
        }
        buf.clear();
      } else {
        buf += c;
      }
    }
  }

  busy_ = false;
  return response.empty() ? "TIMEOUT" : response;
}

} // namespace teensy2_interface

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<teensy2_interface::Teensy2BridgeNode>());
  rclcpp::shutdown();
}
