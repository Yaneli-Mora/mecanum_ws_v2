#include "mecanum_sensors/teensy2_bridge_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <sstream>

namespace mecanum_sensors {

Teensy2BridgeNode::Teensy2BridgeNode() : Node("teensy2_bridge_node") {
  declare_parameter("serial_port", "/dev/teensy2");
  port_ = get_parameter("serial_port").as_string();

  // ── Publishers ─────────────────────────────────────────────────────────
  us_pub_             = create_publisher<std_msgs::msg::Float32MultiArray>(
                          "/ultrasonic_distances", 10);
  line_pub_           = create_publisher<std_msgs::msg::String>("/line_state",      10);
  task_status_pub_    = create_publisher<std_msgs::msg::String>("/task_status",     10);
  antenna_colors_pub_ = create_publisher<std_msgs::msg::String>("/antenna_colors",  10);

  // ── Subscriptions ──────────────────────────────────────────────────────
  // Commands: TURN_CRANK, READ_LED <1-4>, TRANSMIT_IR
  cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "/task_command", 10,
    std::bind(&Teensy2BridgeNode::onCommand, this, std::placeholders::_1));

  if (openSerial(port_)) {
    RCLCPP_INFO(get_logger(), "Teensy2 bridge open on %s", port_.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open %s", port_.c_str());
  }

  // Poll for spontaneous messages (LINE:*, US:*) at 100Hz
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

// ── Poll for spontaneous messages from Teensy2 (not during a command) ────
void Teensy2BridgeNode::pollSerial() {
  if (fd_ < 0 || busy_) return;
  char c;
  while (::read(fd_, &c, 1) == 1) {
    if (c == '\n') {
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
  // ── LINE:WHITE / LINE:COLORED / LINE:BLACK ─────────────────────────────
  if (line.rfind("LINE:", 0) == 0) {
    auto msg = std_msgs::msg::String();
    msg.data = line.substr(5);
    line_pub_->publish(msg);
    return;
  }

  // ── US:<fl>,<fr>,<rl>,<rr>  distances in mm → publish in metres ────────
  if (line.rfind("US:", 0) == 0) {
    auto msg = std_msgs::msg::Float32MultiArray();
    std::stringstream ss(line.substr(3));
    std::string token;
    while (std::getline(ss, token, ',')) {
      try { msg.data.push_back(std::stof(token) / 1000.0f); }
      catch (...) { msg.data.push_back(9.999f); }
    }
    if (msg.data.size() == 4) us_pub_->publish(msg);
    return;
  }

  // ── COLOR:<antenna>:<color>  LED reading result ─────────────────────────
  // e.g. "COLOR:2:GREEN"
  // Publish on /antenna_colors and store locally
  if (line.rfind("COLOR:", 0) == 0) {
    auto msg = std_msgs::msg::String();
    msg.data = line;
    antenna_colors_pub_->publish(msg);

    // Parse antenna index and color name
    auto p1 = line.find(':', 6);
    if (p1 != std::string::npos) {
      int ant_idx        = std::stoi(line.substr(6, p1 - 6)) - 1;
      std::string color  = line.substr(p1 + 1);
      if (ant_idx >= 0 && ant_idx < 4) {
        antenna_colors_[ant_idx] = color;
        RCLCPP_INFO(get_logger(), "Antenna %d LED = %s  (stored)",
          ant_idx + 1, color.c_str());
      }
    }
    return;
  }
}

// ── Handle commands from task_manager ─────────────────────────────────────
void Teensy2BridgeNode::onCommand(const std_msgs::msg::String::SharedPtr msg) {
  const std::string & cmd = msg->data;
  RCLCPP_INFO(get_logger(), "→ Teensy2: %s", cmd.c_str());

  std::string result = sendAndWait(cmd, 10000);
  RCLCPP_INFO(get_logger(), "← Teensy2: %s", result.c_str());

  auto out = std_msgs::msg::String();
  out.data = result;
  task_status_pub_->publish(out);
}

// ── Send command and wait for DONE / ERROR ────────────────────────────────
// Handles multi-line responses from READ_LED:
//   Teensy2 sends:  "COLOR:2:GREEN\n"  then  "DONE\n"
// We parse COLOR: lines into handleIncomingLine and wait for DONE/ERROR
std::string Teensy2BridgeNode::sendAndWait(const std::string & cmd, int timeout_ms) {
  std::lock_guard<std::mutex> lock(serial_mtx_);
  if (fd_ < 0) return "ERROR_NO_PORT";
  busy_ = true;

  // Send command
  std::string line = cmd + "\n";
  ::write(fd_, line.c_str(), line.size());

  std::string response;
  char        c;
  std::string buf;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    int r = ::read(fd_, &c, 1);
    if (r != 1) continue;

    if (c == '\n') {
      // Strip \r
      while (!buf.empty() && buf.back() == '\r') buf.pop_back();

      if (buf.empty()) continue;

      // Always skip spontaneous sensor lines
      if (buf.rfind("US:",   0) == 0 ||
          buf.rfind("LINE:", 0) == 0) {
        buf.clear();
        continue;
      }

      // COLOR: line — parse and publish, then keep waiting for DONE
      if (buf.rfind("COLOR:", 0) == 0) {
        handleIncomingLine(buf);
        buf.clear();
        continue;
      }

      // DONE or ERROR — command complete
      if (buf == "DONE" || buf == "ERROR" || buf == "TIMEOUT") {
        response = buf;
        buf.clear();
        break;
      }

      buf.clear();
    } else {
      buf += c;
    }
  }

  busy_ = false;
  return response.empty() ? "TIMEOUT" : response;
}

} // namespace mecanum_sensors

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_sensors::Teensy2BridgeNode>());
  rclcpp::shutdown();
}
