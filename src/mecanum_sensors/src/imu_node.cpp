// imu_node.cpp — MPU-9250 reader via Linux I2C, publishes sensor_msgs/Imu
// Connects directly to Pi 5 I2C bus (/dev/i2c-1, pins SDA=GPIO2, SCL=GPIO3)
// Publishes to /imu/data which the EKF node consumes
//
// MPU-9250 default I2C address: 0x68 (AD0 low) or 0x69 (AD0 high)

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstring>
#include <stdexcept>

// ── MPU-9250 register map ─────────────────────────────────────────────────
static const uint8_t MPU_ADDR        = 0x68;  // change to 0x69 if AD0 pulled high
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_ACCEL_XOUT  = 0x3B;  // 6 bytes accel
static const uint8_t REG_TEMP_OUT    = 0x41;  // 2 bytes temp
static const uint8_t REG_GYRO_XOUT   = 0x43;  // 6 bytes gyro
static const uint8_t REG_WHO_AM_I    = 0x75;
static const uint8_t WHO_AM_I_VAL    = 0x71;  // expected response

// Scale factors (datasheet defaults)
// Accel: ±2g  → 16384 LSB/g
// Gyro:  ±250°/s → 131 LSB/(°/s)
static const double ACCEL_SCALE = 9.80665 / 16384.0;  // → m/s²
static const double GYRO_SCALE  = (M_PI / 180.0) / 131.0;  // → rad/s

class ImuNode : public rclcpp::Node {
public:
  ImuNode() : Node("imu_node") {
    declare_parameter("i2c_bus",   "/dev/i2c-1");
    declare_parameter("i2c_addr",  static_cast<int>(MPU_ADDR));
    declare_parameter("frame_id",  std::string("imu_link"));
    declare_parameter("publish_hz", 100.0);

    bus_path_ = get_parameter("i2c_bus").as_string();
    addr_     = static_cast<uint8_t>(get_parameter("i2c_addr").as_int());
    frame_id_ = get_parameter("frame_id").as_string();
    double hz = get_parameter("publish_hz").as_double();

    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    openI2C();
    initMPU();

    auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = create_wall_timer(period, std::bind(&ImuNode::readAndPublish, this));

    RCLCPP_INFO(get_logger(), "IMU node ready — %s addr=0x%02X @ %.0fHz",
                bus_path_.c_str(), addr_, hz);
  }

  ~ImuNode() { if (fd_ >= 0) close(fd_); }

private:
  // ── I2C helpers ──────────────────────────────────────────────────────────
  void openI2C() {
    fd_ = open(bus_path_.c_str(), O_RDWR);
    if (fd_ < 0)
      throw std::runtime_error("Cannot open I2C bus: " + bus_path_);
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0)
      throw std::runtime_error("Cannot set I2C slave address");
  }

  void writeReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(fd_, buf, 2) != 2)
      RCLCPP_WARN(get_logger(), "I2C write failed reg=0x%02X", reg);
  }

  uint8_t readReg(uint8_t reg) {
    write(fd_, &reg, 1);
    uint8_t val = 0;
    read(fd_, &val, 1);
    return val;
  }

  void readRegs(uint8_t reg, uint8_t* buf, int len) {
    write(fd_, &reg, 1);
    read(fd_, buf, len);
  }

  // ── MPU init ─────────────────────────────────────────────────────────────
  void initMPU() {
    uint8_t who = readReg(REG_WHO_AM_I);
    if (who != WHO_AM_I_VAL)
      RCLCPP_WARN(get_logger(),
        "WHO_AM_I=0x%02X (expected 0x71) — wrong sensor or address?", who);

    writeReg(REG_PWR_MGMT_1, 0x00);  // wake up, use internal oscillator
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    writeReg(REG_PWR_MGMT_1, 0x01);  // auto-select best clock source
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // ── Main read/publish loop ────────────────────────────────────────────────
  void readAndPublish() {
    uint8_t raw[14];
    readRegs(REG_ACCEL_XOUT, raw, 14);  // accel(6) + temp(2) + gyro(6)

    auto int16 = [&](int i) -> int16_t {
      return static_cast<int16_t>((raw[i] << 8) | raw[i + 1]);
    };

    double ax = int16(0)  * ACCEL_SCALE;
    double ay = int16(2)  * ACCEL_SCALE;
    double az = int16(4)  * ACCEL_SCALE;
    double gx = int16(8)  * GYRO_SCALE;
    double gy = int16(10) * GYRO_SCALE;
    double gz = int16(12) * GYRO_SCALE;

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp    = now();
    msg.header.frame_id = frame_id_;

    // Orientation — not computed here, let EKF integrate from gyro
    msg.orientation_covariance[0] = -1.0;  // signals "not provided"

    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;
    // Gyro noise: ~0.01 rad/s std dev → variance ~0.0001
    msg.angular_velocity_covariance = {
      0.0001, 0, 0,
      0, 0.0001, 0,
      0, 0, 0.0001
    };

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    // Accel noise: ~0.05 m/s² std dev → variance ~0.0025
    msg.linear_acceleration_covariance = {
      0.0025, 0, 0,
      0, 0.0025, 0,
      0, 0, 0.0025
    };

    pub_->publish(msg);
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string bus_path_, frame_id_;
  uint8_t addr_ = MPU_ADDR;
  int fd_ = -1;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
}
