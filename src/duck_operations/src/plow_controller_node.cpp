#include "duck_operations/plow_controller_node.hpp"
#include <cmath>
#include <numeric>

namespace duck_operations {

PlowControllerNode::PlowControllerNode() : Node("plow_controller_node") {
  declare_parameter("wheel_radius", 0.050);
  wheel_radius_ = get_parameter("wheel_radius").as_double();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  state_pub_   = create_publisher<std_msgs::msg::String>("/plow_state", 10);

  cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "/plow_command", 10,
    std::bind(&PlowControllerNode::onCommand, this, std::placeholders::_1));

  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&PlowControllerNode::onJointStates, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds((int)(LOOP_DT * 1000)),
    std::bind(&PlowControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "Plow controller started");
}

void PlowControllerNode::onCommand(const std_msgs::msg::String::SharedPtr msg) {
  const std::string & cmd = msg->data;
  RCLCPP_INFO(get_logger(), "Plow command: %s", cmd.c_str());

  if (cmd == "START_PUSH") {
    dist_traveled_m_ = 0.0;
    state_ = PlowState::PUSHING;
  } else if (cmd == "PRESS_BUTTON") {
    press_cycles_remaining_ = PRESS_CYCLES;
    press_fwd_              = true;
    dist_traveled_m_        = 0.0;
    wheel_pos_init_         = false;
    state_ = PlowState::BUTTON_PRESSING;
  } else if (cmd == "RETREAT") {
    dist_traveled_m_ = 0.0;
    state_ = PlowState::RETREATING;
  } else if (cmd == "STOP") {
    stopRobot();
    state_ = PlowState::IDLE;
  }
}

void PlowControllerNode::onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < 4) return;
  if (!wheel_pos_init_) {
    for (int i = 0; i < 4; i++) last_wheel_pos_[i] = msg->position[i];
    wheel_pos_init_ = true;
    return;
  }
  // Compute distance traveled from average wheel rotation
  double delta = 0.0;
  for (int i = 0; i < 4; i++) {
    delta += std::abs(msg->position[i] - last_wheel_pos_[i]);
    last_wheel_pos_[i] = msg->position[i];
  }
  dist_traveled_m_ += (delta / 4.0) * wheel_radius_;
}

void PlowControllerNode::controlLoop() {
  auto state_msg = std_msgs::msg::String();

  switch (state_) {
    case PlowState::PUSHING: {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = PUSH_SPEED;
      cmd_vel_pub_->publish(cmd);
      state_msg.data = "PUSHING";
      // task_manager monitors color sensor to know when duck is in blue square
      break;
    }

    case PlowState::BUTTON_PRESSING:
      buttonPressLoop();
      state_msg.data = "BUTTON_PRESSING";
      break;

    case PlowState::RETREATING: {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -REVERSE_SPEED;
      cmd_vel_pub_->publish(cmd);
      state_msg.data = "RETREATING";
      if (dist_traveled_m_ >= 0.20) {   // retreat 20cm then stop
        stopRobot();
        state_ = PlowState::DONE;
      }
      break;
    }

    case PlowState::DONE:
      state_msg.data = "DONE";
      break;

    default:
      state_msg.data = "IDLE";
      break;
  }

  state_pub_->publish(state_msg);
}

void PlowControllerNode::buttonPressLoop() {
  geometry_msgs::msg::Twist cmd;

  double target = press_fwd_ ? PRESS_DIST_M : REVERSE_DIST_M;

  if (dist_traveled_m_ < target) {
    cmd.linear.x = press_fwd_ ? PRESS_SPEED : -REVERSE_SPEED;
    cmd_vel_pub_->publish(cmd);
  } else {
    stopRobot();
    dist_traveled_m_  = 0.0;
    wheel_pos_init_   = false;

    if (!press_fwd_) {
      press_cycles_remaining_--;
      RCLCPP_INFO(get_logger(), "Button press cycle done — %d remaining",
                  press_cycles_remaining_);
    }

    if (press_cycles_remaining_ <= 0) {
      state_ = PlowState::DONE;
      RCLCPP_INFO(get_logger(), "Button press sequence complete");
      return;
    }
    press_fwd_ = !press_fwd_;   // toggle direction
  }
}

void PlowControllerNode::stopRobot() {
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
}

} // namespace duck_operations

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<duck_operations::PlowControllerNode>());
  rclcpp::shutdown();
}
