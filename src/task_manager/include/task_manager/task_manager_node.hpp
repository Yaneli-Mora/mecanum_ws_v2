#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <string>
#include <functional>

namespace task_manager {

enum class MissionPhase {
  WAITING_FOR_START,
  INIT,
  PUSHING_DUCKS,
  PERFORMING_TASKS,
  RETURNING_HOME,
  COMPLETE
};

struct DuckTarget {
  std::string name;
  double approx_x, approx_y;
  double blue_square_x, blue_square_y;
};

enum class ApproachMode {
  NORMAL,   // Nav2 navigates directly, position verifier confirms
  REVERSE,  // Robot backs up using rear ToF sensors 0+1 (keypad)
  STRAFE    // Robot strafes sideways using side ToF sensor 2 (antenna_2)
};

struct ArenaStation {
  std::string  name;
  double       x, y, yaw;
  bool         use_plow;
  ApproachMode approach      = ApproachMode::NORMAL;
  double       approach_target_m = 0.12;  // target ToF distance for final approach
  int          approach_sensor   = -1;    // which ToF sensor to use for approach
  std::string  task_command;
};

class TaskManagerNode : public rclcpp::Node {
public:
  TaskManagerNode();

private:
  using NavToPose   = nav2_msgs::action::NavigateToPose;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<NavToPose>;

  // ── State machine ──────────────────────────────────────────────────────
  void missionTick();
  void transitionTo(MissionPhase phase);
  std::string phaseToString(MissionPhase p);

  // ── Navigation ─────────────────────────────────────────────────────────
  void sendNavGoal(double x, double y, double yaw,
                   std::function<void(bool)> on_result = nullptr);
  void cancelNav();

  // ── Task execution ─────────────────────────────────────────────────────
  void executeDuckPush(const DuckTarget & duck);
  void executeStation(const ArenaStation & station);

  // ── Search behavior ────────────────────────────────────────────────────
  void startSearch(double center_x, double center_y);
  void tickSearch();

  // ── Callbacks ──────────────────────────────────────────────────────────
  void onStartSignal(const std_msgs::msg::Bool::SharedPtr msg);
  void onTaskStatus(const std_msgs::msg::String::SharedPtr msg);
  void onPlowState(const std_msgs::msg::String::SharedPtr msg);
  void onPositionVerified(const std_msgs::msg::Bool::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onDuckDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // ── Action client ──────────────────────────────────────────────────────
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;

  // ── Publishers ─────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     phase_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     task_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     plow_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ── Subscriptions ──────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         start_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       task_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       plow_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         verified_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr duck_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tof_sub_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  // ── Mission state ──────────────────────────────────────────────────────
  MissionPhase phase_          = MissionPhase::WAITING_FOR_START;
  bool nav_complete_           = false;
  bool nav_succeeded_          = false;
  bool position_verified_      = false;
  bool task_done_              = false;
  bool plow_done_              = false;
  std::string last_task_result_;
  std::string last_plow_state_;

  geometry_msgs::msg::PoseStamped home_pose_;
  bool home_saved_ = false;

  std::vector<DuckTarget>   duck_queue_;
  std::vector<ArenaStation> station_queue_;
  size_t current_duck_    = 0;
  size_t current_station_ = 0;
  int    step_            = 0;

  std_msgs::msg::Float32MultiArray latest_tof_;

  // ── Duck detection state ───────────────────────────────────────────────
  bool   duck_visible_    = false;
  double duck_pixel_x_    = 0.0;
  double duck_pixel_y_    = 0.0;

  // ── Search state ───────────────────────────────────────────────────────
  bool   searching_       = false;
  int    search_step_     = 0;
  double search_center_x_ = 0.0;
  double search_center_y_ = 0.0;
  rclcpp::Time search_start_time_;

  static constexpr double SEARCH_TIMEOUT_S   = 15.0;
  static constexpr int    NUM_SEARCH_POINTS  = 8;

  // Spiral search offsets from approximate duck position (meters)
  const double SEARCH_OFFSETS[8][2] = {
    { 0.0,  0.0},   // stay at approx position first
    { 0.2,  0.0},   // 20cm right
    { 0.0,  0.2},   // 20cm forward
    {-0.2,  0.0},   // 20cm left
    { 0.0, -0.2},   // 20cm back
    { 0.3,  0.3},   // diagonal far
    {-0.3,  0.3},   // diagonal far
    {-0.3, -0.3},   // diagonal far
  };
};

} // namespace task_manager
