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

// ── Mission phases ──────────────────────────────────────────────────────────
enum class MissionPhase {
  WAITING_FOR_START,
  RUNNING,           // unified — steps through mission_queue_ in order
  RETURNING_HOME,
  COMPLETE
};

// ── Mission item types ──────────────────────────────────────────────────────
enum class MissionItemType {
  STATION,      // navigate to station, do task
  DUCK,         // find duck, push to blue square
  CRATER_LOOP,  // extend plank, drive clockwise around crater, retract plank
  TRANSMIT_IR   // send all recorded antenna colors via IR transmitter
};

enum class ApproachMode {
  NORMAL,   // Nav2 direct
  REVERSE,  // back up using ToF 0+1 (rear L+R)
  STRAFE    // strafe left using ToF 2 (right side)
};

// ── Station definition ──────────────────────────────────────────────────────
struct ArenaStation {
  std::string  name;
  double       x, y, yaw;
  bool         use_plow;
  ApproachMode approach           = ApproachMode::NORMAL;
  double       approach_target_m  = 0.12;
  int          approach_sensor    = -1;
  std::string  task_command;
  int          read_led_antenna   = 0;  // 1-4 = read LED after task, 0 = skip
};

// ── Duck push modes ─────────────────────────────────────────────────────────
enum class DuckPushMode {
  NORMAL,    // push directly to blue square, reverse to approx position
  UTURN,     // push forward 0.559m, pivot around duck, push to blue square,
             // then reverse to approx position
  UTURN_TO_STATION  // same U-turn push but after drop, reverse to next
                    // station approach position instead of approx position
};

// ── Duck definition ─────────────────────────────────────────────────────────
struct DuckTarget {
  std::string  name;
  double       approx_x, approx_y;
  double       blue_square_x, blue_square_y;
  DuckPushMode push_mode         = DuckPushMode::NORMAL;
  double       after_x           = 0.0;
  double       after_y           = 0.0;
  double       after_yaw         = 0.0;
  bool         skip_retreat      = false;  // true = nav directly to next item
                                           // instead of retreating to approx pos
};

// ── Crater loop definition ──────────────────────────────────────────────────
struct CraterLoop {
  double center_x, center_y;   // crater centre in map frame (metres)
  double radius;                // drive radius around crater (metres)
  int    num_waypoints;         // how many waypoints around perimeter
};

// ── Unified mission item ────────────────────────────────────────────────────
struct MissionItem {
  MissionItemType type;
  ArenaStation    station;    // used when type == STATION
  DuckTarget      duck;       // used when type == DUCK
  CraterLoop      crater;     // used when type == CRATER_LOOP
};

class TaskManagerNode : public rclcpp::Node {
public:
  TaskManagerNode();

private:
  using NavToPose  = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavToPose>;

  // ── State machine ──────────────────────────────────────────────────────
  void missionTick();
  void transitionTo(MissionPhase phase);

  // ── Item executors ─────────────────────────────────────────────────────
  void executeStation(const ArenaStation & s);
  void executeDuck(const DuckTarget & d);
  void executeCraterLoop(const CraterLoop & c);

  // ── Helpers ────────────────────────────────────────────────────────────
  void sendNavGoal(double x, double y, double yaw,
                   std::function<void(bool)> cb = nullptr);
  void sendTeensyCmd(const std::string & cmd);
  void startSearch(double cx, double cy);
  void tickSearch();

  // ── Callbacks ──────────────────────────────────────────────────────────
  void onStartSignal      (const std_msgs::msg::Bool::SharedPtr msg);
  void onTaskStatus       (const std_msgs::msg::String::SharedPtr msg);
  void onPlowState        (const std_msgs::msg::String::SharedPtr msg);
  void onPositionVerified (const std_msgs::msg::Bool::SharedPtr msg);
  void onOdom             (const nav_msgs::msg::Odometry::SharedPtr msg);
  void onDuckDetections   (const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onLineState        (const std_msgs::msg::String::SharedPtr msg);

  // ── Action client ──────────────────────────────────────────────────────
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;

  // ── Publishers ─────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     phase_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     task_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     plow_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     teensy_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ── Subscriptions ──────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              start_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            task_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            plow_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              verified_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    duck_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tof_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            line_sub_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  // ── Mission queue ──────────────────────────────────────────────────────
  std::vector<MissionItem> mission_queue_;
  size_t current_item_ = 0;
  int    step_         = 0;

  // ── Crater loop state ──────────────────────────────────────────────────
  int    crater_wp_      = 0;   // current waypoint index
  bool   plank_extended_ = false;

  // ── Nav state ──────────────────────────────────────────────────────────
  bool nav_complete_  = false;
  bool nav_succeeded_ = false;

  // ── Task state ─────────────────────────────────────────────────────────
  bool        position_verified_ = false;
  bool        task_done_         = false;
  bool        plow_done_         = false;
  std::string last_task_result_;
  std::string last_plow_state_;
  std::string line_state_        = "BLACK";

  // ── Odometry / home ────────────────────────────────────────────────────
  geometry_msgs::msg::PoseStamped home_pose_;
  bool home_saved_ = false;

  // ── Duck detection ─────────────────────────────────────────────────────
  bool   duck_visible_ = false;
  double duck_pixel_x_ = 0.0;
  double duck_pixel_y_ = 0.0;

  // ── U-turn push state ──────────────────────────────────────────────────
  static constexpr double UTURN_PUSH_DIST_M = 0.559;  // 22 inches
  double uturn_start_x_ = 0.0;
  double uturn_start_y_ = 0.0;
  double current_x_     = 0.0;
  double current_y_     = 0.0;
  double current_yaw_   = 0.0;

  // ── Search state ───────────────────────────────────────────────────────
  bool   searching_       = false;
  int    search_step_     = 0;
  double search_center_x_ = 0.0;
  double search_center_y_ = 0.0;
  rclcpp::Time search_start_time_;

  std_msgs::msg::Float32MultiArray latest_tof_;

  MissionPhase phase_ = MissionPhase::WAITING_FOR_START;

  static constexpr double SEARCH_TIMEOUT_S  = 15.0;
  static constexpr int    NUM_SEARCH_POINTS = 8;
  const double SEARCH_OFFSETS[8][2] = {
    { 0.0,  0.0}, { 0.2,  0.0}, { 0.0,  0.2}, {-0.2,  0.0},
    { 0.0, -0.2}, { 0.3,  0.3}, {-0.3,  0.3}, {-0.3, -0.3},
  };
};

} // namespace task_manager
