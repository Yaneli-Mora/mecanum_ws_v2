#include "task_manager/task_manager_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

namespace task_manager {

TaskManagerNode::TaskManagerNode() : Node("task_manager_node") {

  // ── Publishers ─────────────────────────────────────────────────────────
  phase_pub_    = create_publisher<std_msgs::msg::String>("/mission_phase", 10);
  task_cmd_pub_ = create_publisher<std_msgs::msg::String>("/task_command",  10);
  plow_cmd_pub_ = create_publisher<std_msgs::msg::String>("/plow_command",  10);
  cmd_vel_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",   10);

  // ── Subscriptions ──────────────────────────────────────────────────────
  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/start_signal", 10,
    std::bind(&TaskManagerNode::onStartSignal, this, std::placeholders::_1));
  task_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/task_status", 10,
    std::bind(&TaskManagerNode::onTaskStatus, this, std::placeholders::_1));
  plow_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/plow_state", 10,
    std::bind(&TaskManagerNode::onPlowState, this, std::placeholders::_1));
  verified_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/position_verified", 10,
    std::bind(&TaskManagerNode::onPositionVerified, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&TaskManagerNode::onOdom, this, std::placeholders::_1));

  // Subscribe to ToF distances for reverse approach
  tof_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/tof_distances", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
      latest_tof_ = *m;
    });

  // Subscribe to duck detections from duck_detector_node
  duck_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "/duck_detections", 10,
    std::bind(&TaskManagerNode::onDuckDetections, this, std::placeholders::_1));

  // ── Nav2 action client ─────────────────────────────────────────────────
  nav_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");

  // ── Mission queue ──────────────────────────────────────────────────────
  // Duck targets: name, approx_x, approx_y, blue_square_x, blue_square_y
  duck_queue_ = {
    {"duck_1", 0.25, 0.60, 0.65, 0.30},
    {"duck_2", 0.85, 0.60, 0.65, 0.30},
    {"duck_3", 0.25, 0.25, 0.65, 0.30},
    {"duck_4", 1.10, 0.30, 0.65, 0.30},
  };

  // Station targets: name, x, y, yaw, use_plow, task_command
  // ApproachMode::NORMAL   — Nav2 + EKF only
  // ApproachMode::REVERSE  — backs up using ToF sensors 0+1 (rear L+R)
  // ApproachMode::STRAFE   — strafes left using ToF sensor 2 (right side)
  station_queue_ = {
    {"antenna_1",    0.15, 0.75, 1.5708, true,  ApproachMode::NORMAL,  0.0,  -1, "PRESS_BUTTON"},
    {"antenna_2",    1.30, 0.75, 0.0,    true,  ApproachMode::STRAFE,  0.10,  2, "PRESS_BUTTON"},
    {"antenna_3",    0.65, 0.40, 0.0,    true,  ApproachMode::NORMAL,  0.0,  -1, "PRESS_BUTTON"},
    {"antenna_4",    0.65, 0.10, 1.5708, true,  ApproachMode::NORMAL,  0.0,  -1, "PRESS_BUTTON"},
    {"keypad_panel", 0.65, 0.80, 3.1416, false, ApproachMode::REVERSE, 0.12,  0, "PRESS_KEY 1 1"},
    {"crank",        0.10, 0.40, 0.0,    false, ApproachMode::NORMAL,  0.0,  -1, "TURN_CRANK 90"},
  };

  // ── Tick timer (10Hz) ──────────────────────────────────────────────────
  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TaskManagerNode::missionTick, this));

  RCLCPP_INFO(get_logger(), "Task manager ready — waiting for start signal");
}

// ── Callbacks ──────────────────────────────────────────────────────────────
void TaskManagerNode::onStartSignal(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && phase_ == MissionPhase::WAITING_FOR_START)
    transitionTo(MissionPhase::INIT);
}
void TaskManagerNode::onTaskStatus(const std_msgs::msg::String::SharedPtr msg) {
  last_task_result_ = msg->data;
  task_done_ = true;
}
void TaskManagerNode::onPlowState(const std_msgs::msg::String::SharedPtr msg) {
  last_plow_state_ = msg->data;
  if (msg->data == "DONE") plow_done_ = true;
}
void TaskManagerNode::onPositionVerified(const std_msgs::msg::Bool::SharedPtr msg) {
  position_verified_ = msg->data;
}
void TaskManagerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!home_saved_ && phase_ == MissionPhase::INIT) {
    home_pose_.header = msg->header;
    home_pose_.pose   = msg->pose.pose;
    home_saved_ = true;
    RCLCPP_INFO(get_logger(), "Home pose saved: (%.2f, %.2f)",
                home_pose_.pose.position.x, home_pose_.pose.position.y);
  }
}
void TaskManagerNode::onDuckDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  duck_visible_ = !msg->poses.empty();
  if (duck_visible_) {
    // Store pixel position of first (closest/largest) detected duck
    duck_pixel_x_ = msg->poses[0].position.x;
    duck_pixel_y_ = msg->poses[0].position.y;
  }
}

// ── Navigation ─────────────────────────────────────────────────────────────
void TaskManagerNode::sendNavGoal(double x, double y, double yaw,
  std::function<void(bool)> on_result)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
    nav_complete_ = true; nav_succeeded_ = false;
    if (on_result) on_result(false);
    return;
  }

  auto goal = NavToPose::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp    = now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  tf2::Quaternion q; q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  nav_complete_ = false; nav_succeeded_ = false;

  auto opts = rclcpp_action::Client<NavToPose>::SendGoalOptions();
  opts.goal_response_callback =
    [this](const GoalHandle::SharedPtr & h) {
      if (!h) { nav_complete_ = true; nav_succeeded_ = false; }
    };
  opts.result_callback =
    [this, on_result](const GoalHandle::WrappedResult & r) {
      nav_succeeded_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
      nav_complete_  = true;
      if (on_result) on_result(nav_succeeded_);
    };

  nav_client_->async_send_goal(goal, opts);
  RCLCPP_INFO(get_logger(), "Nav goal → (%.2f, %.2f, %.2frad)", x, y, yaw);
}

// ── Search behavior ─────────────────────────────────────────────────────────
void TaskManagerNode::startSearch(double center_x, double center_y) {
  searching_       = true;
  search_step_     = 0;
  search_center_x_ = center_x;
  search_center_y_ = center_y;
  search_start_time_ = now();
  RCLCPP_INFO(get_logger(), "Starting duck search around (%.2f, %.2f)", center_x, center_y);
}

void TaskManagerNode::tickSearch() {
  // Check timeout — give up and skip duck
  double elapsed = (now() - search_start_time_).seconds();
  if (elapsed > SEARCH_TIMEOUT_S) {
    RCLCPP_WARN(get_logger(), "Duck search timed out after %.1fs — skipping duck", elapsed);
    searching_ = false;
    current_duck_++;
    step_ = 0;
    return;
  }

  // Duck found — stop searching
  if (duck_visible_) {
    RCLCPP_INFO(get_logger(), "Duck found during search! Proceeding to push.");
    searching_  = false;
    nav_complete_ = true;  // trigger step 2 in executeDuckPush
    return;
  }

  // Navigate to next search waypoint
  if (nav_complete_ && search_step_ < NUM_SEARCH_POINTS) {
    double wx = search_center_x_ + SEARCH_OFFSETS[search_step_][0];
    double wy = search_center_y_ + SEARCH_OFFSETS[search_step_][1];
    RCLCPP_INFO(get_logger(), "Search waypoint %d/%d → (%.2f, %.2f)",
                search_step_ + 1, NUM_SEARCH_POINTS, wx, wy);
    sendNavGoal(wx, wy, 0.0);
    search_step_++;
  } else if (search_step_ >= NUM_SEARCH_POINTS && nav_complete_) {
    // Exhausted all waypoints — do a slow 360 rotation in place as last resort
    RCLCPP_INFO(get_logger(), "Search waypoints exhausted — rotating 360...");
    geometry_msgs::msg::Twist twist;
    twist.angular.z = 0.5;  // slow rotation
    cmd_vel_pub_->publish(twist);
  }
}

// ── Mission tick ────────────────────────────────────────────────────────────
void TaskManagerNode::missionTick() {
  auto msg = std_msgs::msg::String();
  msg.data = phaseToString(phase_);
  phase_pub_->publish(msg);

  switch (phase_) {

    case MissionPhase::INIT:
      if (home_saved_) {
        RCLCPP_INFO(get_logger(), "INIT complete — beginning duck push phase");
        step_ = 0;
        transitionTo(MissionPhase::PUSHING_DUCKS);
      }
      break;

    case MissionPhase::PUSHING_DUCKS:
      if (current_duck_ >= duck_queue_.size()) {
        RCLCPP_INFO(get_logger(), "All ducks pushed — starting arena tasks");
        step_ = 0;
        transitionTo(MissionPhase::PERFORMING_TASKS);
      } else {
        if (searching_) {
          tickSearch();
        } else {
          executeDuckPush(duck_queue_[current_duck_]);
        }
      }
      break;

    case MissionPhase::PERFORMING_TASKS:
      if (current_station_ >= station_queue_.size()) {
        RCLCPP_INFO(get_logger(), "All tasks done — returning home");
        step_ = 0;
        transitionTo(MissionPhase::RETURNING_HOME);
      } else {
        executeStation(station_queue_[current_station_]);
      }
      break;

    case MissionPhase::RETURNING_HOME:
      if (step_ == 0) {
        RCLCPP_INFO(get_logger(), "Navigating home...");
        sendNavGoal(home_pose_.pose.position.x, home_pose_.pose.position.y, 0.0);
        step_ = 1;
      } else if (step_ == 1 && nav_complete_) {
        transitionTo(MissionPhase::COMPLETE);
      }
      break;

    case MissionPhase::COMPLETE:
      RCLCPP_INFO(get_logger(), "Mission COMPLETE");
      tick_timer_->cancel();
      break;

    default: break;
  }
}

void TaskManagerNode::executeDuckPush(const DuckTarget & duck) {
  // Step 0: Navigate to approximate duck position
  // Step 1: Wait for nav — if duck visible go to step 3, else start search
  // Step 2: Searching — handled by tickSearch() in missionTick
  // Step 3: Duck confirmed visible — start plow push
  // Step 4: Wait for plow DONE
  // Step 5: Retreat, advance queue

  if (step_ == 0) {
    RCLCPP_INFO(get_logger(), "Navigating to duck: %s at (%.2f, %.2f)",
                duck.name.c_str(), duck.approx_x, duck.approx_y);
    duck_visible_ = false;
    sendNavGoal(duck.approx_x, duck.approx_y, 0.0);
    step_ = 1;

  } else if (step_ == 1 && nav_complete_) {
    if (duck_visible_) {
      // Duck spotted immediately — go straight to push
      RCLCPP_INFO(get_logger(), "Duck %s visible on arrival — pushing", duck.name.c_str());
      step_ = 3;
    } else {
      // Duck not visible — start search pattern
      RCLCPP_WARN(get_logger(), "Duck %s not visible at approx position — searching",
                  duck.name.c_str());
      startSearch(duck.approx_x, duck.approx_y);
      step_ = 2;
    }

  } else if (step_ == 2) {
    // Search is running via tickSearch() — wait until search finds duck
    // or times out (tickSearch handles both cases)
    if (!searching_) {
      // Search ended — either duck found or timed out
      // tickSearch already incremented current_duck_ on timeout
      if (duck_visible_) step_ = 3;
      // else: already skipped by tickSearch
    }

  } else if (step_ == 3) {
    // Duck confirmed — start push
    RCLCPP_INFO(get_logger(), "Starting push for duck %s", duck.name.c_str());
    plow_done_ = false;
    auto cmd = std_msgs::msg::String(); cmd.data = "START_PUSH";
    plow_cmd_pub_->publish(cmd);
    step_ = 4;

  } else if (step_ == 4 && plow_done_) {
    RCLCPP_INFO(get_logger(), "Duck %s pushed — retreating", duck.name.c_str());
    plow_done_ = false;
    auto cmd = std_msgs::msg::String(); cmd.data = "RETREAT";
    plow_cmd_pub_->publish(cmd);
    step_ = 5;

  } else if (step_ == 5 && plow_done_) {
    RCLCPP_INFO(get_logger(), "Duck %s complete", duck.name.c_str());
    current_duck_++;
    step_ = 0;
  }
}

void TaskManagerNode::executeStation(const ArenaStation & station) {
  // Step 0: Navigate to station position
  // Step 1: Wait for nav complete, decide approach mode
  // Step 2: REVERSE — back up using ToF sensors 0+1 until at target distance
  //         STRAFE  — strafe left using ToF sensor 2 until at target distance
  //         NORMAL  — skip to step 3
  // Step 3: Wait for position verified
  // Step 4: Execute task
  // Step 5: Wait for task done

  if (step_ == 0) {
    RCLCPP_INFO(get_logger(), "Navigating to station: %s", station.name.c_str());
    nav_complete_ = false;
    sendNavGoal(station.x, station.y, station.yaw);
    step_ = 1;

  } else if (step_ == 1 && nav_complete_) {
    if (!nav_succeeded_) {
      RCLCPP_WARN(get_logger(), "Nav failed to %s — skipping", station.name.c_str());
      current_station_++;
      step_ = 0;
      return;
    }
    if (station.approach == ApproachMode::REVERSE ||
        station.approach == ApproachMode::STRAFE) {
      RCLCPP_INFO(get_logger(), "At %s — beginning %s approach",
        station.name.c_str(),
        station.approach == ApproachMode::REVERSE ? "reverse" : "strafe");
      step_ = 2;
    } else {
      position_verified_ = false;
      step_ = 3;
    }

  } else if (step_ == 2) {
    if ((int)latest_tof_.data.size() <= station.approach_sensor) return;

    if (station.approach == ApproachMode::REVERSE) {
      // Back up using sensors 0 (left) + 1 (right) — stay square
      float l   = latest_tof_.data[0];
      float r   = latest_tof_.data[1];
      float avg = (l + r) / 2.0f;
      float diff = std::abs(l - r);

      if (avg <= station.approach_target_m + 0.010f && diff < 0.015f) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        RCLCPP_INFO(get_logger(),
          "Reverse done at %s: L=%.3fm R=%.3fm", station.name.c_str(), l, r);
        position_verified_ = false;
        step_ = 3;
      } else {
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = -0.05f;           // reverse
        twist.angular.z = (r - l) * 1.0f;   // correct to stay square
        cmd_vel_pub_->publish(twist);
      }

    } else if (station.approach == ApproachMode::STRAFE) {
      // Strafe left using sensor 2 (right side) until at target distance
      float side = latest_tof_.data[station.approach_sensor];

      if (side <= station.approach_target_m + 0.010f) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        RCLCPP_INFO(get_logger(),
          "Strafe done at %s: sensor2=%.3fm", station.name.c_str(), side);
        position_verified_ = false;
        step_ = 3;
      } else {
        geometry_msgs::msg::Twist twist;
        twist.linear.y = -0.05f;  // strafe left (negative y = left on mecanum)
        cmd_vel_pub_->publish(twist);
      }
    }

  } else if (step_ == 3 && position_verified_) {
    RCLCPP_INFO(get_logger(), "Position verified at %s — executing task",
                station.name.c_str());
    task_done_ = false; plow_done_ = false;
    auto cmd = std_msgs::msg::String();
    cmd.data = station.task_command;
    if (station.use_plow) {
      plow_cmd_pub_->publish(cmd);
    } else {
      task_cmd_pub_->publish(cmd);
    }
    step_ = 4;

  } else if (step_ == 4) {
    bool done = station.use_plow ? plow_done_ : task_done_;
    if (done) {
      RCLCPP_INFO(get_logger(), "Station %s complete", station.name.c_str());
      current_station_++;
      step_ = 0;
    }
  }
}

void TaskManagerNode::transitionTo(MissionPhase p) {
  phase_ = p;
  step_  = 0;
  RCLCPP_INFO(get_logger(), "Phase → %s", phaseToString(p).c_str());
}

std::string TaskManagerNode::phaseToString(MissionPhase p) {
  switch (p) {
    case MissionPhase::WAITING_FOR_START: return "WAITING_FOR_START";
    case MissionPhase::INIT:              return "INIT";
    case MissionPhase::PUSHING_DUCKS:     return "PUSHING_DUCKS";
    case MissionPhase::PERFORMING_TASKS:  return "PERFORMING_TASKS";
    case MissionPhase::RETURNING_HOME:    return "RETURNING_HOME";
    case MissionPhase::COMPLETE:          return "COMPLETE";
    default:                              return "UNKNOWN";
  }
}

} // namespace task_manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<task_manager::TaskManagerNode>());
  rclcpp::shutdown();
}
