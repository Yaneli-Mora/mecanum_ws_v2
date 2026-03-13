
#include "task_manager/task_manager_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace task_manager {

TaskManagerNode::TaskManagerNode() : Node("task_manager_node") {

  // ── Load arena positions from yaml ─────────────────────────────────────
  declare_parameter("arena_positions_yaml",
    ament_index_cpp::get_package_share_directory("task_manager")
    + "/config/arena_positions.yaml");
  std::string yaml_path = get_parameter("arena_positions_yaml").as_string();
  loadArenaPositions(yaml_path);

  // ── Publishers ─────────────────────────────────────────────────────────
  phase_pub_      = create_publisher<std_msgs::msg::String>("/mission_phase",  10);
  task_cmd_pub_   = create_publisher<std_msgs::msg::String>("/task_command",   10);
  plow_cmd_pub_   = create_publisher<std_msgs::msg::String>("/plow_command",   10);
  teensy_cmd_pub_ = create_publisher<std_msgs::msg::String>("/teensy_command", 10);
  cmd_vel_pub_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",    10);

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
  duck_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "/duck_detections", 10,
    std::bind(&TaskManagerNode::onDuckDetections, this, std::placeholders::_1));
  tof_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/tof_distances", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr m){ latest_tof_ = *m; });
  line_sub_ = create_subscription<std_msgs::msg::String>(
    "/line_state", 10,
    std::bind(&TaskManagerNode::onLineState, this, std::placeholders::_1));

  // ── Nav2 action client ─────────────────────────────────────────────────
  nav_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");

  // ══════════════════════════════════════════════════════════════════════
  //  MISSION QUEUE — ordered sequence of all tasks
  //  ⚠️ Update x/y coordinates after measuring physical field
  // ══════════════════════════════════════════════════════════════════════
  //
  //  Layout:
  //    antenna_1  = top-left    (button press)
  //    antenna_4  = bottom-mid  (keypad, reverse approach)
  //    antenna_2  = top-right   (crank, left side of robot)
  //    antenna_3  = crater loop (extend plank, clockwise, knock object)
  //
  //  Order:
  //    1. antenna_1 (top-left button — first task, right across from start)
  //    2. duck_1    (push to blue square)
  //    3. duck_2    (push to blue square)
  //    4. antenna_4 (keypad — reverse approach)
  //    5. duck_3    (top of crater)
  //    6. duck_4    (right of crater)
  //    7. antenna_2 (crank — strafe left side, ToF2 on left)
  //    8. duck_5    (lower side of crater)
  //    9. antenna_3 (crater loop — plank extends before loop)
  //   10. return home
  // ══════════════════════════════════════════════════════════════════════

  auto S  = MissionItemType::STATION;
  auto D  = MissionItemType::DUCK;
  auto C  = MissionItemType::CRATER_LOOP;
  auto IR = MissionItemType::TRANSMIT_IR;

  mission_queue_ = {

    // 1. Antenna_1 — top-left, reverse approach (robot backs into button 3x),
    //    plank B lifts during button press, read LED, then pivot to duck_1
    { S, {"antenna_1", 0.15, 0.90, 3.1416, true,
          ApproachMode::REVERSE, 0.08, 0, "PRESS_BUTTON", 1} },

    // 2. Duck_1
    { D, {}, {"duck_1", 0.30, 0.80, BLUE_X, BLUE_Y, DuckPushMode::NORMAL} },

    // 3. Duck_2
    { D, {}, {"duck_2", 0.30, 0.55, BLUE_X, BLUE_Y, DuckPushMode::NORMAL} },

    // 4. Antenna_4/Keypad — reverse approach, fire solenoids 7→3→7→3→8→#
    //    read LED after
    { S, {"antenna_4", 0.60, 0.15, 3.1416, false,
          ApproachMode::REVERSE, 0.12, 0, "PRESS_KEYPAD", 4} },

    // 5. Duck_3 — top of crater, U-turn push
    { D, {}, {"duck_3", CRATER_CX, CRATER_CY + CRATER_RADIUS + 0.15, BLUE_X, BLUE_Y,
              DuckPushMode::UTURN, 0.0, 0.0, 0.0, false} },

    // 6. Duck_4 — right of crater, U-turn push then reverse to crank
    { D, {}, {"duck_4", CRATER_CX + CRATER_RADIUS + 0.15, CRATER_CY, BLUE_X, BLUE_Y,
              DuckPushMode::UTURN_TO_STATION, 1.20, 0.90, 0.0} },

    // 7. Antenna_2/Crank — top-right corner, crank face points south
    //    Crank module on LEFT side of robot, ToF2 also on left side
    //    Robot yaw=0.0 (facing east/right wall), left side faces south → engages crank ✓
    //    Strafe in +y (toward top wall) until ToF2 reads 0.10m from crank face
    //    After crank: drive forward (east = +x) to clear → pivot right 90° (yaw=-1.5708)
    //    → rear faces north/top wall → reverse into top wall → READ_LED 2
    { S, {"antenna_2", 1.20, 0.90, 0.0, false,
          ApproachMode::STRAFE, 0.10, 2, "TURN_CRANK", 2} },

    // 8. Duck_5 — lower side of crater
    { D, {}, {"duck_5", CRATER_CX, CRATER_CY - CRATER_RADIUS - 0.15, BLUE_X, BLUE_Y,
              DuckPushMode::NORMAL} },

    // 9. Antenna_3 — crater loop (plank A extends, clockwise, knocks object off)
    //    read LED 3 after loop completes
    { C, {}, {}, {CRATER_CX, CRATER_CY, DRIVE_RADIUS, CRATER_WPS} },

    // 10. TRANSMIT_IR — send all 4 recorded antenna colors on way home
    { IR },
  };

  // ── Apply yaml overrides to mission queue ──────────────────────────────
  for (auto & item : mission_queue_) {
    if (item.type == MissionItemType::STATION) {
      auto it = station_overrides_.find(item.station.name);
      if (it != station_overrides_.end()) {
        item.station.x   = it->second.x;
        item.station.y   = it->second.y;
        item.station.yaw = it->second.yaw;
        RCLCPP_INFO(get_logger(), "Overrode station %s → (%.3f, %.3f, yaw=%.4f)",
          item.station.name.c_str(), item.station.x, item.station.y, item.station.yaw);
      }
    } else if (item.type == MissionItemType::DUCK) {
      for (const auto & p : duck_patches_) {
        if (p.name == item.duck.name) {
          item.duck.approx_x = p.x;
          item.duck.approx_y = p.y;
          RCLCPP_INFO(get_logger(), "Overrode duck %s → (%.3f, %.3f)",
            item.duck.name.c_str(), p.x, p.y);
          break;
        }
      }
      // Always update blue square from loaded values
      item.duck.blue_square_x = BLUE_X;
      item.duck.blue_square_y = BLUE_Y;
    } else if (item.type == MissionItemType::CRATER_LOOP) {
      item.crater.center_x      = CRATER_CX;
      item.crater.center_y      = CRATER_CY;
      item.crater.radius        = DRIVE_RADIUS;
      item.crater.num_waypoints = CRATER_WPS;
    }
  }

  // ── Tick timer 10Hz ────────────────────────────────────────────────────
  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TaskManagerNode::missionTick, this));

  RCLCPP_INFO(get_logger(), "Task manager ready — waiting for start signal");
}

// ── Callbacks ───────────────────────────────────────────────────────────────

void TaskManagerNode::onStartSignal(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && phase_ == MissionPhase::WAITING_FOR_START) {
    RCLCPP_INFO(get_logger(), "Start signal received — beginning mission");
    transitionTo(MissionPhase::RUNNING);
  }
}

void TaskManagerNode::onTaskStatus(const std_msgs::msg::String::SharedPtr msg) {
  last_task_result_ = msg->data;
  task_done_ = (msg->data == "DONE");
}

void TaskManagerNode::onPlowState(const std_msgs::msg::String::SharedPtr msg) {
  last_plow_state_ = msg->data;
  plow_done_ = (msg->data == "DONE");
}

void TaskManagerNode::onPositionVerified(const std_msgs::msg::Bool::SharedPtr msg) {
  position_verified_ = msg->data;
}

void TaskManagerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Track current position for U-turn distance check
  current_x_   = msg->pose.pose.position.x;
  current_y_   = msg->pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch;
  tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);

  if (!home_saved_) {
    home_pose_.header = msg->header;
    home_pose_.pose   = msg->pose.pose;
    home_saved_       = true;
    RCLCPP_INFO(get_logger(), "Home position saved: (%.3f, %.3f)",
      home_pose_.pose.position.x, home_pose_.pose.position.y);
  }
}

void TaskManagerNode::onDuckDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  duck_visible_ = !msg->poses.empty();
  if (duck_visible_) {
    duck_pixel_x_ = msg->poses[0].position.x;
    duck_pixel_y_ = msg->poses[0].position.y;
  }
}

void TaskManagerNode::onLineState(const std_msgs::msg::String::SharedPtr msg) {
  line_state_ = msg->data;
}

// ── State transitions ────────────────────────────────────────────────────────

void TaskManagerNode::transitionTo(MissionPhase phase) {
  phase_ = phase;
  step_  = 0;
  auto msg = std_msgs::msg::String();
  msg.data = [&]() -> std::string {
    switch (phase) {
      case MissionPhase::WAITING_FOR_START: return "WAITING";
      case MissionPhase::RUNNING:           return "RUNNING";
      case MissionPhase::RETURNING_HOME:    return "RETURNING_HOME";
      case MissionPhase::COMPLETE:          return "COMPLETE";
      default: return "UNKNOWN";
    }
  }();
  phase_pub_->publish(msg);
}

// ── Main tick ────────────────────────────────────────────────────────────────

void TaskManagerNode::missionTick() {
  if (phase_ == MissionPhase::WAITING_FOR_START) return;

  if (phase_ == MissionPhase::RUNNING) {
    if (current_item_ >= mission_queue_.size()) {
      RCLCPP_INFO(get_logger(), "All tasks complete — returning home");
      transitionTo(MissionPhase::RETURNING_HOME);
      sendNavGoal(
        home_pose_.pose.position.x,
        home_pose_.pose.position.y,
        0.0,
        [this](bool ok) {
          transitionTo(MissionPhase::COMPLETE);
          RCLCPP_INFO(get_logger(), "Mission complete!");
        });
      return;
    }

    const auto & item = mission_queue_[current_item_];
    switch (item.type) {
      case MissionItemType::STATION:    executeStation(item.station);   break;
      case MissionItemType::DUCK:       executeDuck(item.duck);         break;
      case MissionItemType::CRATER_LOOP:executeCraterLoop(item.crater); break;
      case MissionItemType::TRANSMIT_IR:
        // IR transmit position: same x/y as antenna_3 LED read (left edge of crater)
        // Yaw: facing north (1.5708) + 15° toward left wall = 1.5708 + 0.2618 = 1.8326
        // Plank B: 120°
        if (step_ == 0) {
          RCLCPP_INFO(get_logger(), "[IR] Navigating to transmit position");
          nav_complete_ = false;
          double ir_x   = IR_X;
          double ir_y   = IR_Y;
          double ir_yaw = IR_YAW;
          sendNavGoal(ir_x, ir_y, ir_yaw);
          step_ = 1;

        } else if (step_ == 1 && nav_complete_) {
          if (!nav_succeeded_)
            RCLCPP_WARN(get_logger(), "[IR] Nav failed — attempting transmit anyway");
          // Extend plank B to 120° for IR transmitter angle
          RCLCPP_INFO(get_logger(), "[IR] Extending plank B to 120° for transmit");
          auto ext = std_msgs::msg::String();
          ext.data = "EXTEND_PLANK_B_120";
          teensy_cmd_pub_->publish(ext);
          rclcpp::sleep_for(std::chrono::milliseconds(600));
          RCLCPP_INFO(get_logger(), "[IR] Transmitting antenna colors");
          task_done_ = false;
          auto cmd = std_msgs::msg::String();
          cmd.data  = "TRANSMIT_IR";
          task_cmd_pub_->publish(cmd);
          step_ = 2;

        } else if (step_ == 2 && task_done_) {
          auto ret = std_msgs::msg::String();
          ret.data = "RETRACT_PLANK_B";
          teensy_cmd_pub_->publish(ret);
          rclcpp::sleep_for(std::chrono::milliseconds(600));
          RCLCPP_INFO(get_logger(), "[IR] Transmit complete");
          current_item_++; step_ = 0;
        }
        break;
    }
  }
}

// ── Station executor ─────────────────────────────────────────────────────────

void TaskManagerNode::executeStation(const ArenaStation & s) {
  if (step_ == 0) {
    RCLCPP_INFO(get_logger(), "[%s] Navigating", s.name.c_str());
    nav_complete_ = false;
    sendNavGoal(s.x, s.y, s.yaw);
    step_ = 1;

  } else if (step_ == 1 && nav_complete_) {
    if (!nav_succeeded_) {
      RCLCPP_WARN(get_logger(), "[%s] Nav failed — skipping", s.name.c_str());
      current_item_++; step_ = 0; return;
    }
    if (s.approach == ApproachMode::REVERSE ||
        s.approach == ApproachMode::STRAFE) {
      step_ = 2;
    } else {
      position_verified_ = false;
      step_ = 3;
    }

  } else if (step_ == 2) {
    if ((int)latest_tof_.data.size() <= s.approach_sensor) return;

    if (s.approach == ApproachMode::REVERSE) {
      float l = latest_tof_.data[0], r = latest_tof_.data[1];
      float avg = (l + r) / 2.0f;
      if (avg <= s.approach_target_m + 0.010f && std::abs(l-r) < 0.015f) {
        geometry_msgs::msg::Twist stop; cmd_vel_pub_->publish(stop);
        RCLCPP_INFO(get_logger(), "[%s] Reverse done L=%.3f R=%.3f", s.name.c_str(), l, r);
        position_verified_ = false; step_ = 3;
      } else {
        geometry_msgs::msg::Twist t;
        t.linear.x  = -0.05f;
        t.angular.z = (r - l) * 1.0f;
        cmd_vel_pub_->publish(t);
      }

    } else if (s.approach == ApproachMode::STRAFE) {
      float side = latest_tof_.data[s.approach_sensor];
      if (side <= s.approach_target_m + 0.010f) {
        geometry_msgs::msg::Twist stop; cmd_vel_pub_->publish(stop);
        RCLCPP_INFO(get_logger(), "[%s] Strafe done side=%.3f", s.name.c_str(), side);
        position_verified_ = false; step_ = 3;
      } else {
        geometry_msgs::msg::Twist t;
        t.linear.y = +0.05f;   // strafe toward top wall (left side toward crank)
        cmd_vel_pub_->publish(t);
      }
    }

  } else if (step_ == 3 && position_verified_) {
    RCLCPP_INFO(get_logger(), "[%s] Verified — executing task", s.name.c_str());
    task_done_ = false; plow_done_ = false;

    if (s.task_command.empty()) {
      task_done_ = true;  // nothing to do, advance
    } else {
      // For antenna_2: start lifting plank B DURING crank turn (~3s) for time efficiency
      if (s.name == "antenna_2") {
        auto ext = std_msgs::msg::String();
        ext.data = "EXTEND_PLANK_B_150";
        teensy_cmd_pub_->publish(ext);
        RCLCPP_INFO(get_logger(), "[antenna_2] Lifting plank B during crank");
      }
      auto cmd = std_msgs::msg::String();
      cmd.data = s.task_command;
      if (s.use_plow) plow_cmd_pub_->publish(cmd);
      else            task_cmd_pub_->publish(cmd);
    }
    step_ = 4;

  } else if (step_ == 4) {
    if (s.use_plow ? plow_done_ : task_done_) {
      RCLCPP_INFO(get_logger(), "[%s] Task done", s.name.c_str());
      if (s.read_led_antenna > 0) {
        if (s.name == "antenna_1") {
          // Button press done — NOW extend plank B and read LED
          auto ext = std_msgs::msg::String();
          ext.data = "EXTEND_PLANK_B_90";
          teensy_cmd_pub_->publish(ext);
          rclcpp::sleep_for(std::chrono::milliseconds(600));
          RCLCPP_INFO(get_logger(), "[antenna_1] Plank B raised — reading LED");
          task_done_ = false;
          auto cmd = std_msgs::msg::String();
          cmd.data  = "READ_LED 1";
          task_cmd_pub_->publish(cmd);
          step_ = 5;
        } else if (s.name == "antenna_2") {
          // Crank done, plank B already raised during crank
          // Robot currently yaw=0.0 (facing east/right wall)
          // Drive forward (east = +x) ~0.15m to clear crank before pivoting
          RCLCPP_INFO(get_logger(), "[antenna_2] Crank done — driving east to clear crank");
          nav_complete_ = false;
          sendNavGoal(s.x + 0.15, s.y, 0.0);
          step_ = 7;
        } else {
          // All other antennas — extend plank B now
          const char* plank_cmd =
            (s.read_led_antenna == 2) ? "EXTEND_PLANK_B_150" : "EXTEND_PLANK_B_90";
          auto ext = std_msgs::msg::String();
          ext.data = plank_cmd;
          teensy_cmd_pub_->publish(ext);
          rclcpp::sleep_for(std::chrono::milliseconds(600));
          task_done_ = false;
          auto cmd = std_msgs::msg::String();
          cmd.data  = "READ_LED " + std::to_string(s.read_led_antenna);
          task_cmd_pub_->publish(cmd);
          step_ = 5;
        }
      } else {
        RCLCPP_INFO(get_logger(), "[%s] Complete", s.name.c_str());
        current_item_++; step_ = 0;
      }
    }

  } else if (step_ == 5 && task_done_) {
    // LED read complete — retract plank B
    auto ret = std_msgs::msg::String();
    ret.data = "RETRACT_PLANK_B";
    teensy_cmd_pub_->publish(ret);
    rclcpp::sleep_for(std::chrono::milliseconds(600));

    // For antenna_1 — pivot to face duck_1 before advancing
    if (s.name == "antenna_1") {
      RCLCPP_INFO(get_logger(), "[antenna_1] Pivoting to face duck_1");
      nav_complete_ = false;
      sendNavGoal(s.x, s.y, 1.5708);  // face up toward field
      step_ = 6;
      return;
    }
    RCLCPP_INFO(get_logger(), "[%s] LED read complete", s.name.c_str());
    current_item_++; step_ = 0;

  } else if (step_ == 6 && nav_complete_) {
    // antenna_1 only — pivot complete, advance to duck_1
    RCLCPP_INFO(get_logger(), "[antenna_1] Pivot complete — advancing to duck_1");
    current_item_++; step_ = 0;

  // ── antenna_2 reposition steps ────────────────────────────────────────────
  // step 7: forward clear (east) done → pivot RIGHT 90°
  //         yaw=0.0 → turn right 90° → yaw=-1.5708 (facing south)
  //         rear of robot now faces north/top wall ✓
  } else if (step_ == 7 && nav_complete_) {
    if (!nav_succeeded_)
      RCLCPP_WARN(get_logger(), "[antenna_2] Clear nav failed — attempting pivot anyway");
    RCLCPP_INFO(get_logger(), "[antenna_2] Cleared crank — pivoting right 90° to yaw=-1.5708");
    nav_complete_ = false;
    sendNavGoal(s.x + 0.15, s.y, -1.5708);  // same position, rear now faces top wall
    step_ = 8;

  // step 8: pivot done → enter ToF reverse loop toward top wall
  } else if (step_ == 8 && nav_complete_) {
    if (!nav_succeeded_)
      RCLCPP_WARN(get_logger(), "[antenna_2] Pivot nav failed — attempting reverse anyway");
    RCLCPP_INFO(get_logger(), "[antenna_2] Pivoted — reversing into top wall for LED");
    step_ = 9;

  } else if (step_ == 9) {
    // ToF-guided reverse toward top wall
    // Robot facing south (yaw=-1.5708), reverse = +y = toward top wall ✓
    if ((int)latest_tof_.data.size() < 2) return;
    float l = latest_tof_.data[0], r = latest_tof_.data[1];
    float avg = (l + r) / 2.0f;
    static constexpr float ANT2_LED_TARGET = 0.05f;  // ⚠️ tune on field
    if (avg <= ANT2_LED_TARGET + 0.010f && std::abs(l - r) < 0.015f) {
      geometry_msgs::msg::Twist stop; cmd_vel_pub_->publish(stop);
      RCLCPP_INFO(get_logger(), "[antenna_2] At LED position L=%.3f R=%.3f — reading", l, r);
      task_done_ = false;
      auto cmd = std_msgs::msg::String();
      cmd.data  = "READ_LED 2";
      task_cmd_pub_->publish(cmd);
      step_ = 10;
    } else {
      geometry_msgs::msg::Twist t;
      t.linear.x  = -0.05f;          // reverse = toward top wall when facing south
      t.angular.z = (r - l) * 1.0f;  // square up
      cmd_vel_pub_->publish(t);
    }
    } else {
      geometry_msgs::msg::Twist t;
      t.linear.x  = -0.05f;              // reverse (toward top wall, robot facing down)
      t.angular.z = (r - l) * 1.0f;     // square up
      cmd_vel_pub_->publish(t);
    }

  } else if (step_ == 10 && task_done_) {
    RCLCPP_INFO(get_logger(), "[antenna_2] LED read complete — retracting and clearing");
    auto ret = std_msgs::msg::String();
    ret.data = "RETRACT_PLANK_B";
    teensy_cmd_pub_->publish(ret);
    rclcpp::sleep_for(std::chrono::milliseconds(600));
    nav_complete_ = false;
    // Drive south (away from top wall) to clear, ready for duck_5 approach
    // ⚠️ tune x/y on field
    sendNavGoal(s.x + 0.15, s.y - 0.20, 0.0);
    step_ = 11;

  } else if (step_ == 11 && nav_complete_) {
    RCLCPP_INFO(get_logger(), "[antenna_2] Complete — advancing to duck_5");
    current_item_++; step_ = 0;
  }
}

// ── Duck executor ────────────────────────────────────────────────────────────
//
// Steps for NORMAL push:
//   0 → nav to approx   1 → search if needed   2 → search tick
//   3 → plow down       4 → nav to blue square  5 → stop on COLORED
//   6 → plow up         7 → retreat             8 → done
//
// Steps for UTURN / UTURN_TO_STATION push:
//   0 → nav to approx   1 → search if needed   2 → search tick
//   3 → plow down       4 → push forward 0.559m (cmd_vel, check odometry)
//   5 → stop, spin 180° in place (nav goal at same pos, opposite yaw)
//   6 → wait for spin   7 → nav to blue square  8 → stop on COLORED
//   9 → plow up        10 → retreat (approx or after_x/y for UTURN_TO_STATION)
//  11 → done

void TaskManagerNode::executeDuck(const DuckTarget & d) {
  bool is_uturn = (d.push_mode == DuckPushMode::UTURN ||
                   d.push_mode == DuckPushMode::UTURN_TO_STATION);

  // ── Step 0: navigate to approximate duck position ────────────────────
  if (step_ == 0) {
    RCLCPP_INFO(get_logger(), "[%s] Navigating to approx (%.3f, %.3f)",
      d.name.c_str(), d.approx_x, d.approx_y);
    nav_complete_ = false;
    duck_visible_ = false;
    sendNavGoal(d.approx_x, d.approx_y, 0.0);
    step_ = 1;

  // ── Step 1: arrived — check if duck visible ──────────────────────────
  } else if (step_ == 1 && nav_complete_) {
    if (duck_visible_) {
      step_ = 3;
    } else {
      RCLCPP_INFO(get_logger(), "[%s] Duck not visible — searching", d.name.c_str());
      startSearch(d.approx_x, d.approx_y);
      step_ = 2;
    }

  // ── Step 2: search behavior ──────────────────────────────────────────
  } else if (step_ == 2) {
    if (duck_visible_) {
      searching_ = false;
      step_ = 3;
    } else {
      tickSearch();
      if ((this->now() - search_start_time_).seconds() > SEARCH_TIMEOUT_S) {
        RCLCPP_WARN(get_logger(), "[%s] Search timeout — skipping", d.name.c_str());
        searching_ = false;
        current_item_++; step_ = 0;
      }
    }

  // ── Step 3: duck found — drive forward into duck (plow is fixed) ────
  } else if (step_ == 3) {
    RCLCPP_INFO(get_logger(), "[%s] Duck found — pushing with fixed plow", d.name.c_str());
    uturn_start_x_ = current_x_;
    uturn_start_y_ = current_y_;
    step_ = 4;

  // ── Step 4: push forward ─────────────────────────────────────────────
  } else if (step_ == 4) {
    if (is_uturn) {
      // Drive forward until we've pushed 0.559m (22 inches)
      double dist = std::hypot(current_x_ - uturn_start_x_,
                               current_y_ - uturn_start_y_);
      if (dist >= UTURN_PUSH_DIST_M) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        RCLCPP_INFO(get_logger(), "[%s] Pushed %.3fm — spinning 180°",
          d.name.c_str(), dist);
        step_ = 5;
      } else {
        // Drive straight forward slowly with duck pinned on plow
        geometry_msgs::msg::Twist t;
        t.linear.x = 0.15f;  // 15cm/s forward push
        cmd_vel_pub_->publish(t);
      }
    } else {
      // NORMAL — go straight to blue square nav
      step_ = 7;
    }

  // ── Step 5 (U-turn only): spin 180° in place around duck ─────────────
  } else if (step_ == 5) {
    // Send Nav2 goal at same position but flipped yaw
    // Duck stays pinned as plow sweeps around it
    double flip_yaw = current_yaw_ + M_PI;
    RCLCPP_INFO(get_logger(), "[%s] Pivoting to yaw=%.3f", d.name.c_str(), flip_yaw);
    nav_complete_ = false;
    // Navigate to current position with flipped yaw — robot rotates in place
    sendNavGoal(current_x_, current_y_, flip_yaw);
    step_ = 6;

  // ── Step 6 (U-turn only): wait for spin complete ─────────────────────
  } else if (step_ == 6 && nav_complete_) {
    if (!nav_succeeded_) {
      RCLCPP_WARN(get_logger(), "[%s] Spin failed — continuing anyway", d.name.c_str());
    }
    RCLCPP_INFO(get_logger(), "[%s] Spin done — pushing to blue square", d.name.c_str());
    step_ = 7;

  // ── Step 7: navigate to blue square pushing duck ──────────────────────
  } else if (step_ == 7) {
    RCLCPP_INFO(get_logger(), "[%s] Pushing to blue (%.3f, %.3f)",
      d.name.c_str(), d.blue_square_x, d.blue_square_y);
    nav_complete_ = false;
    sendNavGoal(d.blue_square_x, d.blue_square_y, 0.0);
    step_ = 8;

  // ── Step 8: stop when TCRT5000 detects colored/blue square ───────────
  } else if (step_ == 8) {
    if (line_state_ == "COLORED" || line_state_ == "WHITE") {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      RCLCPP_INFO(get_logger(), "[%s] Blue square detected — duck dropped", d.name.c_str());
      step_ = 9;
    }
    if (nav_complete_ && step_ == 8) step_ = 9;  // fallback if TCRT misses

  // ── Step 9: retreat or skip straight to next item ────────────────────
  } else if (step_ == 9) {
    if (d.skip_retreat) {
      // Skip retreat — advance mission queue immediately
      // Nav2 will handle getting to next item's approx position
      RCLCPP_INFO(get_logger(), "[%s] Skipping retreat — advancing to next item",
        d.name.c_str());
      current_item_++; step_ = 0;
      return;
    }

    // Decide retreat destination
    double rx, ry, ryaw;
    if (d.push_mode == DuckPushMode::UTURN_TO_STATION) {
      rx   = d.after_x;
      ry   = d.after_y;
      ryaw = d.after_yaw;
      RCLCPP_INFO(get_logger(), "[%s] Reversing to station (%.3f, %.3f)",
        d.name.c_str(), rx, ry);
    } else {
      rx   = d.approx_x;
      ry   = d.approx_y;
      ryaw = 0.0;
      RCLCPP_INFO(get_logger(), "[%s] Reversing to approx (%.3f, %.3f)",
        d.name.c_str(), rx, ry);
    }
    nav_complete_ = false;
    sendNavGoal(rx, ry, ryaw);
    step_ = 10;

  // ── Step 10: wait for retreat nav complete ────────────────────────────
  } else if (step_ == 10 && nav_complete_) {
    RCLCPP_INFO(get_logger(), "[%s] Complete", d.name.c_str());
    current_item_++; step_ = 0;
  }
}

// ── Crater loop executor ─────────────────────────────────────────────────────
// Drives clockwise around crater perimeter with plank extended
// Waypoints generated as evenly spaced points on a circle,
// starting from bottom (6 o'clock) going clockwise

void TaskManagerNode::executeCraterLoop(const CraterLoop & c) {

  if (step_ == 0) {
    // Extend plank before starting loop
    RCLCPP_INFO(get_logger(), "[CRATER] Extending plank");
    auto cmd = std_msgs::msg::String();
    cmd.data = "EXTEND_PLANK_A";
    teensy_cmd_pub_->publish(cmd);
    plank_extended_ = true;
    crater_wp_      = 0;
    step_ = 1;

  } else if (step_ == 1) {
    // Drive to each clockwise waypoint in sequence
    if (crater_wp_ >= c.num_waypoints) {
      // Completed full loop — retract plank
      RCLCPP_INFO(get_logger(), "[CRATER] Loop complete — retracting plank");
      auto cmd = std_msgs::msg::String();
      cmd.data = "RETRACT_PLANK_A";
      teensy_cmd_pub_->publish(cmd);
      plank_extended_ = false;
      step_ = 2;
      return;
    }

    if (!nav_complete_) return;  // wait for previous waypoint

    // Generate clockwise waypoint:
    // Start at bottom (angle = -PI/2), go clockwise (decreasing angle)
    double angle = (-M_PI / 2.0) - (2.0 * M_PI * crater_wp_ / c.num_waypoints);
    double wx    = c.center_x + c.radius * std::cos(angle);
    double wy    = c.center_y + c.radius * std::sin(angle);

    // Yaw: face tangent to circle (clockwise = yaw points right of radius)
    double tangent_yaw = angle - M_PI / 2.0;

    RCLCPP_INFO(get_logger(), "[CRATER] Waypoint %d/%d (%.3f, %.3f)",
      crater_wp_ + 1, c.num_waypoints, wx, wy);

    nav_complete_ = false;
    sendNavGoal(wx, wy, tangent_yaw);
    crater_wp_++;

  } else if (step_ == 2) {
    // Loop complete — nav to antenna_3 LED read position:
    // Left edge of crater, facing left/west wall
    RCLCPP_INFO(get_logger(), "[CRATER] Navigating to antenna_3 LED read position");
    nav_complete_ = false;
    double read_x = c.center_x - c.radius;   // left edge of crater
    double read_y = c.center_y;               // same y as crater centre
    sendNavGoal(read_x, read_y, M_PI);        // yaw=π = facing left/west wall
    step_ = 3;

  } else if (step_ == 3 && nav_complete_) {
    if (!nav_succeeded_)
      RCLCPP_WARN(get_logger(), "[CRATER] LED read nav failed — attempting read anyway");
    // Antenna_3 is shorter — use 150° for plank B
    auto ext = std_msgs::msg::String();
    ext.data = "EXTEND_PLANK_B_150";
    teensy_cmd_pub_->publish(ext);
    rclcpp::sleep_for(std::chrono::milliseconds(600));
    RCLCPP_INFO(get_logger(), "[CRATER] Reading antenna_3 LED");
    task_done_ = false;
    auto cmd = std_msgs::msg::String();
    cmd.data  = "READ_LED 3";
    task_cmd_pub_->publish(cmd);
    step_ = 4;

  } else if (step_ == 4 && task_done_) {
    auto ret = std_msgs::msg::String();
    ret.data = "RETRACT_PLANK_B";
    teensy_cmd_pub_->publish(ret);
    rclcpp::sleep_for(std::chrono::milliseconds(600));
    RCLCPP_INFO(get_logger(), "[CRATER] Antenna_3 complete");
    current_item_++; step_ = 0;
  }
}

// ── Search helpers ───────────────────────────────────────────────────────────

void TaskManagerNode::startSearch(double cx, double cy) {
  searching_       = true;
  search_step_     = 0;
  search_center_x_ = cx;
  search_center_y_ = cy;
  search_start_time_ = this->now();
  nav_complete_    = true;  // trigger first waypoint immediately
}

void TaskManagerNode::tickSearch() {
  if (!nav_complete_ || search_step_ >= NUM_SEARCH_POINTS) return;
  double wx = search_center_x_ + SEARCH_OFFSETS[search_step_][0];
  double wy = search_center_y_ + SEARCH_OFFSETS[search_step_][1];
  nav_complete_ = false;
  sendNavGoal(wx, wy, 0.0);
  search_step_++;
}

// ── Nav goal helper ──────────────────────────────────────────────────────────

void TaskManagerNode::sendNavGoal(double x, double y, double yaw,
                                   std::function<void(bool)> cb) {
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "Nav2 not available");
    return;
  }

  auto goal = NavToPose::Goal();
  goal.pose.header.frame_id    = "map";
  goal.pose.header.stamp       = this->now().to_msg();
  goal.pose.pose.position.x    = x;
  goal.pose.pose.position.y    = y;
  goal.pose.pose.position.z    = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation   = tf2::toMsg(q);

  auto opts = rclcpp_action::Client<NavToPose>::SendGoalOptions();
  opts.result_callback = [this, cb](const GoalHandle::WrappedResult & result) {
    nav_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    nav_complete_  = true;
    if (cb) cb(nav_succeeded_);
  };

  nav_client_->async_send_goal(goal, opts);
}

void TaskManagerNode::sendTeensyCmd(const std::string & cmd) {
  auto msg = std_msgs::msg::String();
  msg.data = cmd;
  teensy_cmd_pub_->publish(msg);
}

// ── Load arena positions from yaml ─────────────────────────────────────────
void TaskManagerNode::loadArenaPositions(const std::string & yaml_path) {
  RCLCPP_INFO(get_logger(), "Loading arena positions from: %s", yaml_path.c_str());

  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(),
      "Failed to load arena_positions.yaml: %s — using defaults", e.what());
    return;
  }

  // ── Crater ───────────────────────────────────────────────────────────────
  auto crater = cfg["crater"];
  if (crater) {
    CRATER_CX     = crater["center_x"].as<double>(CRATER_CX);
    CRATER_CY     = crater["center_y"].as<double>(CRATER_CY);
    CRATER_RADIUS = crater["radius"].as<double>(CRATER_RADIUS);
    DRIVE_RADIUS  = crater["drive_radius"].as<double>(DRIVE_RADIUS);
    CRATER_WPS    = crater["waypoints"].as<int>(CRATER_WPS);
  }

  // ── Blue square ──────────────────────────────────────────────────────────
  auto blue = cfg["blue_square"];
  if (blue) {
    BLUE_X = blue["x"].as<double>(BLUE_X);
    BLUE_Y = blue["y"].as<double>(BLUE_Y);
  }

  // ── IR transmit position ─────────────────────────────────────────────────
  auto ir = cfg["ir_transmit"];
  if (ir) {
    IR_X   = ir["x"].as<double>(IR_X);
    IR_Y   = ir["y"].as<double>(IR_Y);
    IR_YAW = ir["yaw"].as<double>(IR_YAW);
  }

  // ── Duck positions ───────────────────────────────────────────────────────
  // Loaded directly into mission queue after buildMissionQueue() runs,
  // so we re-patch duck approx positions here using named lookup
  auto ducks = cfg["duck_positions"];
  if (ducks) {
    struct DuckPatch { std::string name; double x; double y; };
    std::vector<DuckPatch> patches;
    for (auto it = ducks.begin(); it != ducks.end(); ++it) {
      patches.push_back({
        it->first.as<std::string>(),
        it->second["approx_x"].as<double>(),
        it->second["approx_y"].as<double>()
      });
    }
    // Store for patching after mission queue is built
    duck_patches_ = patches;
  }

  // ── Station positions ─────────────────────────────────────────────────────
  auto stations = cfg["arena_stations"];
  if (stations) {
    for (auto it = stations.begin(); it != stations.end(); ++it) {
      std::string n = it->first.as<std::string>();
      station_overrides_[n] = {
        it->second["x"].as<double>(),
        it->second["y"].as<double>(),
        it->second["yaw"].as<double>()
      };
    }
  }

  RCLCPP_INFO(get_logger(),
    "Arena loaded — crater=(%.3f,%.3f) r=%.3f blue=(%.3f,%.3f)",
    CRATER_CX, CRATER_CY, CRATER_RADIUS, BLUE_X, BLUE_Y);
}

} // namespace task_manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<task_manager::TaskManagerNode>());
  rclcpp::shutdown();
}
