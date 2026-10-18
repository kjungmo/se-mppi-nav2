// Copyright (c) 2026 Jungmo Kang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Headless closed-loop rig for the U-pocket tests: the real MPPI optimizer and
// the SafeEscapeController on a lifecycle node, a unicycle robot, a 3 m x 3 m
// rolling local costmap drawn from a development geometry, and a stand-in for
// the behaviour tree that replans a straight line to the goal every second.
// No Gazebo.

#ifndef DEPLOYMENT__UTRAP_RIG_HPP_
#define DEPLOYMENT__UTRAP_RIG_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_se_controller/no_valid_control.hpp"
#include "nav2_se_controller/safe_escape_controller.hpp"
#include "utrap_dev_geometries.hpp"

namespace utrap
{

constexpr double kDt = 0.1;           // control period (s)
constexpr double kReplanPeriod = 1.0;  // BT replanning period (s)
constexpr int kWindowCells = 60;       // 3 m local costmap, as in the live stack
constexpr double kRes = 0.05;

// Draw the rolling window centred on (cx, cy) from the geometry into `cm`.
inline void drawWindow(
  nav2_costmap_2d::Costmap2D & cm, const std::vector<Segment> & ws, double cx, double cy)
{
  const double ox = std::floor((cx - 0.5 * kWindowCells * kRes) / kRes + 0.5) * kRes;
  const double oy = std::floor((cy - 0.5 * kWindowCells * kRes) / kRes + 0.5) * kRes;
  cm = nav2_costmap_2d::Costmap2D(kWindowCells, kWindowCells, kRes, ox, oy, 0);
  for (int j = 0; j < kWindowCells; ++j) {
    for (int i = 0; i < kWindowCells; ++i) {
      const double x = ox + (i + 0.5) * kRes;
      const double y = oy + (j + 0.5) * kRes;
      const unsigned char c = cellCost(surfaceDistance(ws, x, y));
      if (c != 0) {
        cm.setCost(i, j, c);
      }
    }
  }
}

struct RunResult
{
  std::string geometry;
  bool entered_pocket{false};
  double enter_time{-1.0};
  bool left_pocket{false};
  double exit_time{-1.0};
  bool reached{false};
  double goal_time{-1.0};
  int contacts{0};
  int throws{0};
  int enters{0};
  int hops{0};
  double sim_time{0.0};
  double min_goal_dist{1e9};
  std::vector<std::array<double, 3>> trace;  // t, x, y every control cycle
};

template<typename Controller>
class Rig
{
public:
  explicit Rig(const std::vector<rclcpp::Parameter> & extra = {})
  {
    verbose_ = std::getenv("SE_UTRAP_VERBOSE") != nullptr;
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("controller_frequency", 10.0),
      rclcpp::Parameter("FollowPath.time_steps", 28),
      rclcpp::Parameter("FollowPath.model_dt", 0.1),
      rclcpp::Parameter("FollowPath.batch_size", 600),
      rclcpp::Parameter("FollowPath.ax_max", 3.0),
      rclcpp::Parameter("FollowPath.ax_min", -3.0),
      rclcpp::Parameter("FollowPath.ay_max", 3.0),
      rclcpp::Parameter("FollowPath.ay_min", -3.0),
      rclcpp::Parameter("FollowPath.az_max", 3.5),
      rclcpp::Parameter("FollowPath.vx_std", 0.2),
      rclcpp::Parameter("FollowPath.vy_std", 0.2),
      rclcpp::Parameter("FollowPath.wz_std", 0.4),
      rclcpp::Parameter("FollowPath.vx_max", 0.5),
      rclcpp::Parameter("FollowPath.vx_min", -0.35),
      rclcpp::Parameter("FollowPath.vy_max", 0.5),
      rclcpp::Parameter("FollowPath.wz_max", 1.9),
      rclcpp::Parameter("FollowPath.iteration_count", 1),
      rclcpp::Parameter("FollowPath.prune_distance", 1.7),
      rclcpp::Parameter("FollowPath.temperature", 0.3),
      rclcpp::Parameter("FollowPath.gamma", 0.015),
      rclcpp::Parameter("FollowPath.motion_model", std::string("DiffDrive")),
      rclcpp::Parameter("FollowPath.regenerate_noises", true),
      rclcpp::Parameter(
        "FollowPath.critics", std::vector<std::string>{
          "ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
          "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
          "PreferForwardCritic", "EscapeCritic"}),
      rclcpp::Parameter("FollowPath.ConstraintCritic.cost_weight", 4.0),
      rclcpp::Parameter("FollowPath.GoalCritic.cost_weight", 5.0),
      rclcpp::Parameter("FollowPath.GoalCritic.threshold_to_consider", 1.4),
      rclcpp::Parameter("FollowPath.GoalAngleCritic.cost_weight", 3.0),
      rclcpp::Parameter("FollowPath.GoalAngleCritic.threshold_to_consider", 2.0),
      rclcpp::Parameter("FollowPath.PreferForwardCritic.cost_weight", 5.0),
      rclcpp::Parameter("FollowPath.PreferForwardCritic.threshold_to_consider", 0.5),
      rclcpp::Parameter("FollowPath.CostCritic.cost_weight", 3.81),
      rclcpp::Parameter("FollowPath.CostCritic.near_collision_cost", 253),
      rclcpp::Parameter("FollowPath.CostCritic.critical_cost", 300.0),
      rclcpp::Parameter("FollowPath.CostCritic.consider_footprint", false),
      rclcpp::Parameter("FollowPath.CostCritic.collision_cost", 1000000.0),
      rclcpp::Parameter("FollowPath.CostCritic.near_goal_distance", 1.0),
      rclcpp::Parameter("FollowPath.CostCritic.trajectory_point_step", 2),
      rclcpp::Parameter("FollowPath.PathAlignCritic.cost_weight", 14.0),
      rclcpp::Parameter("FollowPath.PathAlignCritic.max_path_occupancy_ratio", 0.05),
      rclcpp::Parameter("FollowPath.PathAlignCritic.trajectory_point_step", 4),
      rclcpp::Parameter("FollowPath.PathAlignCritic.threshold_to_consider", 0.5),
      rclcpp::Parameter("FollowPath.PathAlignCritic.offset_from_furthest", 20),
      rclcpp::Parameter("FollowPath.PathAlignCritic.use_path_orientations", false),
      rclcpp::Parameter("FollowPath.PathFollowCritic.cost_weight", 5.0),
      rclcpp::Parameter("FollowPath.PathFollowCritic.offset_from_furthest", 5),
      rclcpp::Parameter("FollowPath.PathFollowCritic.threshold_to_consider", 1.4),
      rclcpp::Parameter("FollowPath.PathAngleCritic.cost_weight", 2.0),
      rclcpp::Parameter("FollowPath.PathAngleCritic.offset_from_furthest", 4),
      rclcpp::Parameter("FollowPath.PathAngleCritic.threshold_to_consider", 2.0),
      rclcpp::Parameter("FollowPath.PathAngleCritic.max_angle_to_furthest", 1.0),
      rclcpp::Parameter("FollowPath.PathAngleCritic.mode", 0),
      rclcpp::Parameter("FollowPath.EscapeCritic.enabled", true),
      rclcpp::Parameter("FollowPath.EscapeCritic.use_apf", true),
      rclcpp::Parameter("FollowPath.EscapeCritic.repulsion_weight", 2.0),
      rclcpp::Parameter("FollowPath.EscapeCritic.apf_influence_dist", 0.6),
      rclcpp::Parameter("FollowPath.EscapeCritic.progress_stall_window", 30),
    };
    params.insert(params.end(), extra.begin(), extra.end());
    // Ablation aid, never set in CI:
    //   SE_UTRAP_PARAMS="se_escape_avoid_radius=0.0,se_escape_avoid_hop_radius=0.0,se_escape_stall_time=1e6"
    // sets FollowPath parameters (an integer-looking value is an integer, anything else a double,
    // so write 0.0 for a double parameter).
    // The review of 2026-10-18 used it to switch the additions beyond the advised design off.
    if (const char * env = std::getenv("SE_UTRAP_PARAMS")) {
      std::stringstream list(env);
      std::string item;
      while (std::getline(list, item, ',')) {
        const auto eq = item.find('=');
        if (eq == std::string::npos) {
          continue;
        }
        const std::string name = "FollowPath." + item.substr(0, eq);
        const std::string value = item.substr(eq + 1);
        if (value.find_first_of(".eE") == std::string::npos) {
          params.emplace_back(name, static_cast<int64_t>(std::stoll(value)));
        } else {
          params.emplace_back(name, std::stod(value));
        }
      }
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_ros_->set_parameter(rclcpp::Parameter("global_frame", std::string("map")));
    costmap_ros_->on_configure(rclcpp_lifecycle::State{});
    *(costmap_ros_->getCostmap()) =
      nav2_costmap_2d::Costmap2D(kWindowCells, kWindowCells, kRes, -1.5, -1.5, 0);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    controller_ = std::make_shared<Controller>();
    controller_->configure(node_, "FollowPath", tf_, costmap_ros_);
    controller_->activate();
  }

  ~Rig()
  {
    controller_->deactivate();
    controller_->cleanup();
    controller_.reset();
    costmap_ros_.reset();
    node_.reset();
  }

  Controller & controller() {return *controller_;}
  nav2_costmap_2d::Costmap2D & costmap() {return *costmap_ros_->getCostmap();}
  rclcpp_lifecycle::LifecycleNode::SharedPtr node() {return node_;}

  // Straight line from (x, y) to the goal, one pose every 0.025 m.
  nav_msgs::msg::Path straightToGoal(double x, double y) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    const double len = std::hypot(kGoalX - x, kGoalY - y);
    const double yaw = std::atan2(kGoalY - y, kGoalX - x);
    const int n = std::max(1, static_cast<int>(std::round(len / 0.025)));
    for (int i = 0; i <= n; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x + (kGoalX - x) * i / n;
      p.pose.position.y = y + (kGoalY - y) * i / n;
      p.pose.orientation.z = std::sin(0.5 * yaw);
      p.pose.orientation.w = std::cos(0.5 * yaw);
      path.poses.push_back(p);
    }
    return path;
  }

  // One closed-loop run on geometry `p`. Ends when the goal is reached, when
  // the robot has not left the pocket after `exit_limit` seconds, or at
  // `goal_limit` seconds.
  RunResult run(const Pocket & p, double exit_limit = 60.0, double goal_limit = 150.0)
  {
    RunResult r;
    r.geometry = name(p);
    const auto ws = walls(p);
    double x = kStartX;
    double y = kStartY;
    double yaw = 0.0;
    double v = 0.0;
    double w = 0.0;
    const int replan_every = static_cast<int>(std::round(kReplanPeriod / kDt));
    const int max_steps = static_cast<int>(goal_limit / kDt);
    bool left = false;
    bool entered = false;
    for (int k = 0; k < max_steps; ++k) {
      const double t = k * kDt;
      drawWindow(costmap(), ws, x, y);
      if (k % replan_every == 0) {
        controller_->setPlan(straightToGoal(x, y));
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = node_->now();
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      geometry_msgs::msg::Twist speed;
      speed.linear.x = v;
      speed.angular.z = w;
      double cv = 0.0;
      double cw = 0.0;
      try {
        const auto cmd = controller_->computeVelocityCommands(pose, speed, nullptr);
        cv = cmd.twist.linear.x;
        cw = cmd.twist.angular.z;
      } catch (const std::exception & e) {
        if (!nav2_se_controller::isNoValidControl(e)) {
          throw;
        }
        ++r.throws;  // controller_server publishes zero velocity
      }
      if (verbose_ && k % 5 == 0) {
        std::printf(
          "UTRAPV t=%.1f pose=(%.2f,%.2f,%.2f) cmd=(%.3f,%.3f) throws=%d\n",
          t, x, y, yaw, cv, cw, r.throws);
      }
      // Acceleration limits as in the MPPI configuration.
      cv = std::clamp(cv, v - 3.0 * kDt, v + 3.0 * kDt);
      cw = std::clamp(cw, w - 3.5 * kDt, w + 3.5 * kDt);
      v = cv;
      w = cw;
      yaw += w * kDt;
      const double nx = x + v * std::cos(yaw) * kDt;
      const double ny = y + v * std::sin(yaw) * kDt;
      const double now_d = surfaceDistance(ws, x, y);
      const double new_d = surfaceDistance(ws, nx, ny);
      if (new_d < kRobotRadius && new_d < now_d) {
        ++r.contacts;  // the wall stops the robot
        v = 0.0;
      } else {
        x = nx;
        y = ny;
      }
      r.sim_time = t + kDt;
      r.trace.push_back({t, x, y});
      const double gd = std::hypot(kGoalX - x, kGoalY - y);
      r.min_goal_dist = std::min(r.min_goal_dist, gd);
      if (!entered && insidePocket(p, x, y)) {
        entered = true;
        r.entered_pocket = true;
        r.enter_time = r.sim_time;
      }
      if (entered && !left && !insidePocket(p, x, y)) {
        left = true;
        r.left_pocket = true;
        r.exit_time = r.sim_time;
      }
      if (gd <= 0.5) {
        r.reached = true;
        r.goal_time = r.sim_time;
        break;
      }
      if (!left && r.sim_time >= exit_limit) {  // trapped, or never entered
        break;
      }
    }
    r.enters = controller_->enters();
    r.hops = controller_->hops();
    return r;
  }

private:
  bool verbose_{false};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<Controller> controller_;
};

}  // namespace utrap

#endif  // DEPLOYMENT__UTRAP_RIG_HPP_
