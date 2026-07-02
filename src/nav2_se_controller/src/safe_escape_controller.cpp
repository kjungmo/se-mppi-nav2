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

#include "nav2_se_controller/safe_escape_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "tf2/utils.hpp"

#include "nav2_se_controller/path_progress.hpp"

namespace nav2_se_controller
{

void SafeEscapeController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Reuse the full MPPI setup (optimizer, path handler, parameters handler).
  MPPIController::configure(parent, name, tf, costmap_ros);

  robot_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(se_enabled_, "se_enabled", true);
  getParam(goal_reached_tolerance_, "se_goal_reached_tolerance", 0.5);

  getParam(dynamic_speed_threshold_, "se_dynamic_speed_threshold", 0.1);
  getParam(max_dynamic_radius_, "se_max_obstacle_radius", 1.0);

  EntrapmentConfig ec;
  getParam(ec.progress_stall_window, "se_progress_stall_window", 30);
  detector_.configure(ec);

  CoordinationConfig cc;
  getParam(cc.alpha_base, "se_alpha_base", 2.0);
  getParam(cc.alpha_escape, "se_alpha_escape", 6.0);
  getParam(cc.ttc_override_threshold, "se_ttc_override_threshold", 1.5);
  getParam(cc.q_trust_threshold, "se_q_trust_threshold", 0.25);
  coordinator_.configure(cc);

  CbfConfig fc;
  fc.alpha = cc.alpha_base;
  getParam(fc.lookahead, "se_cbf_lookahead", 0.2);
  getParam(fc.safety_margin, "se_cbf_safety_margin", 0.05);
  getParam(fc.slack_weight, "se_cbf_slack_weight", 1.0e3);
  fc.robot_radius = robot_radius_;
  filter_.configure(fc);

  TrackerConfig tc;
  int cost_threshold = 253;
  getParam(cost_threshold, "se_obstacle_cost_threshold", 253);
  // Costmap occupied values are 0..254 (LETHAL); clamp to that range so a
  // threshold can never be set so high (255 == NO_INFORMATION) that no real
  // obstacle cell ever qualifies and obstacle detection is silently disabled.
  tc.cost_threshold = static_cast<unsigned char>(std::clamp(cost_threshold, 0, 254));
  getParam(tc.min_cells, "se_obstacle_min_cells", 2);
  getParam(tc.association_gate, "se_obstacle_association_gate", 0.6);
  getParam(tc.max_speed, "se_obstacle_max_speed", 2.0);
  // SE-Predict N1: occupancy-persistence static/dynamic classification.
  getParam(tc.classify_static, "se_classify_static", true);
  getParam(tc.static_min_frames, "se_static_min_frames", 10);
  getParam(tc.static_fraction_threshold, "se_static_fraction", 0.5);
  // SE-Predict N2: persistent tracks + short-horizon prediction (the horizon
  // is published on TrackedObstacle; the CBF consumes it from N3).
  getParam(tc.predict_horizon, "se_predict_horizon", true);
  getParam(tc.history_length, "se_track_history", 10);
  getParam(tc.max_missed_frames, "se_track_max_missed", 3);
  // Default "cv": CVCA wins on accelerating/turning agents but loses on
  // oscillatory ones; flip after N3's conformal bound absorbs model misfit.
  std::string predict_model;
  getParam(predict_model, "se_predict_model", std::string("cv"));
  tc.predictor.model = (predict_model == "cvca") ?
    PredictorConfig::Model::kConstantAcceleration :
    PredictorConfig::Model::kConstantVelocity;
  getParam(tc.predictor.horizon_steps, "se_predict_steps", 15);
  getParam(tc.predictor.horizon_dt, "se_predict_dt", 0.1);
  tc.predictor.max_speed = tc.max_speed;
  // SE-Predict N3: conformal calibration -> time-varying CBF radius + the
  // coordinator's prediction-trust gate.
  getParam(tc.conformal, "se_conformal", true);
  getParam(tc.conformal_cfg.coverage, "se_conformal_coverage", 0.9);
  getParam(tc.conformal_cfg.learning_rate, "se_conformal_lr", 0.02);
  getParam(tc.conformal_cfg.initial_q, "se_conformal_initial_q", 0.05);
  getParam(tc.conformal_cfg.max_q, "se_conformal_max_q", 0.40);
  tracker_.configure(tc);

  // Multi-SE-MPPI N2: reciprocal coordination with neighbor robots.
  getParam(multirobot_enabled_, "se_multirobot", false);
  if (multirobot_enabled_) {
    MultiRobotConfig mc;
    getParam(mc.match_radius, "se_neighbor_match_radius", 0.5);
    getParam(mc.reciprocal_lambda, "se_reciprocal_lambda", 0.5);
    getParam(mc.pass_lambda, "se_pass_lambda", 0.7);
    getParam(mc.yield_lambda, "se_yield_lambda", 0.3);
    getParam(mc.deadlock_range, "se_deadlock_range", 1.6);
    getParam(mc.deadlock_speed, "se_deadlock_speed", 0.12);
    getParam(mc.yield_v_max, "se_yield_v_max", 0.10);
    multi_.configure(mc);
    getParam(my_priority_id_, "se_priority_id", 0);

    std::vector<std::string> topics;
    getParam(topics, "se_neighbor_odom_topics", std::vector<std::string>{});
    if (auto node = parent_.lock()) {
      neighbors_.assign(topics.size(), NeighborRobot{});
      for (std::size_t i = 0; i < topics.size(); ++i) {
        neighbors_[i].id = static_cast<int>(i);
        neighbor_subs_.push_back(
          node->create_subscription<nav_msgs::msg::Odometry>(
            topics[i], rclcpp::SensorDataQoS(),
            [this, i](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
              neighbors_[i].position = Eigen::Vector2d(
                msg->pose.pose.position.x, msg->pose.pose.position.y);
              neighbors_[i].velocity = Eigen::Vector2d(
                msg->twist.twist.linear.x, msg->twist.twist.linear.y);
              neighbors_[i].valid = true;
            }));
      }
      RCLCPP_INFO(
        logger_, "SE multirobot: priority_id=%d neighbors=%zu",
        my_priority_id_, topics.size());
    }
  }

  // RViz introspection markers (CBF discs + q inflation, horizons, status).
  getParam(viz_enabled_, "se_viz", true);
  if (viz_enabled_) {
    if (auto node = parent_.lock()) {
      viz_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        name_ + "/se_markers", rclcpp::QoS(1));
    }
  }

  // Register this controller as the single entrapment source of truth; the
  // EscapeCritic (loaded under this controller's name) follows it.
  shared_ = EntrapmentRegistry::get(name_);
  shared_->driven.store(true, std::memory_order_relaxed);

  RCLCPP_INFO(
    logger_,
    "SafeEscapeController[%s] configured: se_enabled=%d robot_radius=%.3f "
    "alpha_base=%.2f alpha_escape=%.2f",
    name_.c_str(), se_enabled_, robot_radius_, cc.alpha_base, cc.alpha_escape);
}

void SafeEscapeController::activate()
{
  MPPIController::activate();
  if (viz_pub_) {
    viz_pub_->on_activate();
  }
}

void SafeEscapeController::deactivate()
{
  if (viz_pub_) {
    viz_pub_->on_deactivate();
  }
  MPPIController::deactivate();
}

void SafeEscapeController::setPlan(const nav_msgs::msg::Path & path)
{
  MPPIController::setPlan(path);
  global_plan_ = path;
  // New reference path => reset the per-task escape/tracking state.
  detector_.reset();
  tracker_.reset();
  furthest_progress_ = 0;
  has_stamp_ = false;
  if (shared_) {
    shared_->entrapped.store(false, std::memory_order_relaxed);
  }
}

void SafeEscapeController::reset()
{
  MPPIController::reset();
  detector_.reset();
  tracker_.reset();
  multi_.reset();
  furthest_progress_ = 0;
  has_stamp_ = false;
  if (shared_) {
    shared_->entrapped.store(false, std::memory_order_relaxed);
  }
}

geometry_msgs::msg::TwistStamped SafeEscapeController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  // Nominal command from the stock MPPI optimizer (incl. EscapeCritic if listed).
  geometry_msgs::msg::TwistStamped cmd =
    MPPIController::computeVelocityCommands(robot_pose, robot_speed, goal_checker);

  if (!se_enabled_) {
    return cmd;
  }

  RobotState state;
  state.x = robot_pose.pose.position.x;
  state.y = robot_pose.pose.position.y;
  state.yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 1. Entrapment from MONOTONIC global-path progress (single source of truth).
  //    nearestPathIndex is non-monotonic, so track the furthest reached index.
  const std::size_t nearest = nearestPathIndex(global_plan_, state.x, state.y);
  furthest_progress_ = std::max(furthest_progress_, nearest);
  bool entrapped = detector_.update(furthest_progress_);

  // Suppress entrapment near the goal: a robot finishing at the path end would
  // otherwise stall the progress signal and trigger a false escape.
  if (!global_plan_.poses.empty()) {
    const auto & goal = global_plan_.poses.back().pose.position;
    const double dist_to_goal = std::hypot(goal.x - state.x, goal.y - state.y);
    if (dist_to_goal <= goal_reached_tolerance_) {
      entrapped = false;
      detector_.reset();
      furthest_progress_ = 0;
    }
  }
  if (shared_) {
    shared_->entrapped.store(entrapped, std::memory_order_relaxed);
  }

  // Runtime evidence for the live A/B: log escape-mode transitions so a launch
  // log proves whether entrapment detection fired (and when it cleared).
  if (entrapped != prev_entrapped_) {
    RCLCPP_INFO(
      logger_,
      "SE escape %s at (%.2f, %.2f): progress_idx=%zu stall=%d",
      entrapped ? "ENTER" : "EXIT", state.x, state.y,
      furthest_progress_, detector_.stallCount());
    prev_entrapped_ = entrapped;
  }

  // 2. Dynamic obstacles from the local costmap. On a clock-lock failure reuse
  //    the previous stamp (dt ~ 0) rather than 0.0, which would make dt negative
  //    and silently zero all obstacle velocities (an unsafe TTC = +inf).
  double stamp = prev_stamp_;
  if (auto node = parent_.lock()) {
    stamp = node->now().seconds();
  }
  prev_stamp_ = stamp;
  has_stamp_ = true;
  const std::vector<TrackedObstacle> tracked =
    tracker_.update(*costmap_ros_->getCostmap(), stamp);

  // Keep only genuinely DYNAMIC obstacles for the CBF/coordinator: static walls
  // (large, ~zero velocity) are the MPPI/costmap's job, and would otherwise enter
  // the look-ahead-point CBF as room-sized circles that can never be hard-safe and
  // would brake the robot in place. Scope matches the design (CBF = dynamic only).
  // is_dynamic (occupancy persistence, SE-Predict N1) vetoes first: a wall with
  // an association-jitter phantom velocity passes the speed test but not this.
  std::vector<TrackedObstacle> obstacles;
  obstacles.reserve(tracked.size());
  for (const auto & o : tracked) {
    if (o.is_dynamic &&
      o.velocity.norm() >= dynamic_speed_threshold_ && o.radius <= max_dynamic_radius_)
    {
      obstacles.push_back(o);
    }
  }

  // 2b. Multi-robot coordination (when enabled): mark neighbor robots with
  //     the reciprocal budget share and run the deadlock/priority machine.
  MultiRobotCoordinator::Role role = MultiRobotCoordinator::Role::kNone;
  if (multirobot_enabled_) {
    Eigen::Vector2d goal_xy(state.x, state.y);
    if (!global_plan_.poses.empty()) {
      goal_xy = Eigen::Vector2d(
        global_plan_.poses.back().pose.position.x,
        global_plan_.poses.back().pose.position.y);
    }
    role = multi_.update(
      my_priority_id_, state, goal_xy, cmd.twist.linear.x, entrapped,
      neighbors_);
    multi_.markNeighbors(obstacles, neighbors_);
  }

  // 3. Coordinate the CBF gain (raise it to permit certified-safe escape,
  //    unless a dynamic obstacle's TTC is imminent). A sanctioned PASS role
  //    keeps the escape gain even when the deadlock stall has frozen the
  //    normal progress dynamics.
  const bool escape_intent =
    entrapped || role == MultiRobotCoordinator::Role::kPass;
  const double alpha =
    coordinator_.resolveAlpha(
    escape_intent, state, cmd.twist.linear.x, obstacles, robot_radius_);

  // 4. Project the nominal control onto the CBF-safe set.
  const CbfSafetyFilter::Result safe =
    filter_.filter(state, cmd.twist.linear.x, cmd.twist.angular.z, obstacles, alpha);

  cmd.twist.linear.x = safe.v;
  cmd.twist.angular.z = safe.w;
  // If the QP could only stay safe by relaxing the barrier (slack > 0) or failed
  // to verify safety, brake the forward motion: stop rather than drive into an
  // imminent collision, while keeping the (safest-available) turn to clear it.
  if (!safe.hard_safe) {
    cmd.twist.linear.x = 0.0;
  }
  // Yield primitive: hold back while the passer clears (Multi-SE-MPPI N2).
  if (role == MultiRobotCoordinator::Role::kYield) {
    cmd.twist.linear.x =
      std::min(cmd.twist.linear.x, multi_.config().yield_v_max);
  }

  publishMarkers(state, obstacles, alpha, safe.slack, entrapped);
  return cmd;
}

void SafeEscapeController::publishMarkers(
  const RobotState & state, const std::vector<TrackedObstacle> & obstacles,
  double alpha, double slack, bool entrapped)
{
  if (!viz_pub_ || !viz_pub_->is_activated() ||
    viz_pub_->get_subscription_count() == 0)
  {
    return;
  }
  visualization_msgs::msg::MarkerArray arr;
  const std::string frame = costmap_ros_->getGlobalFrameID();
  rclcpp::Time stamp;
  if (auto node = parent_.lock()) {
    stamp = node->now();
  }
  const rclcpp::Duration life = rclcpp::Duration::from_seconds(0.3);

  int id = 0;
  for (const auto & o : obstacles) {
    const double q0 = o.q.empty() ? 0.0 : o.q.front();
    const double eff_r =
      robot_radius_ + o.radius + filter_.config().safety_margin + q0;

    visualization_msgs::msg::Marker disc;
    disc.header.frame_id = frame;
    disc.header.stamp = stamp;
    disc.ns = "se_cbf";
    disc.id = id++;
    disc.type = visualization_msgs::msg::Marker::CYLINDER;
    disc.action = visualization_msgs::msg::Marker::ADD;
    disc.pose.position.x = o.position.x();
    disc.pose.position.y = o.position.y();
    disc.pose.position.z = 0.05;
    disc.pose.orientation.w = 1.0;
    disc.scale.x = 2.0 * eff_r;
    disc.scale.y = 2.0 * eff_r;
    disc.scale.z = 0.02;
    disc.color.r = 1.0f;
    disc.color.g = 0.45f;
    disc.color.b = 0.0f;
    disc.color.a = 0.25f;
    disc.lifetime = life;
    arr.markers.push_back(disc);

    if (!o.horizon.empty()) {
      visualization_msgs::msg::Marker line;
      line.header = disc.header;
      line.ns = "se_horizon";
      line.id = id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.02;
      line.color.r = 0.0f;
      line.color.g = 0.8f;
      line.color.b = 1.0f;
      line.color.a = 0.9f;
      line.lifetime = life;
      geometry_msgs::msg::Point p0;
      p0.x = o.position.x();
      p0.y = o.position.y();
      line.points.push_back(p0);
      for (const auto & ph : o.horizon) {
        geometry_msgs::msg::Point p;
        p.x = ph.x();
        p.y = ph.y();
        line.points.push_back(p);
      }
      arr.markers.push_back(line);
    }
  }

  double max_q = 0.0;
  for (const auto & o : obstacles) {
    if (!o.q.empty()) {
      max_q = std::max(max_q, o.q.front());
    }
  }
  visualization_msgs::msg::Marker text;
  text.header.frame_id = frame;
  text.header.stamp = stamp;
  text.ns = "se_status";
  text.id = id++;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position.x = state.x;
  text.pose.position.y = state.y;
  text.pose.position.z = 0.6;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.18;
  text.color.r = entrapped ? 1.0f : 0.2f;
  text.color.g = entrapped ? 0.3f : 1.0f;
  text.color.b = 0.2f;
  text.color.a = 1.0f;
  text.lifetime = life;
  char buf[96];
  std::snprintf(
    buf, sizeof(buf), "a=%.1f slack=%.3f esc=%d q=%.2f",
    alpha, slack, entrapped ? 1 : 0, max_q);
  text.text = buf;
  arr.markers.push_back(text);

  viz_pub_->publish(arr);
}

}  // namespace nav2_se_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_se_controller::SafeEscapeController, nav2_core::Controller)
