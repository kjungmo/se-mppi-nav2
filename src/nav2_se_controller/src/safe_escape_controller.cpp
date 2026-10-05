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

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
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

  // CBF disc radius: the footprint's CIRCUMSCRIBED radius, so the certified
  // disc covers the whole (padded) footprint; the inscribed radius left the
  // corners of a rectangular robot outside it. se_cbf_robot_radius > 0
  // overrides it (see reapplyConfig()).
  robot_radius_ = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();

  // Parameters bound directly to members (se_enabled_, ...) stay Dynamic: MPPI's
  // ParametersHandler writes the member on a runtime set and the control loop
  // reads it under the same lock. The algorithm configs are members too
  // (ec_, cc_, fc_, tc_, mc_ and the raw values they derive from) and are
  // re-applied to their consumers by reapplyConfig() from MPPI's post-set
  // callback. Binding them to configure()-local structs, as before, made a
  // runtime `ros2 param set` write through a dangling stack reference while
  // reporting success.
  auto getParam = parameters_handler_->getParamGetter(name_);
  // Bind a config field: initial value via MPPI (declared Static so MPPI keeps
  // no reference of its own), runtime changes via our callback, which writes
  // the member field and marks the consumers to re-apply.
  auto bindConfig = [this, &getParam](auto & field, const std::string & key,
    auto default_value, unsigned consumers) {
    using FieldT = std::decay_t<decltype(field)>;
    using ParamT = decltype(default_value);
    parameters_handler_->addDynamicParamCallback(
      name_ + "." + key,
      [this, &field, consumers](const rclcpp::Parameter & p) {
        field = static_cast<FieldT>(p.get_value<ParamT>());
        pending_reapply_ |= consumers;
      });
    getParam(field, key, std::move(default_value), mppi::ParameterType::Static);
  };

  getParam(se_enabled_, "se_enabled", true);
  getParam(goal_reached_tolerance_, "se_goal_reached_tolerance", 0.5);

  getParam(dynamic_speed_threshold_, "se_dynamic_speed_threshold", 0.1);
  getParam(max_dynamic_radius_, "se_max_obstacle_radius", 1.0);
  bindConfig(
    stale_grid_timeout_param_, "se_tracker_stale_grid_timeout", 0.0, kTracker);

  bindConfig(ec_.progress_stall_window, "se_progress_stall_window", 30, kDetector);

  bindConfig(cc_.alpha_base, "se_alpha_base", 2.0, kCoordinator | kFilter);
  bindConfig(cc_.alpha_escape, "se_alpha_escape", 6.0, kCoordinator);
  bindConfig(cc_.ttc_override_threshold, "se_ttc_override_threshold", 1.5, kCoordinator);
  bindConfig(cc_.q_trust_threshold, "se_q_trust_threshold", 0.25, kCoordinator);

  bindConfig(fc_.lookahead, "se_cbf_lookahead", 0.2, kFilter);
  bindConfig(fc_.safety_margin, "se_cbf_safety_margin", 0.05, kFilter);
  bindConfig(fc_.slack_weight, "se_cbf_slack_weight", 1.0e3, kFilter);
  bindConfig(cbf_robot_radius_, "se_cbf_robot_radius", 0.0, kFilter);

  bindConfig(cost_threshold_, "se_obstacle_cost_threshold", 253, kTracker);
  bindConfig(tc_.min_cells, "se_obstacle_min_cells", 2, kTracker);
  bindConfig(tc_.association_gate, "se_obstacle_association_gate", 0.6, kTracker);
  bindConfig(tc_.max_speed, "se_obstacle_max_speed", 2.0, kTracker);
  // SE-Predict N1: occupancy-persistence static/dynamic classification.
  bindConfig(tc_.classify_static, "se_classify_static", true, kTracker);
  bindConfig(tc_.static_min_frames, "se_static_min_frames", 10, kTracker);
  bindConfig(tc_.static_fraction_threshold, "se_static_fraction", 0.5, kTracker);
  // SE-Predict N2: persistent tracks + short-horizon prediction (the horizon
  // is published on TrackedObstacle; the CBF consumes it from N3).
  bindConfig(tc_.predict_horizon, "se_predict_horizon", true, kTracker);
  bindConfig(tc_.history_length, "se_track_history", 10, kTracker);
  bindConfig(tc_.max_missed_frames, "se_track_max_missed", 3, kTracker);
  // Default "cv": CVCA wins on accelerating/turning agents but loses on
  // oscillatory ones; flip after N3's conformal bound absorbs model misfit.
  bindConfig(predict_model_, "se_predict_model", std::string("cv"), kTracker);
  // The conformal bounds are per horizon step: a new step count or spacing
  // invalidates them (kConformal restarts the calibration).
  bindConfig(tc_.predictor.horizon_steps, "se_predict_steps", 15, kTracker | kConformal);
  bindConfig(tc_.predictor.horizon_dt, "se_predict_dt", 0.1, kTracker | kConformal);
  // SE-Predict N3: conformal calibration -> time-varying CBF radius + the
  // coordinator's prediction-trust gate.
  bindConfig(tc_.conformal, "se_conformal", true, kTracker);
  bindConfig(tc_.conformal_cfg.coverage, "se_conformal_coverage", 0.9, kTracker | kConformal);
  bindConfig(tc_.conformal_cfg.learning_rate, "se_conformal_lr", 0.02, kTracker | kConformal);
  bindConfig(
    tc_.conformal_cfg.initial_q, "se_conformal_initial_q", 0.05, kTracker | kConformal);
  bindConfig(tc_.conformal_cfg.max_q, "se_conformal_max_q", 0.40, kTracker | kConformal);

  // Multi-SE-MPPI N2: reciprocal coordination with neighbor robots.
  // se_multirobot and se_viz decide what configure() creates (neighbor
  // subscriptions and parameters, the marker publisher), so they are
  // configure-only: Static, and a runtime set is rejected below.
  getParam(multirobot_enabled_, "se_multirobot", false, mppi::ParameterType::Static);
  configure_only_params_.insert(name_ + ".se_multirobot");
  if (multirobot_enabled_) {
    bindConfig(mc_.match_radius, "se_neighbor_match_radius", 0.5, kMulti);
    bindConfig(mc_.reciprocal_lambda, "se_reciprocal_lambda", 0.5, kMulti);
    bindConfig(mc_.pass_lambda, "se_pass_lambda", 0.7, kMulti);
    bindConfig(mc_.yield_lambda, "se_yield_lambda", 0.3, kMulti);
    bindConfig(mc_.deadlock_range, "se_deadlock_range", 1.6, kMulti);
    bindConfig(mc_.deadlock_speed, "se_deadlock_speed", 0.12, kMulti);
    bindConfig(mc_.yield_v_max, "se_yield_v_max", 0.10, kMulti);
    getParam(my_priority_id_, "se_priority_id", 0);

    // The subscriptions are created here only, so the topic list cannot be
    // re-applied at runtime: Static, and the set callback below rejects it.
    std::vector<std::string> topics;
    getParam(
      topics, "se_neighbor_odom_topics", std::vector<std::string>{},
      mppi::ParameterType::Static);
    configure_only_params_.insert(name_ + ".se_neighbor_odom_topics");
    if (auto node = parent_.lock()) {
      neighbors_.assign(topics.size(), NeighborRobot{});
      for (std::size_t i = 0; i < topics.size(); ++i) {
        neighbors_[i].id = static_cast<int>(i);
        neighbor_subs_.push_back(
          node->create_subscription<nav_msgs::msg::Odometry>(
            topics[i], rclcpp::SensorDataQoS(),
            [this, i](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
              std::lock_guard<std::mutex> lock(neighbors_mutex_);
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

  // The CBF filter's velocity box is MPPI's own limits (declared by the MPPI
  // optimizer above), not separate hard-coded values: a robot configured with
  // vx_max = 1.0 must not be clamped to 0.5 by the safety layer.
  if (auto node = parent_.lock()) {
    node->get_parameter(name_ + ".vx_max", fc_.v_max);
    node->get_parameter(name_ + ".vx_min", fc_.v_min);
    node->get_parameter(name_ + ".wz_max", fc_.w_max);
    // Braking to a stop must stay feasible even when MPPI's vx_min > 0.
    fc_.v_min = std::min(fc_.v_min, 0.0);
  }

  pending_reapply_ = kAllConsumers;
  reapplyConfig();
  // MPPI runs post-set callbacks under its parameter lock, which the SE part
  // of computeVelocityCommands also holds, so the consumers never change
  // mid-cycle.
  parameters_handler_->addPostCallback([this]() {reapplyConfig();});

  if (auto node = parent_.lock()) {
    // MPPI's ParametersHandler answers successful=true for a name it has no
    // callback for, so a Static parameter needs an explicit rejection or a
    // runtime set would be "successful" with no effect.
    reject_set_handle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (configure_only_params_.count(p.get_name()) != 0) {
            result.successful = false;
            result.reason = p.get_name() +
            " is read only at configure; set it while the controller_server is "
            "unconfigured (lifecycle cleanup), then configure";
            return result;
          }
        }
        // MPPI's own velocity limits: follow them into the CBF box. The node
        // value is not committed yet here, so take the incoming value.
        std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
        for (const auto & p : params) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            continue;
          }
          if (p.get_name() == name_ + ".vx_max") {
            fc_.v_max = p.as_double();
          } else if (p.get_name() == name_ + ".vx_min") {
            fc_.v_min = std::min(p.as_double(), 0.0);  // stopping stays feasible
          } else if (p.get_name() == name_ + ".wz_max") {
            fc_.w_max = p.as_double();
          } else {
            continue;
          }
          pending_reapply_ |= kFilter;
        }
        // Apply here: rclcpp does not guarantee this callback runs before
        // MPPI's handler (and its post-set re-apply).
        reapplyConfig();
        return result;
      });
  }

  // RViz introspection markers (CBF discs + q inflation, horizons, status).
  getParam(viz_enabled_, "se_viz", true, mppi::ParameterType::Static);
  configure_only_params_.insert(name_ + ".se_viz");
  if (viz_enabled_) {
    if (auto node = parent_.lock()) {
      viz_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        name_ + "/se_markers", rclcpp::QoS(1));
    }
  }

  // Failure visibility on /diagnostics (the aggregator's conventional topic).
  if (auto node = parent_.lock()) {
    diag_name_ = std::string(node->get_name()) + ": se_mppi (" + name_ + ")";
    diag_hardware_id_ = node->get_namespace();
    diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
  }

  // Register this controller as the single entrapment source of truth; the
  // EscapeCritic (loaded under this controller's name) follows it.
  shared_ = EntrapmentRegistry::get(name_);
  shared_->driven.store(true, std::memory_order_relaxed);

  RCLCPP_INFO(
    logger_,
    "SafeEscapeController[%s] configured: se_enabled=%d robot_radius=%.3f "
    "alpha_base=%.2f alpha_escape=%.2f",
    name_.c_str(), se_enabled_, robot_radius_, cc_.alpha_base, cc_.alpha_escape);
}

void SafeEscapeController::reapplyConfig()
{
  const unsigned pending = pending_reapply_;
  pending_reapply_ = 0;
  if (pending & kDetector) {
    detector_.configure(ec_);
  }
  if (pending & kCoordinator) {
    coordinator_.configure(cc_);
  }
  if (pending & kFilter) {
    robot_radius_ = cbf_robot_radius_ > 0.0 ? cbf_robot_radius_ :
      costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    CbfConfig fc = fc_;
    // Same scaling MPPI's Optimizer::setSpeedLimit applies to its limits.
    fc.v_max = fc_.v_max * speed_limit_ratio_;
    fc.v_min = fc_.v_min * speed_limit_ratio_;
    fc.w_max = fc_.w_max * speed_limit_ratio_;
    fc.alpha = cc_.alpha_base;
    fc.robot_radius = robot_radius_;
    filter_.configure(fc);
  }
  if (pending & kTracker) {
    // Default stale-grid timeout: two local-costmap update periods. A live
    // costmap re-renders every period, so a grid identical across two periods
    // is a still scene or stale data; either way the tracker must advance.
    double update_frequency = 0.0;
    costmap_ros_->get_parameter("update_frequency", update_frequency);
    stale_grid_timeout_ = stale_grid_timeout_param_ > 0.0 ? stale_grid_timeout_param_ :
      (update_frequency > 0.0 ? 2.0 / update_frequency : 0.5);
    TrackerConfig tc = tc_;
    // Costmap occupied values are 0..254 (LETHAL); clamp to that range so a
    // threshold can never be set so high (255 == NO_INFORMATION) that no real
    // obstacle cell ever qualifies and obstacle detection is silently disabled.
    tc.cost_threshold = static_cast<unsigned char>(std::clamp(cost_threshold_, 0, 254));
    tc.predictor.model = (predict_model_ == "cvca") ?
      PredictorConfig::Model::kConstantAcceleration :
      PredictorConfig::Model::kConstantVelocity;
    tc.predictor.max_speed = tc.max_speed;
    const bool reset_calibration = (pending & kConformal) != 0;
    tracker_.configure(tc, reset_calibration);
    if (reset_calibration && config_applied_once_) {
      RCLCPP_WARN(
        logger_,
        "SE conformal settings changed at runtime: the learned prediction-error "
        "bounds were reset to se_conformal_initial_q=%.3f (the CBF inflation "
        "restarts from there)", tc.conformal_cfg.initial_q);
    }
  }
  if (pending & kMulti) {
    multi_.configure(mc_);
  }
  se_enabled_mirror_.store(se_enabled_);
  config_applied_once_ = true;
}

void SafeEscapeController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  MPPIController::setSpeedLimit(speed_limit, percentage);
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  if (speed_limit == 0.0) {  // nav2_costmap_2d::NO_SPEED_LIMIT
    speed_limit_ratio_ = 1.0;
  } else if (percentage) {
    speed_limit_ratio_ = speed_limit / 100.0;
  } else {
    speed_limit_ratio_ = fc_.v_max > 0.0 ? speed_limit / fc_.v_max : 1.0;
  }
  pending_reapply_ |= kFilter;
  reapplyConfig();
}

void SafeEscapeController::cleanup()
{
  diag_timer_.reset();
  diag_pub_.reset();
  if (reject_set_handle_) {
    if (auto node = parent_.lock()) {
      node->remove_on_set_parameters_callback(reject_set_handle_.get());
    }
    reject_set_handle_.reset();
  }
  configure_only_params_.clear();
  MPPIController::cleanup();
}

void SafeEscapeController::activate()
{
  MPPIController::activate();
  if (viz_pub_) {
    viz_pub_->on_activate();
  }
  if (diag_pub_) {
    diag_pub_->on_activate();
    if (auto node = parent_.lock()) {
      // Wall timer: diagnostics keep flowing with use_sim_time and no /clock.
      diag_timer_ = node->create_wall_timer(
        std::chrono::seconds(1), [this]() {publishDiagnostics();});
    }
  }
}

void SafeEscapeController::deactivate()
{
  diag_timer_.reset();
  if (diag_pub_) {
    diag_pub_->on_deactivate();
  }
  if (viz_pub_) {
    viz_pub_->on_deactivate();
  }
  MPPIController::deactivate();
}

void SafeEscapeController::setPlan(const nav_msgs::msg::Path & path)
{
  MPPIController::setPlan(path);
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  global_plan_ = path;
  if (path.poses.empty()) {
    return;
  }
  const auto & g = path.poses.back().pose.position;
  const bool same_goal = has_goal_ && path.header.frame_id == goal_frame_ &&
    std::hypot(g.x - goal_x_, g.y - goal_y_) < 1.0e-3;
  has_goal_ = true;
  goal_frame_ = path.header.frame_id;
  goal_x_ = g.x;
  goal_y_ = g.y;
  if (same_goal) {
    // Replanned path to the same goal: obstacle tracks and the stall count are
    // world/task state, not path state. Only the path-index baseline changes.
    rebase_progress_ = true;
    return;
  }
  // New goal => reset the per-task escape/tracking state.
  rebase_progress_ = false;
  detector_.reset();
  tracker_.reset();
  last_grid_.clear();
  last_tracked_.clear();
  furthest_progress_ = 0;
  has_stamp_ = false;
  if (shared_) {
    shared_->entrapped.store(false, std::memory_order_relaxed);
  }
}

void SafeEscapeController::reset()
{
  MPPIController::reset();
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  detector_.reset();
  tracker_.reset();
  last_grid_.clear();
  last_tracked_.clear();
  multi_.reset();
  furthest_progress_ = 0;
  has_stamp_ = false;
  has_goal_ = false;
  rebase_progress_ = false;
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

  // MPPI released its parameter lock on return; hold it for the SE part so a
  // runtime parameter set (and the config re-apply) cannot land mid-cycle.
  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

  if (!se_enabled_) {
    return cmd;
  }

  RobotState state;
  state.x = robot_pose.pose.position.x;
  state.y = robot_pose.pose.position.y;
  state.yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 0. Frames: the global plan is in the planner's frame (e.g. map), the pose
  //    in the local costmap's frame (e.g. odom). Express the robot in the plan
  //    frame for progress and goal distance, and the goal in the robot frame
  //    for the multi-robot coordinator.
  double plan_x = state.x;
  double plan_y = state.y;
  Eigen::Vector2d goal_xy(state.x, state.y);
  bool plan_frame_ok = true;
  if (!global_plan_.poses.empty()) {
    const auto & g = global_plan_.poses.back().pose.position;
    goal_xy = Eigen::Vector2d(g.x, g.y);
    const std::string & plan_frame = global_plan_.header.frame_id;
    if (!plan_frame.empty() && plan_frame != robot_pose.header.frame_id) {
      try {
        const auto t = tf_buffer_->lookupTransform(
          plan_frame, robot_pose.header.frame_id, tf2::TimePointZero);
        const tf2::Transform plan_from_robot(
          tf2::Quaternion(
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w),
          tf2::Vector3(
            t.transform.translation.x, t.transform.translation.y,
            t.transform.translation.z));
        const tf2::Vector3 r = plan_from_robot * tf2::Vector3(state.x, state.y, 0.0);
        plan_x = r.x();
        plan_y = r.y();
        const tf2::Vector3 gr = plan_from_robot.inverse() * tf2::Vector3(g.x, g.y, 0.0);
        goal_xy = Eigen::Vector2d(gr.x(), gr.y());
      } catch (const tf2::TransformException & e) {
        plan_frame_ok = false;
        RCLCPP_WARN_THROTTLE(
          logger_, steady_clock_, 2000,
          "SE: no transform %s -> %s (%s); entrapment detection holds its state",
          robot_pose.header.frame_id.c_str(), plan_frame.c_str(), e.what());
      }
    }
  }

  // 1. Entrapment from MONOTONIC global-path progress (single source of truth).
  //    nearestPathIndex is non-monotonic, so track the furthest reached index.
  bool entrapped = prev_entrapped_;
  if (plan_frame_ok) {
    const std::size_t nearest = nearestPathIndex(global_plan_, plan_x, plan_y);
    if (rebase_progress_) {
      furthest_progress_ = nearest;
      detector_.rebase(nearest);
      rebase_progress_ = false;
    }
    furthest_progress_ = std::max(furthest_progress_, nearest);
    entrapped = detector_.update(furthest_progress_);

    // Suppress entrapment near the goal: a robot finishing at the path end would
    // otherwise stall the progress signal and trigger a false escape.
    if (!global_plan_.poses.empty()) {
      const auto & goal = global_plan_.poses.back().pose.position;
      const double dist_to_goal = std::hypot(goal.x - plan_x, goal.y - plan_y);
      if (dist_to_goal <= goal_reached_tolerance_) {
        entrapped = false;
        detector_.reset();
        furthest_progress_ = 0;
      }
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
    if (entrapped) {
      counters_.escape_entries.fetch_add(1);
    }
  }
  counters_.entrapped.store(entrapped);
  counters_.cycles.fetch_add(1);

  // 2. Dynamic obstacles from the local costmap. On a clock-lock failure reuse
  //    the previous stamp (dt ~ 0) rather than 0.0, which would make dt negative
  //    and silently zero all obstacle velocities (an unsafe TTC = +inf).
  double stamp = prev_stamp_;
  if (auto node = parent_.lock()) {
    stamp = node->now().seconds();
  }
  prev_stamp_ = stamp;
  has_stamp_ = true;
  // The stock MPPI step released the costmap lock when it returned; take it
  // again so the costmap update thread cannot rewrite cells mid-read.
  std::vector<TrackedObstacle> tracked;
  {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
    const unsigned int w = costmap->getSizeInCellsX();
    const unsigned int h = costmap->getSizeInCellsY();
    const unsigned char * grid = costmap->getCharMap();
    const bool unchanged = !last_grid_.empty() && w == last_grid_w_ && h == last_grid_h_ &&
      costmap->getOriginX() == last_grid_ox_ && costmap->getOriginY() == last_grid_oy_ &&
      std::equal(last_grid_.begin(), last_grid_.end(), grid);
    if (unchanged && stamp - last_tracker_stamp_ < stale_grid_timeout_) {
      tracked = last_tracked_;  // same frame as last cycle: keep its estimates
    } else {
      // New data, or a grid frozen past the timeout: advance the tracker.
      tracked = tracker_.update(*costmap, stamp);
      last_tracker_stamp_ = stamp;
      last_grid_.assign(grid, grid + static_cast<std::size_t>(w) * h);
      last_grid_w_ = w;
      last_grid_h_ = h;
      last_grid_ox_ = costmap->getOriginX();
      last_grid_oy_ = costmap->getOriginY();
      last_tracked_ = tracked;
    }
  }
  const std::size_t dropped = tracker_.droppedTrackCount();
  const std::uint64_t dropped_before = counters_.tracks_dropped.exchange(dropped);
  if (dropped > dropped_before) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 10000,
      "SE tracker: %zu obstacle track(s) dropped after more than %d unmatched frames "
      "(se_track_max_missed); %zu in total",
      static_cast<std::size_t>(dropped - dropped_before), tc_.max_missed_frames, dropped);
  }

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
    // Snapshot under the lock: the odom callbacks run on the executor thread.
    std::vector<NeighborRobot> neighbors;
    {
      std::lock_guard<std::mutex> lock(neighbors_mutex_);
      neighbors = neighbors_;
    }
    role = multi_.update(
      my_priority_id_, state, goal_xy, cmd.twist.linear.x, entrapped,
      neighbors);
    multi_.markNeighbors(obstacles, neighbors);
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
  // Visibility only: the command above is already decided.
  counters_.cbf_obstacles.store(static_cast<int>(obstacles.size()));
  counters_.alpha.store(alpha);
  qp_failure_streak_.record(!safe.feasible, steady_clock_.now().nanoseconds());
  if (!safe.hard_safe) {
    counters_.forced_stops.fetch_add(1);
    if (!safe.feasible) {
      counters_.qp_failures.fetch_add(1);
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 2000,
        "SE CBF filter: QP setup/solve failed (%zu dynamic obstacle(s), alpha=%.2f); "
        "forward velocity forced to 0",
        obstacles.size(), alpha);
    } else {
      counters_.slack_active.fetch_add(1);
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 2000,
        "SE CBF filter: barrier relaxed, slack=%.4f (%zu dynamic obstacle(s), alpha=%.2f); "
        "forward velocity forced to 0",
        safe.slack, obstacles.size(), alpha);
    }
  }
  // Yield primitive: hold back while the passer clears (Multi-SE-MPPI N2).
  if (role == MultiRobotCoordinator::Role::kYield) {
    cmd.twist.linear.x =
      std::min(cmd.twist.linear.x, multi_.config().yield_v_max);
  }

  publishMarkers(state, obstacles, alpha, safe.slack, entrapped);
  return cmd;
}

void SafeEscapeController::publishDiagnostics()
{
  if (!diag_pub_ || !diag_pub_->is_activated()) {
    return;
  }
  const std::uint64_t forced = counters_.forced_stops.load();
  const std::uint64_t qp_failed = counters_.qp_failures.load();
  const std::uint64_t new_forced = forced - reported_forced_stops_;
  const std::uint64_t new_qp_failed = qp_failed - reported_qp_failures_;
  reported_forced_stops_ = forced;
  reported_qp_failures_ = qp_failed;
  const bool se_enabled = se_enabled_mirror_.load();
  const std::int64_t now_ns = steady_clock_.now().nanoseconds();
  const bool qp_failing_long = qp_failure_streak_.longerThan(
    now_ns, static_cast<std::int64_t>(kQpFailureErrorAfterSec * 1e9),
    static_cast<std::int64_t>(0.5e9));

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = diag_name_;
  st.hardware_id = diag_hardware_id_;
  if (!se_enabled) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "SE layer disabled (se_enabled=false): stock MPPI output";
  } else if (qp_failing_long) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "CBF QP failing continuously for more than " +
      std::to_string(kQpFailureErrorAfterSec).substr(0, 3) +
      " s; forward velocity held at 0";
  } else if (new_qp_failed > 0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "CBF QP failed in the last second; forward velocity forced to 0";
  } else if (new_forced > 0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "CBF barrier relaxed in the last second; forward velocity forced to 0";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = counters_.entrapped.load() ? "escape mode" : "ok";
  }
  auto kv = [&st](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue e;
    e.key = key;
    e.value = value;
    st.values.push_back(e);
  };
  kv("se_enabled", se_enabled ? "true" : "false");
  kv("cycles", std::to_string(counters_.cycles.load()));
  kv("forced_stops", std::to_string(forced));
  kv("forced_stops_last_period", std::to_string(new_forced));
  kv("qp_failures", std::to_string(qp_failed));
  kv("slack_active", std::to_string(counters_.slack_active.load()));
  kv("tracks_dropped", std::to_string(counters_.tracks_dropped.load()));
  kv("escape_entries", std::to_string(counters_.escape_entries.load()));
  kv("entrapped", counters_.entrapped.load() ? "true" : "false");
  kv("cbf_obstacles", std::to_string(counters_.cbf_obstacles.load()));
  kv("alpha", std::to_string(counters_.alpha.load()));

  diagnostic_msgs::msg::DiagnosticArray arr;
  if (auto node = parent_.lock()) {
    arr.header.stamp = node->now();
  }
  arr.status.push_back(st);
  diag_pub_->publish(arr);
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
