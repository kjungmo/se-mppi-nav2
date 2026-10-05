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

#ifndef NAV2_SE_CONTROLLER__SAFE_ESCAPE_CONTROLLER_HPP_
#define NAV2_SE_CONTROLLER__SAFE_ESCAPE_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_se_controller/cbf_safety_filter.hpp"
#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"
#include "nav2_se_controller/entrapment_detector.hpp"
#include "nav2_se_controller/entrapment_state.hpp"
#include "nav2_se_controller/escape_safety_coordinator.hpp"
#include "nav2_se_controller/multi_robot_coordinator.hpp"

namespace nav2_se_controller
{

/**
 * @class SafeEscapeController
 * @brief Nav2-native SE-MPPI controller: stock MPPI + local-minima escape +
 *        dynamic-obstacle CBF safety, reconciled by the escape-safety coordinator.
 *
 * Subclasses the stock nav2_mppi_controller::MPPIController to reuse its
 * (heavily optimised) sampling optimizer for the nominal command, then post-
 * processes that command each cycle:
 *   1. Detect entrapment from global-path progress (EntrapmentDetector).
 *   2. Track dynamic obstacles from the local costmap (DynamicObstacleTracker).
 *   3. Resolve the CBF gain alpha from entrapment + time-to-collision
 *      (EscapeSafetyCoordinator) — raising it to permit certified-safe escape.
 *   4. Project the nominal (v, w) onto the CBF-safe set (CbfSafetyFilter).
 *
 * The sampling-time repulsive escape (EscapeCritic) is enabled via the
 * optimizer's `critics` parameter list and is complementary to this output-side
 * safety projection. See docs/architecture/2026-06_safe-escape-mppi-design.md.
 */
class SafeEscapeController : public nav2_mppi_controller::MPPIController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  bool se_enabled_{true};
  double robot_radius_{0.22};
  double goal_reached_tolerance_{0.5};  // suppress entrapment within this of the goal (m)
  // The CBF layer is scoped to DYNAMIC obstacles; static structure (walls) is
  // handled by the MPPI obstacle critic + costmap inflation. The tracker clusters
  // any LETHAL cell, so static walls would otherwise enter the CBF as huge circular
  // obstacles and brake the robot forever. Admit a cluster only if it is actually
  // moving (speed gate) and small enough to be a movable body (radius cap).
  double dynamic_speed_threshold_{0.1};  // m/s; below this a cluster is static
  double max_dynamic_radius_{1.0};       // m; above this a cluster is structure, not a body

  nav_msgs::msg::Path global_plan_;
  EntrapmentDetector detector_;
  DynamicObstacleTracker tracker_;
  EscapeSafetyCoordinator coordinator_;
  CbfSafetyFilter filter_;

  // Parameter-bound algorithm configs. They are members (not configure()
  // locals) because a runtime parameter set writes into them; reapplyConfig()
  // then pushes the marked ones into their consumers. Guarded by MPPI's
  // parameter lock (parameters_handler_->getLock()).
  enum Consumer : unsigned
  {
    kDetector = 1u, kCoordinator = 2u, kFilter = 4u, kTracker = 8u, kMulti = 16u,
    kAllConsumers = 31u
  };
  EntrapmentConfig ec_;
  CoordinationConfig cc_;
  CbfConfig fc_;              // alpha and robot_radius are filled in on re-apply
  TrackerConfig tc_;          // cost_threshold / predictor model derived on re-apply
  MultiRobotConfig mc_;
  int cost_threshold_{253};
  double cbf_robot_radius_{0.0};  // se_cbf_robot_radius; <= 0: footprint circumscribed radius
  std::string predict_model_{"cv"};
  unsigned pending_reapply_{0};
  void reapplyConfig();

  // Static parameters that cannot be re-applied at runtime; the set callback
  // rejects them (MPPI's handler would otherwise report success, no effect).
  std::set<std::string> configure_only_params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reject_set_handle_;

  /// RViz introspection (se_viz param): per-obstacle CBF discs inflated by the
  /// conformal bound q, predicted horizons, and a status text (alpha / slack /
  /// entrapped / max q). What the live-run debugging always had to infer from
  /// logs, drawn in the costmap frame.
  void publishMarkers(
    const RobotState & state, const std::vector<TrackedObstacle> & obstacles,
    double alpha, double slack, bool entrapped);

  // Multi-SE-MPPI N2 (off by default: se_multirobot). Each configured
  // neighbor odom topic feeds a NeighborRobot slot (id = list index, the
  // fleet priority convention); the coordinator marks matching tracked
  // clusters with the reciprocal barrier-budget share and runs the
  // deadlock/priority state machine.
  bool multirobot_enabled_{false};
  int my_priority_id_{0};
  MultiRobotCoordinator multi_;
  std::vector<NeighborRobot> neighbors_;
  std::mutex neighbors_mutex_;  // odom callbacks (executor thread) vs the control loop
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
  neighbor_subs_;

  bool viz_enabled_{true};
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    viz_pub_;

  // Failure visibility. The control loop counts events; a 1 Hz wall timer
  // publishes them on /diagnostics (status "<node name>: se_mppi (<plugin>)"),
  // and the loop logs throttled warnings on a steady clock. None of this feeds
  // back into the command.
  struct Counters
  {
    std::atomic<std::uint64_t> cycles{0};
    std::atomic<std::uint64_t> forced_stops{0};    // filter forced v = 0
    std::atomic<std::uint64_t> qp_failures{0};     // QP setup/solve failed
    std::atomic<std::uint64_t> slack_active{0};    // barrier relaxed (slack > 0)
    std::atomic<std::uint64_t> tracks_dropped{0};  // tracks aged out unmatched
    std::atomic<std::uint64_t> escape_entries{0};
    std::atomic<bool> entrapped{false};
    std::atomic<int> cbf_obstacles{0};
    std::atomic<double> alpha{0.0};
  };
  Counters counters_;
  std::uint64_t reported_forced_stops_{0};  // diagnostics timer only
  std::uint64_t reported_qp_failures_{0};   // diagnostics timer only
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::string diag_name_;
  std::string diag_hardware_id_;
  void publishDiagnostics();

  // Single entrapment source of truth, shared with the EscapeCritic.
  std::shared_ptr<SharedEntrapment> shared_;
  std::size_t furthest_progress_{0};  // monotonic furthest reached path index
  // Same-goal replanning (Nav2's default tree replans at 1 Hz) keeps the task
  // state; only the progress index is re-anchored on the new path.
  bool has_goal_{false};
  std::string goal_frame_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  bool rebase_progress_{false};

  // The tracker advances only when the costmap content changed: the control
  // loop usually runs faster than the local costmap, and re-reading the same
  // grid with a new stamp gave moving obstacles zero velocity.
  std::vector<unsigned char> last_grid_;
  unsigned int last_grid_w_{0};
  unsigned int last_grid_h_{0};
  double last_grid_ox_{0.0};
  double last_grid_oy_{0.0};
  std::vector<TrackedObstacle> last_tracked_;
  double prev_stamp_{0.0};
  bool has_stamp_{false};
  bool prev_entrapped_{false};  // for ENTER/EXIT escape-mode transition logs
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__SAFE_ESCAPE_CONTROLLER_HPP_
