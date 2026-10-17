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
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tf2/LinearMath/Transform.h"
#include "nav2_mppi_controller/controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_se_controller/cbf_safety_filter.hpp"
#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"
#include "nav2_se_controller/entrapment_detector.hpp"
#include "nav2_se_controller/entrapment_state.hpp"
#include "nav2_se_controller/escape_safety_coordinator.hpp"
#include "nav2_se_controller/failure_streak.hpp"
#include "nav2_se_controller/mppi_rollout_check.hpp"
#include "nav2_se_controller/multi_robot_coordinator.hpp"
#include "nav2_se_controller/opening_search.hpp"
#include "nav2_se_controller/path_progress.hpp"
#include "nav2_se_controller/task_boundary.hpp"

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

  /// Nav2's speed filter: MPPI scales its limits, and the CBF box follows.
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
#ifdef NAV2_SE_CONTROLLER_HUMBLE_API
  void reset();  // Humble: not virtual in nav2_core::Controller
#else
  void reset() override;
#endif

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  /// The nominal command of the cycle: the base MPPI optimizer's output. The
  /// SE layer calls the base class by qualified name, so a test cannot
  /// intercept that call; it overrides this hook instead (to throw
  /// nav2_core::NoValidControl, or to script a command). The default forwards
  /// to MPPIController::computeVelocityCommands.
  virtual geometry_msgs::msg::TwistStamped nominalCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker);

  /// The path the base MPPI optimizer follows. The default forwards to
  /// MPPIController::setPlan. A test overrides it to capture what the
  /// optimizer is given.
  virtual void handNominalPath(const nav_msgs::msg::Path & path);

  /// Self-check of the loaded nav2_mppi_controller library's batch rollout,
  /// run once per configure() before anything is created. Virtual so that a
  /// test can stand in for a library with the defect.
  virtual RolloutCheckResult runRolloutCheck() const;
  // Written in configure() only (the diagnostics timer does not exist yet),
  // read by the diagnostics timer. se_enforce_rollout_check: true refuses to
  // configure on a library that fails the check; false runs and reports ERROR.
  bool enforce_rollout_check_{true};
  bool rollout_check_failed_{false};
  std::string rollout_failure_;  // describeRolloutFailure(); empty when the check passed

  /// The robot in the plan frame (progress, goal distance) and the goal in the
  /// robot frame. `ok` is false when the transform is missing.
  struct PlanFrameState
  {
    double plan_x{0.0};
    double plan_y{0.0};
    Eigen::Vector2d goal_xy{0.0, 0.0};
    bool ok{true};
  };
  PlanFrameState robotInPlanFrame(
    const geometry_msgs::msg::PoseStamped & robot_pose, const RobotState & state);
  /// Progress along the plan -> detector -> shared flag, ENTER/EXIT log and the
  /// cycle counters. Called with the parameter lock held, once per cycle, also
  /// in a cycle in which the optimizer found no valid control.
  bool updateEntrapment(const RobotState & state, const PlanFrameState & plan_frame);

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
    kConformal = 32u,  // the tracker's conformal calibration must restart
    kProgress = 64u,
    kAllConsumers = 127u
  };
  EntrapmentConfig ec_;
  CoordinationConfig cc_;
  CbfConfig fc_;              // alpha and robot_radius are filled in on re-apply
  TrackerConfig tc_;          // cost_threshold / predictor model derived on re-apply
  MultiRobotConfig mc_;
  ProgressConfig pc_;
  int cost_threshold_{253};
  double cbf_robot_radius_{0.0};  // se_cbf_robot_radius; <= 0: footprint circumscribed radius
  double speed_limit_ratio_{1.0};  // Nav2 speed limit applied to the CBF box
  std::string predict_model_{"cv"};
  unsigned pending_reapply_{0};
  bool config_applied_once_{false};  // warn on calibration resets after configure
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
    std::atomic<std::uint64_t> base_failures{0};   // optimizer threw NoValidControl
    std::atomic<std::uint64_t> escape_hops{0};     // escape waypoints selected
    std::atomic<double> escape_bearing{0.0};       // last selected bearing (rad)
    std::atomic<double> escape_clearance{0.0};     // its clearance (m)
    std::atomic<bool> entrapped{false};
    std::atomic<int> cbf_obstacles{0};
    std::atomic<double> alpha{0.0};
  };
  Counters counters_;
  std::uint64_t reported_forced_stops_{0};  // diagnostics timer only
  std::uint64_t reported_qp_failures_{0};   // diagnostics timer only
  // Continuous CBF QP failure (forced v = 0 every cycle) for longer than this
  // turns the status ERROR. 2 s is 40 cycles at Nav2's default 20 Hz: no
  // longer a transient, and well inside the default progress-checker
  // allowance (10 s), so the operator sees the cause before the goal aborts.
  static constexpr double kQpFailureErrorAfterSec = 2.0;
  // A streak stays alive only while cycles keep coming at least this often.
  static constexpr double kQpFailureStaleSec = 0.5;
  FailureStreak qp_failure_streak_;
  std::atomic<bool> se_enabled_mirror_{true};  // se_enabled_ for the timer, lock-free
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::string diag_name_;
  std::string diag_hardware_id_;
  void publishDiagnostics();

  // Single entrapment source of truth, shared with the EscapeCritic.
  std::shared_ptr<SharedEntrapment> shared_;
  // Progress along the task's anchor path; a same-goal replan (Nav2's default
  // tree replans at 1 Hz) keeps the task state and is only a candidate anchor.
  PathProgressTracker progress_;
  bool has_goal_{false};
  std::string goal_frame_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  // A plan after the control loop was idle longer than se_task_idle_threshold
  // (>= 3 control periods) starts a new task even for the same goal: Humble's
  // controller_server never calls reset(), so a retry would otherwise inherit
  // the previous task's state.
  TaskBoundary task_boundary_;
  double task_idle_threshold_{1.0};
  double controller_frequency_{20.0};

  // The tracker advances only when the costmap content changed: the control
  // loop usually runs faster than the local costmap, and re-reading the same
  // grid with a new stamp gave moving obstacles zero velocity.
  std::vector<unsigned char> last_grid_;
  unsigned int last_grid_w_{0};
  unsigned int last_grid_h_{0};
  double last_grid_ox_{0.0};
  double last_grid_oy_{0.0};
  std::vector<TrackedObstacle> last_tracked_;
  // A grid that stays identical for longer than this is fed to the tracker
  // anyway (velocities decay to zero, tracks age, static evidence builds), so
  // a frozen costmap never keeps a stale velocity in the CBF.
  // se_tracker_stale_grid_timeout; <= 0: two local-costmap update periods.
  double stale_grid_timeout_param_{0.0};
  double stale_grid_timeout_{0.4};
  double last_tracker_stamp_{0.0};
  double prev_stamp_{0.0};
  bool has_stamp_{false};
  bool prev_entrapped_{false};  // for ENTER/EXIT escape-mode transition logs

  // ---- Escape path substitution ------------------------------------------
  // While entrapped, MPPI is handed a short path from the robot along the
  // bearing of the opening chosen by the horizon-ray rule (gap_search.hpp), in
  // place of the behaviour tree's path, which leads through the obstacle. The
  // optimizer's path and goal critics then pull toward the waypoint. The BT
  // path is kept in global_plan_ (progress, near-goal test), is not forwarded
  // to MPPI while the substitution is active, and is forwarded again when the
  // escape ends, the controller is reset, a new task starts, or se_enabled
  // becomes false. controller_server checks goal arrival against its own copy
  // of the BT path, so the substitution cannot fake a goal.
  struct EscapeState
  {
    bool active{false};       // MPPI currently holds the escape path
    double wx{0.0};           // waypoint, costmap frame
    double wy{0.0};
    double bearing{0.0};      // selected ray (rad, costmap frame)
    double clearance{0.0};    // its free length (m)
    int cycles{0};            // control cycles since the selection
    int hop{0};               // selections in this escape episode
    bool has_prev{false};     // hysteresis memory
    double prev_bearing{0.0};
    bool has_stall{false};    // where entrapment was declared, costmap frame
    double stall_x{0.0};
    double stall_y{0.0};
    // Where the recent hops started (costmap frame), oldest first.
    std::vector<std::pair<double, double>> origins;
    // Waypoints that were not reached (timeout, blocked, no movement).
    std::vector<std::pair<double, double>> failed;
    // Movement watchdog: the position at the last 0.15 m of movement.
    double ref_x{0.0};
    double ref_y{0.0};
    int ref_cycles{0};
  };
  EscapeState escape_;
  bool escape_substitution_{true};      // se_escape_path_substitution
  double escape_waypoint_cap_{2.5};     // se_escape_waypoint_cap (m)
  double escape_reselect_radius_{0.4};  // se_escape_reselect_radius (m)
  double escape_reselect_period_{15.0};  // se_escape_reselect_period (s)
  double escape_stall_time_{3.0};       // se_escape_stall_time (s): no movement -> re-select
  double escape_stall_distance_{0.15};  // se_escape_stall_distance (m)
  double escape_hysteresis_{0.5};       // se_escape_hysteresis (rad)
  double escape_avoid_radius_{1.0};     // se_escape_avoid_radius (m): the stall pose
  double escape_avoid_hop_radius_{0.6};  // se_escape_avoid_hop_radius (m): recent hop origins
  int escape_avoid_hops_{3};            // se_escape_avoid_hops: how many recent origins
  int gap_num_rays_{36};                // se_gap_num_rays
  double gap_max_range_{2.0};           // se_gap_max_range (m)
  double gap_min_clearance_{0.6};       // se_gap_min_clearance (m)
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr escape_path_pub_;

  /// Before the base call: hand MPPI the escape path while entrapped (select
  /// or re-select the waypoint), or give it the behaviour tree's path back.
  void manageEscapePath(const geometry_msgs::msg::PoseStamped & robot_pose);
  /// The plan-frame transform of a costmap-frame point; false when the
  /// transform is missing.
  bool planFromRobot(
    const geometry_msgs::msg::PoseStamped & robot_pose, tf2::Transform & out) const;

  // se_enabled with vx_min > 0: the robot cannot stop or reverse, so the
  // escape cannot act (campaign 1, set P). Warned at configure and on every
  // change; reported on /diagnostics.
  std::atomic<double> vx_min_param_{-0.35};
  bool vxMinBlocksReversal() const
  {
    return se_enabled_mirror_.load() && vx_min_param_.load() > 0.0;
  }
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__SAFE_ESCAPE_CONTROLLER_HPP_
