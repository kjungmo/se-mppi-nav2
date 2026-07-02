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

#include <memory>
#include <string>
#include <vector>

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
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
  neighbor_subs_;

  bool viz_enabled_{true};
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    viz_pub_;

  // Single entrapment source of truth, shared with the EscapeCritic.
  std::shared_ptr<SharedEntrapment> shared_;
  std::size_t furthest_progress_{0};  // monotonic furthest reached path index
  double prev_stamp_{0.0};
  bool has_stamp_{false};
  bool prev_entrapped_{false};  // for ENTER/EXIT escape-mode transition logs
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__SAFE_ESCAPE_CONTROLLER_HPP_
