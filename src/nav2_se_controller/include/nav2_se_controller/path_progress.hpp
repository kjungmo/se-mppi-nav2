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

#ifndef NAV2_SE_CONTROLLER__PATH_PROGRESS_HPP_
#define NAV2_SE_CONTROLLER__PATH_PROGRESS_HPP_

#include <cstddef>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace nav2_se_controller
{

/**
 * @brief Index of the path pose nearest to (x, y) in the XY plane.
 *
 * Used as the controller-side global-path progress signal that drives the
 * entrapment detector: a non-increasing nearest index means the robot is not
 * advancing along the plan. Returns 0 for an empty path.
 */
std::size_t nearestPathIndex(const nav_msgs::msg::Path & path, double x, double y);

/// Tunables of PathProgressTracker.
struct ProgressConfig
{
  /// A progress event fires when the arc length at the robot's nearest point on
  /// the anchor path exceeds the best so far by this much (m). Two costmap cells.
  double min_advance{0.10};
  /// A replanned path becomes the anchor when the robot has advanced this far
  /// along the replans since the first of them (m). Nav2's default
  /// required_movement_radius.
  double adopt_advance{0.5};
};

/**
 * @class PathProgressTracker
 * @brief Progress of a robot along a task's path, robust to 1 Hz replanning.
 *
 * Nav2's behaviour tree replans a path that starts at the robot every second.
 * Measuring progress on each new path would count 0.025 m of motion (one path
 * cell) as progress. The tracker keeps an ANCHOR path per task (the first plan,
 * with cumulative arc length). A progress event fires when the arc length at
 * the robot's nearest anchor point exceeds the best so far by `min_advance`.
 * A same-goal replan is only a CANDIDATE: the signed advance of the robot along
 * the successive candidates (a chain, carried over replans) is measured, and at
 * `adopt_advance` the latest candidate becomes the anchor and an event fires.
 * A robot that cannot move 0.5 m along fresh paths never adopts one; a robot
 * that follows a legitimate detour adopts it within seconds.
 *
 * Pure (no ROS node), unit-tested. Feed events() to EntrapmentDetector::update.
 */
class PathProgressTracker
{
public:
  void configure(const ProgressConfig & cfg) {cfg_ = cfg;}

  /// Forget everything (no anchor, no events).
  void reset();

  /// A new task: the plan becomes the anchor. An empty plan clears the anchor.
  void startTask(const nav_msgs::msg::Path & plan);

  /// A same-goal replan: it becomes the candidate. An empty plan is ignored.
  void offerReplan(const nav_msgs::msg::Path & plan);

  /// Feed the robot position in the plan frame. Returns events(). A non-finite
  /// position, or a tracker without an anchor, changes nothing.
  std::size_t update(double x, double y);

  /// Monotone count of progress events (anchor advances and adoptions).
  std::size_t events() const {return events_;}
  /// Replans adopted as the anchor.
  std::size_t adoptions() const {return adoptions_;}
  /// Arc length (m) at the best nearest point on the anchor so far.
  double bestArc() const {return best_arc_;}
  /// Signed advance along the replan chain since the last event (m).
  double chainAdvance() const {return chain_;}
  bool hasAnchor() const {return !anchor_.arc.empty();}

private:
  struct Poly
  {
    std::vector<double> x, y, arc;
  };
  static Poly build(const nav_msgs::msg::Path & path);
  static double arcAtNearest(const Poly & poly, double x, double y);

  ProgressConfig cfg_{};
  Poly anchor_;
  Poly candidate_;
  bool candidate_pending_base_{false};  // base arc is taken at the next update
  double candidate_base_{0.0};
  double carried_{0.0};  // signed advance of the finished candidates in the chain
  double chain_{0.0};
  double best_arc_{0.0};
  bool has_best_{false};
  bool has_last_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  std::size_t events_{0};
  std::size_t adoptions_{0};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__PATH_PROGRESS_HPP_
