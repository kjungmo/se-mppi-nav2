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

#ifndef NAV2_SE_CONTROLLER__TASK_BOUNDARY_HPP_
#define NAV2_SE_CONTROLLER__TASK_BOUNDARY_HPP_

#include <algorithm>
#include <cmath>

namespace nav2_se_controller
{

/**
 * @class TaskBoundary
 * @brief Decides whether a plan starts a new task from how long the control
 *        loop has been idle, independent of the ROS distribution.
 *
 * Jazzy's controller_server calls the controller's reset() when a FollowPath
 * goal ends; Humble's never does. A plan that arrives after the loop has been
 * idle (no computeVelocityCommands for longer than the threshold) is a new
 * task even if its goal equals the previous one (a retry after a failed or
 * cancelled goal), so per-task state must not carry over. Times are seconds
 * on a steady clock.
 */
class TaskBoundary
{
public:
  static constexpr double kMinPeriods = 3.0;

  /// threshold_s is raised to at least three control periods, so an ordinary
  /// slow cycle is never mistaken for an idle loop.
  void configure(double threshold_s, double controller_frequency_hz)
  {
    const double period = std::isfinite(controller_frequency_hz) &&
      controller_frequency_hz > 0.0 ? 1.0 / controller_frequency_hz : 0.05;
    effective_s_ = std::max(threshold_s, kMinPeriods * period);
  }

  double effectiveThreshold() const {return effective_s_;}

  /// Call when computeVelocityCommands RETURNS (a call blocked on a lock is
  /// not idle time).
  void controlReturned(double now_s)
  {
    last_return_s_ = now_s;
    has_return_ = true;
  }

  /// A plan arrived at now_s: true when the loop was idle long enough for it
  /// to start a new task. Nothing is reset here; the caller acts on it.
  bool newPlanStartsTask(double now_s) const
  {
    return has_return_ && now_s - last_return_s_ > effective_s_;
  }

private:
  double effective_s_{0.15};
  double last_return_s_{0.0};
  bool has_return_{false};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__TASK_BOUNDARY_HPP_
