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

#include "nav2_se_controller/multi_robot_coordinator.hpp"

#include <algorithm>
#include <vector>

namespace nav2_se_controller
{

int MultiRobotCoordinator::markNeighbors(
  std::vector<TrackedObstacle> & obstacles,
  const std::vector<NeighborRobot> & neighbors) const
{
  int marked = 0;
  const double r2 = cfg_.match_radius * cfg_.match_radius;
  const double lambda = lambdaForRole();
  for (auto & o : obstacles) {
    for (const auto & n : neighbors) {
      if (!n.valid) {
        continue;
      }
      if ((o.position - n.position).squaredNorm() <= r2) {
        o.responsibility = lambda;
        ++marked;
        break;
      }
    }
  }
  return marked;
}

MultiRobotCoordinator::Role MultiRobotCoordinator::update(
  int my_id, const RobotState & state, const Eigen::Vector2d & goal,
  double my_speed, bool entrapped,
  const std::vector<NeighborRobot> & neighbors)
{
  const Eigen::Vector2d pos(state.x, state.y);

  if (role_ == Role::kNone) {
    if (entrapped && std::abs(my_speed) < cfg_.deadlock_speed) {
      // A mutual deadlock needs a stalled neighbor blocking nearby. Priority
      // is the deterministic id convention: the LOWEST id among the stalled
      // parties passes, everyone else yields.
      bool found = false;
      bool i_win = true;
      for (const auto & n : neighbors) {
        if (!n.valid) {
          continue;
        }
        if ((n.position - pos).norm() < cfg_.deadlock_range &&
          n.velocity.norm() < cfg_.deadlock_speed)
        {
          found = true;
          if (n.id < my_id) {
            i_win = false;
          }
        }
      }
      if (found) {
        role_ = i_win ? Role::kPass : Role::kYield;
      }
    }
  } else if (conflictCleared(state, goal, neighbors)) {
    role_ = Role::kNone;
  }
  return role_;
}

bool MultiRobotCoordinator::conflictCleared(
  const RobotState & state, const Eigen::Vector2d & goal,
  const std::vector<NeighborRobot> & neighbors) const
{
  const Eigen::Vector2d pos(state.x, state.y);
  const Eigen::Vector2d to_goal = goal - pos;
  const double n = to_goal.norm();
  if (n < 1.0e-6) {
    return true;
  }
  const Eigen::Vector2d fwd = to_goal / n;
  for (const auto & nb : neighbors) {
    if (!nb.valid) {
      continue;
    }
    const Eigen::Vector2d rel = nb.position - pos;
    if (rel.norm() < cfg_.clear_range && rel.dot(fwd) > 0.0) {
      return false;  // still a neighbor ahead within conflict range
    }
  }
  return true;
}

}  // namespace nav2_se_controller
