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

#include "nav2_se_controller/escape_safety_coordinator.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_se_controller
{

double minTimeToCollision(
  const RobotState & state, double v,
  const std::vector<TrackedObstacle> & obstacles, double robot_radius)
{
  constexpr double kInf = std::numeric_limits<double>::infinity();
  constexpr double kEps = 1.0e-6;

  const Eigen::Vector2d p_robot(state.x, state.y);
  const Eigen::Vector2d v_robot(v * std::cos(state.yaw), v * std::sin(state.yaw));

  double min_ttc = kInf;
  for (const auto & o : obstacles) {
    const Eigen::Vector2d rel = o.position - p_robot;       // robot -> obstacle
    const Eigen::Vector2d rel_vel = o.velocity - v_robot;   // closing if pointing inward
    const double range = rel.norm();
    if (range < kEps) {
      return 0.0;  // coincident
    }
    const double clearance = range - (robot_radius + o.radius);
    if (clearance <= 0.0) {
      return 0.0;  // already in contact
    }
    // Closing speed: positive when the gap is shrinking.
    const double closing = -rel.dot(rel_vel) / range;
    if (closing <= kEps) {
      continue;  // separating or parallel
    }
    min_ttc = std::min(min_ttc, clearance / closing);
  }
  return min_ttc;
}

}  // namespace nav2_se_controller
