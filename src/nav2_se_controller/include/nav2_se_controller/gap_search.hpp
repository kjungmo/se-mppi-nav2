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

#ifndef NAV2_SE_CONTROLLER__GAP_SEARCH_HPP_
#define NAV2_SE_CONTROLLER__GAP_SEARCH_HPP_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#pragma GCC diagnostic pop

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_se_controller
{

/// Result of a free-space gap search.
struct EscapeGap
{
  bool found{false};
  double bearing{0.0};     ///< world-frame bearing toward the opening (rad).
  double clearance{0.0};   ///< free distance along that bearing (m).
};

/**
 * @brief Find the free-space opening (gap) best aligned with the goal direction.
 *
 * Casts num_rays evenly-spaced rays from (rx, ry) over the costmap, measuring the
 * free distance along each (until a cell with cost >= cost_threshold, the map
 * edge, or max_range). A ray is a viable gap if its free distance >= min_clearance.
 * Returns the viable gap whose bearing is closest to goal_bearing — i.e. the
 * opening that lets the robot escape a local-minimum trap while still heading
 * roughly toward the goal. `found` is false if no ray clears min_clearance.
 *
 * Pure (costmap-only), unit-tested. Powers the temporary escape subgoal (M1.x).
 */
EscapeGap findEscapeGap(
  const nav2_costmap_2d::Costmap2D & costmap,
  double rx, double ry, double goal_bearing,
  int num_rays, double max_range, double min_clearance,
  unsigned char cost_threshold = 253);

/**
 * @brief Per-trajectory attraction toward a gap bearing.
 *
 * Rewards (low cost) trajectories whose endpoint, seen from (rx, ry), points
 * along gap_bearing: cost_i = weight * (1 - cos(delta))^power, where delta is the
 * angle between the endpoint bearing and gap_bearing (0 when perfectly aligned,
 * 2*weight when opposite). Added to the MPPI cost only while entrapped, it biases
 * the optimizer toward the detected opening. Returns an array of length batch_size.
 */
xt::xtensor<float, 1> computeGapAttractionCosts(
  const xt::xtensor<float, 2> & traj_x,
  const xt::xtensor<float, 2> & traj_y,
  double rx, double ry, double gap_bearing,
  float weight, int power);

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__GAP_SEARCH_HPP_
