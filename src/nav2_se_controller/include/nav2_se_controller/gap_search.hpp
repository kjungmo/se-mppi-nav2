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

#include <optional>
#include <vector>

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

/// One ray of the opening search.
struct GapRay
{
  double bearing{0.0};    ///< world-frame bearing (rad), in [-pi, pi).
  double clearance{0.0};  ///< free distance along the bearing (m).
  /// The ray reached the sensing horizon: it ran to `max_range`, or to the
  /// costmap edge, without meeting a cell at or above the cost threshold.
  /// Unknown cells (255) stop a ray.
  bool horizon{false};
};

/// Cast `num_rays` evenly spaced rays from (rx, ry) (bearings -pi + 2 pi k / n).
std::vector<GapRay> castGapRays(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry,
  int num_rays, double max_range, unsigned char cost_threshold = 253);

/// A place the escape must not lead back to (the pose where entrapment was
/// declared), in polar form relative to the robot.
struct AvoidPoint
{
  double bearing{0.0};   ///< bearing from the robot to the place (rad).
  double distance{0.0};  ///< distance from the robot to the place (m).
  double radius{0.6};    ///< rays passing closer than this are excluded (m).
};

/// Result of the horizon-ray selection.
struct HorizonGap
{
  bool found{false};
  double bearing{0.0};
  double clearance{0.0};
  bool horizon{false};         ///< the selected ray reaches the horizon
  bool kept_previous{false};   ///< hysteresis kept the previous bearing
  std::size_t candidates{0};   ///< number of horizon rays offered
};

/**
 * @brief Select the opening a trapped robot should head for.
 *
 * Among the rays that reach the sensing horizon (and clear `min_clearance`),
 * choose the one closest to the goal bearing; if none reaches the horizon, take
 * the longest ray. A plain "longest ray" rule fails at the mouth of a pocket,
 * where the ray back into the pocket is longer than the rays to the window edge.
 * Hysteresis: if `previous_bearing` is given and its ray is still a candidate,
 * keep it unless the best ray is closer to the goal bearing by more than
 * `hysteresis` (rad).
 *
 * `avoid` (optional): rays that pass within `avoid->radius` of the avoided
 * place, ahead of the robot and within the ray's length, are not candidates
 * (from the mouth of a pocket, the ray into the pocket reaches the window edge
 * and would send the robot back to the trap). It is not applied when the robot
 * is within `radius` of the place. If excluding rays leaves nothing, the
 * selection is repeated without the exclusion.
 */
HorizonGap selectHorizonGap(
  const std::vector<GapRay> & rays, double goal_bearing, double min_clearance,
  const std::optional<double> & previous_bearing, double hysteresis,
  const std::optional<AvoidPoint> & avoid = std::nullopt);

/// castGapRays + selectHorizonGap.
HorizonGap findHorizonGap(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry, double goal_bearing,
  int num_rays, double max_range, double min_clearance,
  const std::optional<double> & previous_bearing, double hysteresis,
  const std::optional<AvoidPoint> & avoid = std::nullopt, unsigned char cost_threshold = 253);

/// True if a cell at or above `cost_threshold` lies on the segment from (x0, y0)
/// to (x1, y1), sampled at half the costmap resolution. The start point is not
/// tested. Points off the map do not block.
bool segmentBlocked(
  const nav2_costmap_2d::Costmap2D & costmap, double x0, double y0, double x1, double y1,
  unsigned char cost_threshold = 253);

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
