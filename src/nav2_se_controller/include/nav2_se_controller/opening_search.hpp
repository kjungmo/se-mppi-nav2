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

#ifndef NAV2_SE_CONTROLLER__OPENING_SEARCH_HPP_
#define NAV2_SE_CONTROLLER__OPENING_SEARCH_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_se_controller
{

/// Free distance along `bearing` from (rx, ry): until a cell at or above
/// `cost_threshold`, `max_range`, or the costmap edge. `horizon` (optional
/// out) is set when the ray ran to max_range or off the map instead of meeting
/// a cell. Unknown cells (255) stop a ray.
double rayFreeDistance(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry,
  double bearing, double max_range, unsigned char cost_threshold, bool * horizon = nullptr);

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
/// declared, or where a recent hop started), in polar form relative to the robot.
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
 * `avoid`: rays that pass within `radius` of an avoided place, ahead of the
 * robot and within the ray's length, are not candidates (from the mouth of a
 * pocket, the ray into the pocket reaches the window edge and would send the
 * robot back to the trap; a ray back to the previous hop origin makes the
 * robot swing between two places). A place is ignored while the robot is
 * within its `radius`. If excluding rays leaves nothing, the selection is
 * repeated without the exclusion.
 */
HorizonGap selectHorizonGap(
  const std::vector<GapRay> & rays, double goal_bearing, double min_clearance,
  const std::optional<double> & previous_bearing, double hysteresis,
  const std::vector<AvoidPoint> & avoid = {});

/// castGapRays + selectHorizonGap.
HorizonGap findHorizonGap(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry, double goal_bearing,
  int num_rays, double max_range, double min_clearance,
  const std::optional<double> & previous_bearing, double hysteresis,
  const std::vector<AvoidPoint> & avoid = {}, unsigned char cost_threshold = 253);

/// True if a cell at or above `cost_threshold` lies on the segment from (x0, y0)
/// to (x1, y1), sampled at half the costmap resolution. The start point is not
/// tested. Points off the map do not block.
bool segmentBlocked(
  const nav2_costmap_2d::Costmap2D & costmap, double x0, double y0, double x1, double y1,
  unsigned char cost_threshold = 253);

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__OPENING_SEARCH_HPP_
