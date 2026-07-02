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

#include "nav2_se_controller/gap_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_se_controller
{

namespace
{
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

/// Signed smallest angle a - b wrapped to [-pi, pi].
inline double angleDiff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

/// Free distance (m) from (rx, ry) along `bearing` until an obstacle / edge / max.
double rayFreeDistance(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry,
  double bearing, double max_range, unsigned char cost_threshold)
{
  const double res = costmap.getResolution();
  // Sample at half-resolution so a one-cell-thick wall on a diagonal bearing is
  // not stepped over (full-res Euclidean steps can skip the cell between two
  // diagonal samples and report a blocked bearing as free).
  const double step = std::max(0.5 * res, 1e-3);
  const double cx = std::cos(bearing);
  const double cy = std::sin(bearing);
  for (double r = step; r <= max_range; r += step) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(rx + r * cx, ry + r * cy, mx, my)) {
      return r;  // ran off the map: free up to the edge
    }
    if (costmap.getCost(mx, my) >= cost_threshold) {
      return r;
    }
  }
  return max_range;
}
}  // namespace

EscapeGap findEscapeGap(
  const nav2_costmap_2d::Costmap2D & costmap,
  double rx, double ry, double goal_bearing,
  int num_rays, double max_range, double min_clearance,
  unsigned char cost_threshold)
{
  EscapeGap best;
  double best_align = std::numeric_limits<double>::max();
  const int rays = std::max(num_rays, 1);

  for (int k = 0; k < rays; ++k) {
    const double bearing = -0.5 * kTwoPi + (kTwoPi * k) / rays;  // [-pi, pi)
    const double clearance =
      rayFreeDistance(costmap, rx, ry, bearing, max_range, cost_threshold);
    if (clearance < min_clearance) {
      continue;
    }
    const double align = std::abs(angleDiff(bearing, goal_bearing));
    if (align < best_align) {
      best_align = align;
      best.found = true;
      best.bearing = bearing;
      best.clearance = clearance;
    }
  }

  return best;
}

xt::xtensor<float, 1> computeGapAttractionCosts(
  const xt::xtensor<float, 2> & traj_x,
  const xt::xtensor<float, 2> & traj_y,
  double rx, double ry, double gap_bearing,
  float weight, int power)
{
  const std::size_t batch_size = traj_x.shape(0);
  const std::size_t time_steps = traj_x.shape(1);
  xt::xtensor<float, 1> out = xt::zeros<float>({batch_size});
  if (time_steps == 0) {
    return out;
  }
  const std::size_t last = time_steps - 1;
  const int p = std::max(power, 1);

  for (std::size_t i = 0; i < batch_size; ++i) {
    const double dx = static_cast<double>(traj_x(i, last)) - rx;
    const double dy = static_cast<double>(traj_y(i, last)) - ry;
    const double ep_bearing = std::atan2(dy, dx);
    const double delta = angleDiff(ep_bearing, gap_bearing);
    // (1 - cos(delta)) in [0, 2]: 0 aligned, 2 opposite.
    const float term = static_cast<float>(1.0 - std::cos(delta));
    out(i) = weight * std::pow(term, static_cast<float>(p));
  }

  return out;
}

}  // namespace nav2_se_controller
