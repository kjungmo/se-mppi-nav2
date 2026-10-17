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

#include "nav2_se_controller/opening_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_se_controller
{

namespace
{
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

inline double angleDiff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}
}  // namespace

// Free distance along a bearing; `horizon` is set when the ray ran to
// max_range or off the map (free up to the edge) instead of meeting a cell.
double rayFreeDistance(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry,
  double bearing, double max_range, unsigned char cost_threshold, bool * horizon)
{
  const double res = costmap.getResolution();
  // Sample at half-resolution so a one-cell-thick wall on a diagonal bearing is
  // not stepped over (full-res Euclidean steps can skip the cell between two
  // diagonal samples and report a blocked bearing as free).
  const double step = std::max(0.5 * res, 1e-3);
  const double cx = std::cos(bearing);
  const double cy = std::sin(bearing);
  if (horizon != nullptr) {
    *horizon = false;
  }
  for (double r = step; r <= max_range; r += step) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(rx + r * cx, ry + r * cy, mx, my)) {
      if (horizon != nullptr) {
        *horizon = true;
      }
      return r;  // ran off the map: free up to the edge
    }
    if (costmap.getCost(mx, my) >= cost_threshold) {
      return r;
    }
  }
  if (horizon != nullptr) {
    *horizon = true;
  }
  return max_range;
}

std::vector<GapRay> castGapRays(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry,
  int num_rays, double max_range, unsigned char cost_threshold)
{
  const int rays = std::max(num_rays, 1);
  std::vector<GapRay> out;
  out.reserve(static_cast<std::size_t>(rays));
  for (int k = 0; k < rays; ++k) {
    GapRay ray;
    ray.bearing = -0.5 * kTwoPi + (kTwoPi * k) / rays;  // [-pi, pi)
    ray.clearance = rayFreeDistance(
      costmap, rx, ry, ray.bearing, max_range, cost_threshold, &ray.horizon);
    out.push_back(ray);
  }
  return out;
}

namespace
{
// True if the ray leads back to the avoided place.
bool leadsToAvoided(const GapRay & ray, const std::vector<AvoidPoint> & avoid)
{
  for (const auto & a : avoid) {
    if (a.distance <= a.radius) {
      continue;  // the robot is at the place
    }
    const double delta = std::abs(angleDiff(ray.bearing, a.bearing));
    if (delta >= kTwoPi / 4.0) {  // not ahead (90 degrees or more off)
      continue;
    }
    // The place lies beyond the end of the ray by more than the radius: not reached.
    if (a.distance > ray.clearance + a.radius) {
      continue;
    }
    if (a.distance * std::sin(delta) < a.radius) {
      return true;
    }
  }
  return false;
}

HorizonGap selectOnce(
  const std::vector<GapRay> & rays, double goal_bearing, double min_clearance,
  const std::optional<double> & previous_bearing, double hysteresis,
  const std::vector<AvoidPoint> & avoid)
{
  HorizonGap out;
  if (rays.empty()) {
    return out;
  }
  // Candidates: rays that reach the horizon with a usable clearance.
  const GapRay * best = nullptr;
  double best_align = std::numeric_limits<double>::max();
  std::size_t candidates = 0;
  for (const auto & ray : rays) {
    if (!ray.horizon || ray.clearance < min_clearance || leadsToAvoided(ray, avoid)) {
      continue;
    }
    ++candidates;
    const double align = std::abs(angleDiff(ray.bearing, goal_bearing));
    if (align < best_align) {
      best_align = align;
      best = &ray;
    }
  }
  out.candidates = candidates;
  if (best == nullptr) {
    // None reaches the horizon: take the longest ray (ties: closest to the goal bearing).
    const GapRay * longest = nullptr;
    double longest_align = std::numeric_limits<double>::max();
    for (const auto & ray : rays) {
      if (leadsToAvoided(ray, avoid)) {
        continue;
      }
      const double align = std::abs(angleDiff(ray.bearing, goal_bearing));
      if (longest == nullptr || ray.clearance > longest->clearance + 1e-9 ||
        (std::abs(ray.clearance - longest->clearance) <= 1e-9 && align < longest_align))
      {
        longest = &ray;
        longest_align = align;
      }
    }
    if (longest == nullptr || longest->clearance < min_clearance) {
      return out;  // nothing usable
    }
    out.found = true;
    out.bearing = longest->bearing;
    out.clearance = longest->clearance;
    out.horizon = false;
    return out;
  }
  // Hysteresis: keep the previous bearing unless the best is clearly closer to the goal.
  if (previous_bearing.has_value()) {
    const GapRay * prev = nullptr;
    double prev_diff = std::numeric_limits<double>::max();
    for (const auto & ray : rays) {
      const double d = std::abs(angleDiff(ray.bearing, *previous_bearing));
      if (d < prev_diff) {
        prev_diff = d;
        prev = &ray;
      }
    }
    if (prev != nullptr && prev->horizon && prev->clearance >= min_clearance &&
      !leadsToAvoided(*prev, avoid))
    {
      const double prev_align = std::abs(angleDiff(prev->bearing, goal_bearing));
      if (!(best_align < prev_align - hysteresis)) {
        out.found = true;
        out.bearing = prev->bearing;
        out.clearance = prev->clearance;
        out.horizon = true;
        out.kept_previous = true;
        return out;
      }
    }
  }
  out.found = true;
  out.bearing = best->bearing;
  out.clearance = best->clearance;
  out.horizon = true;
  return out;
}
}  // namespace

HorizonGap selectHorizonGap(
  const std::vector<GapRay> & rays, double goal_bearing, double min_clearance,
  const std::optional<double> & previous_bearing, double hysteresis,
  const std::vector<AvoidPoint> & avoid)
{
  HorizonGap out = selectOnce(rays, goal_bearing, min_clearance, previous_bearing, hysteresis, avoid);
  if (!out.found && !avoid.empty()) {
    out = selectOnce(rays, goal_bearing, min_clearance, previous_bearing, hysteresis, {});
  }
  return out;
}

HorizonGap findHorizonGap(
  const nav2_costmap_2d::Costmap2D & costmap, double rx, double ry, double goal_bearing,
  int num_rays, double max_range, double min_clearance,
  const std::optional<double> & previous_bearing, double hysteresis,
  const std::vector<AvoidPoint> & avoid, unsigned char cost_threshold)
{
  return selectHorizonGap(
    castGapRays(costmap, rx, ry, num_rays, max_range, cost_threshold),
    goal_bearing, min_clearance, previous_bearing, hysteresis, avoid);
}

bool segmentBlocked(
  const nav2_costmap_2d::Costmap2D & costmap, double x0, double y0, double x1, double y1,
  unsigned char cost_threshold)
{
  const double len = std::hypot(x1 - x0, y1 - y0);
  const double step = std::max(0.5 * costmap.getResolution(), 1e-3);
  for (double r = step; r <= len; r += step) {
    const double t = r / len;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap.worldToMap(x0 + t * (x1 - x0), y0 + t * (y1 - y0), mx, my)) {
      continue;
    }
    if (costmap.getCost(mx, my) >= cost_threshold) {
      return true;
    }
  }
  return false;
}

}  // namespace nav2_se_controller
