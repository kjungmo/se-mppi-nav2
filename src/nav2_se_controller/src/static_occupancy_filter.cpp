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

#include "nav2_se_controller/static_occupancy_filter.hpp"

#include <cmath>

namespace nav2_se_controller
{

namespace
{
constexpr unsigned char kNoInformation = 255;  // nav2_costmap_2d::NO_INFORMATION
}  // namespace

std::uint64_t StaticOccupancyFilter::key(double wx, double wy) const
{
  // Quantize world coordinates onto a fixed lattice of the costmap resolution.
  // Anchored to the world origin, not the (rolling) costmap origin, so the
  // same wall cell keeps the same key as the window moves with the robot.
  const auto ix = static_cast<std::int64_t>(std::floor(wx / resolution_));
  const auto iy = static_cast<std::int64_t>(std::floor(wy / resolution_));
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(iy));
}

void StaticOccupancyFilter::update(const nav2_costmap_2d::Costmap2D & costmap)
{
  if (resolution_ <= 0.0) {
    resolution_ = costmap.getResolution();
  }
  ++frame_;

  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      const unsigned char cost = costmap.getCost(mx, my);
      if (cost == kNoInformation) {
        continue;  // not an observation either way
      }
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);
      const std::uint64_t k = key(wx, wy);

      if (cost >= cfg_.cost_threshold) {
        auto & cell = grid_[k];  // upsert: occupied cells enter the grid
        cell.evidence = std::min(cell.evidence + 1, cfg_.evidence_cap);
        cell.last_seen = frame_;
      } else {
        // Observed free: decay only cells we already track (keeps the grid
        // bounded by the occupied set, not the whole map).
        auto it = grid_.find(k);
        if (it != grid_.end()) {
          it->second.last_seen = frame_;
          if (--it->second.evidence <= 0) {
            grid_.erase(it);
          }
        }
      }
    }
  }

  // Periodic prune of cells that left the rolling window long ago.
  if (cfg_.stale_prune_frames > 0 &&
    frame_ % static_cast<std::uint32_t>(cfg_.stale_prune_frames) == 0)
  {
    const std::uint32_t cutoff =
      frame_ > static_cast<std::uint32_t>(cfg_.stale_prune_frames) ?
      frame_ - static_cast<std::uint32_t>(cfg_.stale_prune_frames) : 0;
    for (auto it = grid_.begin(); it != grid_.end(); ) {
      if (it->second.last_seen < cutoff) {
        it = grid_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

bool StaticOccupancyFilter::isStatic(double wx, double wy) const
{
  if (resolution_ <= 0.0) {
    return false;
  }
  const auto it = grid_.find(key(wx, wy));
  return it != grid_.end() && it->second.evidence >= cfg_.static_min_frames;
}

double StaticOccupancyFilter::staticFraction(
  const std::vector<Eigen::Vector2d> & points) const
{
  if (points.empty()) {
    return 0.0;
  }
  int n_static = 0;
  for (const auto & p : points) {
    if (isStatic(p.x(), p.y())) {
      ++n_static;
    }
  }
  return static_cast<double>(n_static) / static_cast<double>(points.size());
}

}  // namespace nav2_se_controller
