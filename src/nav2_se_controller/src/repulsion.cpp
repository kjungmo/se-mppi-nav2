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

#include "nav2_se_controller/repulsion.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>
#include <vector>

namespace nav2_se_controller
{

namespace
{
constexpr unsigned char kNoInformation = 255;  // nav2_costmap_2d::NO_INFORMATION

/// Occupied == at/above threshold but NOT unknown: unknown space must not act
/// as a repulsive obstacle (it would push escape away from unexplored regions).
inline bool isObstacle(unsigned char cost, unsigned char threshold)
{
  return cost >= threshold && cost != kNoInformation;
}
}  // namespace

xt::xtensor<float, 1> computeRepulsionCosts(
  const xt::xtensor<float, 2> & traj_x,
  const xt::xtensor<float, 2> & traj_y,
  const nav2_costmap_2d::Costmap2D & costmap,
  const float weight,
  const int power)
{
  const std::size_t batch_size = traj_x.shape(0);
  const std::size_t time_steps = traj_x.shape(1);
  xt::xtensor<float, 1> out = xt::zeros<float>({batch_size});

  // 254 == LETHAL_OBSTACLE; normalise costmap values into [0, 1].
  constexpr float kMaxCost = 254.0f;

  for (std::size_t i = 0; i < batch_size; ++i) {
    float accum = 0.0f;
    std::size_t counted = 0;
    for (std::size_t t = 0; t < time_steps; ++t) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap.worldToMap(traj_x(i, t), traj_y(i, t), mx, my)) {
        continue;  // off-map: do not penalise horizon length
      }
      const unsigned char raw = costmap.getCost(mx, my);
      const float cost = (raw == kNoInformation) ? 0.0f : static_cast<float>(raw);
      accum += std::min(cost, kMaxCost) / kMaxCost;
      ++counted;
    }
    if (counted == 0) {
      continue;
    }
    const float mean_cost = accum / static_cast<float>(counted);
    out(i) = weight * std::pow(mean_cost, static_cast<float>(std::max(power, 1)));
  }

  return out;
}

std::vector<float> obstacleDistanceField(
  const nav2_costmap_2d::Costmap2D & costmap,
  const unsigned char cost_threshold, const float max_dist)
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const float res = static_cast<float>(costmap.getResolution());
  const std::size_t n = static_cast<std::size_t>(size_x) * size_y;

  std::vector<float> dist(n, max_dist);
  if (n == 0) {
    return dist;
  }

  // Min-heap of (distance, cell index).
  using Node = std::pair<float, std::size_t>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  const auto idx = [size_x](unsigned int x, unsigned int y) {
    return static_cast<std::size_t>(y) * size_x + x;
  };

  // Seed all occupied cells at distance 0.
  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      if (isObstacle(costmap.getCost(x, y), cost_threshold)) {
        const std::size_t i = idx(x, y);
        dist[i] = 0.0f;
        pq.emplace(0.0f, i);
      }
    }
  }

  const float diag = res * static_cast<float>(std::sqrt(2.0));
  while (!pq.empty()) {
    const auto [d, i] = pq.top();
    pq.pop();
    if (d > dist[i]) {
      continue;  // stale entry
    }
    const unsigned int cx = static_cast<unsigned int>(i % size_x);
    const unsigned int cy = static_cast<unsigned int>(i / size_x);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = static_cast<int>(cx) + dx;
        const int ny = static_cast<int>(cy) + dy;
        if (nx < 0 || ny < 0 ||
          nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y))
        {
          continue;
        }
        const float step = (dx != 0 && dy != 0) ? diag : res;
        const float nd = d + step;
        if (nd >= max_dist) {
          continue;
        }
        const std::size_t ni =
          idx(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
        if (nd < dist[ni]) {
          dist[ni] = nd;
          pq.emplace(nd, ni);
        }
      }
    }
  }

  return dist;
}

xt::xtensor<float, 1> computeApfRepulsionCosts(
  const xt::xtensor<float, 2> & traj_x,
  const xt::xtensor<float, 2> & traj_y,
  const nav2_costmap_2d::Costmap2D & costmap,
  const float influence_dist,
  const float eta,
  const int power,
  const unsigned char cost_threshold)
{
  const std::size_t batch_size = traj_x.shape(0);
  const std::size_t time_steps = traj_x.shape(1);
  xt::xtensor<float, 1> out = xt::zeros<float>({batch_size});

  const std::vector<float> field =
    obstacleDistanceField(costmap, cost_threshold, influence_dist);
  const float d_min = 0.5f * static_cast<float>(costmap.getResolution());
  const unsigned int size_x = costmap.getSizeInCellsX();
  const float inv_d0 = influence_dist > 0.0f ? 1.0f / influence_dist : 0.0f;

  for (std::size_t i = 0; i < batch_size; ++i) {
    float accum = 0.0f;
    std::size_t counted = 0;
    for (std::size_t t = 0; t < time_steps; ++t) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap.worldToMap(traj_x(i, t), traj_y(i, t), mx, my)) {
        continue;  // off-map: do not penalise horizon length
      }
      ++counted;
      float d = field[static_cast<std::size_t>(my) * size_x + mx];
      if (d >= influence_dist) {
        continue;  // outside the field of influence -> no repulsion
      }
      d = std::max(d, d_min);
      const float term = (1.0f / d) - inv_d0;
      accum += 0.5f * eta * term * term;
    }
    if (counted == 0) {
      continue;
    }
    const float mean_u = accum / static_cast<float>(counted);
    out(i) = std::pow(mean_u, static_cast<float>(std::max(power, 1)));
  }

  return out;
}

}  // namespace nav2_se_controller
