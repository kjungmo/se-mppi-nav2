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

#include "nav2_se_controller/obstacle_clustering.hpp"

#include <cmath>
#include <vector>

namespace nav2_se_controller
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr unsigned char kNoInformation = 255;  // nav2_costmap_2d::NO_INFORMATION

/// Occupied == at/above threshold but NOT unknown space.
inline bool isObstacle(unsigned char cost, unsigned char threshold)
{
  return cost >= threshold && cost != kNoInformation;
}
}  // namespace

std::vector<ObstacleCluster> clusterObstacles(
  const nav2_costmap_2d::Costmap2D & costmap,
  unsigned char cost_threshold, int min_cells)
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const double resolution = costmap.getResolution();

  std::vector<ObstacleCluster> clusters;
  if (size_x == 0 || size_y == 0) {
    return clusters;
  }

  std::vector<char> visited(static_cast<std::size_t>(size_x) * size_y, 0);
  const auto idx = [size_x](unsigned int mx, unsigned int my) {
    return static_cast<std::size_t>(my) * size_x + mx;
  };

  std::vector<std::pair<unsigned int, unsigned int>> stack;
  for (unsigned int sy = 0; sy < size_y; ++sy) {
    for (unsigned int sx = 0; sx < size_x; ++sx) {
      if (visited[idx(sx, sy)]) {
        continue;
      }
      if (!isObstacle(costmap.getCost(sx, sy), cost_threshold)) {
        visited[idx(sx, sy)] = 1;
        continue;
      }

      // Flood fill this connected component (8-connectivity).
      stack.clear();
      stack.emplace_back(sx, sy);
      visited[idx(sx, sy)] = 1;
      double sum_wx = 0.0;
      double sum_wy = 0.0;
      int count = 0;
      std::vector<Eigen::Vector2d> cells;

      while (!stack.empty()) {
        const auto [cx, cy] = stack.back();
        stack.pop_back();

        double wx = 0.0;
        double wy = 0.0;
        costmap.mapToWorld(cx, cy, wx, wy);
        sum_wx += wx;
        sum_wy += wy;
        ++count;
        cells.emplace_back(wx, wy);

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
            const auto nidx = idx(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
            if (visited[nidx]) {
              continue;
            }
            visited[nidx] = 1;
            if (isObstacle(
                costmap.getCost(
                  static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)), cost_threshold))
            {
              stack.emplace_back(
                static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
            }
          }
        }
      }

      if (count < min_cells) {
        continue;
      }
      ObstacleCluster cluster;
      cluster.centroid = Eigen::Vector2d(sum_wx / count, sum_wy / count);
      const double area = count * resolution * resolution;
      cluster.radius = std::sqrt(area / kPi);
      cluster.cell_count = count;
      cluster.cells = std::move(cells);
      clusters.push_back(std::move(cluster));
    }
  }

  return clusters;
}

}  // namespace nav2_se_controller
