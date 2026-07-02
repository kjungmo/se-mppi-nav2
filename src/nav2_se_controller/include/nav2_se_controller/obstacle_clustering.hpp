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

#ifndef NAV2_SE_CONTROLLER__OBSTACLE_CLUSTERING_HPP_
#define NAV2_SE_CONTROLLER__OBSTACLE_CLUSTERING_HPP_

#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_se_controller
{

/// A connected blob of occupied costmap cells.
struct ObstacleCluster
{
  Eigen::Vector2d centroid{0.0, 0.0};  ///< world-frame centroid (m).
  double radius{0.0};                  ///< equivalent-area radius (m).
  int cell_count{0};
  std::vector<Eigen::Vector2d> cells;  ///< world-frame center of every member cell.
};

/**
 * @brief Cluster occupied costmap cells into obstacle blobs (8-connectivity).
 *
 * Pure function over a Costmap2D: a cell is "occupied" when its cost is
 * >= cost_threshold. Each connected component becomes one cluster with a
 * world-frame centroid and an equivalent-area radius r = sqrt(area / pi),
 * where area = cell_count * resolution^2. Clusters smaller than min_cells are
 * dropped as noise.
 *
 * @param costmap Costmap to scan.
 * @param cost_threshold Minimum cell cost to be considered occupied
 *        (e.g. nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE).
 * @param min_cells Minimum cluster size to keep.
 */
std::vector<ObstacleCluster> clusterObstacles(
  const nav2_costmap_2d::Costmap2D & costmap,
  unsigned char cost_threshold,
  int min_cells = 1);

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__OBSTACLE_CLUSTERING_HPP_
