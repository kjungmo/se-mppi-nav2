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

#ifndef NAV2_SE_CONTROLLER__REPULSION_HPP_
#define NAV2_SE_CONTROLLER__REPULSION_HPP_

#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#pragma GCC diagnostic pop

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_se_controller
{

/**
 * @brief Per-trajectory repulsive-potential cost from the costmap.
 *
 * For each candidate trajectory, accumulates an artificial-potential-field
 * (APF) repulsion proxied by the local costmap value at each rolled-out point
 * (higher costmap cost == closer to an obstacle). Trajectories that linger near
 * obstacles score higher, so when this term is added to the MPPI cost only
 * while entrapped, the optimizer is biased toward detouring into open space.
 *
 * Off-map points are skipped (not penalised) so the planning horizon length
 * does not bias the result. Returns an array of length batch_size.
 *
 * This is the M1 cost-proxy form; a true distance-field APF plus free-space
 * gap search are tracked as M1.x in the design doc (§3.1).
 *
 * @param traj_x  [batch_size, time_steps] world-frame x of each rollout point.
 * @param traj_y  [batch_size, time_steps] world-frame y of each rollout point.
 * @param costmap Local costmap to query.
 * @param weight  Linear scale on the repulsion term.
 * @param power   Exponent applied to the normalised mean cost (>= 1).
 * @return xt::xtensor<float, 1> of length batch_size.
 */
xt::xtensor<float, 1> computeRepulsionCosts(
  const xt::xtensor<float, 2> & traj_x,
  const xt::xtensor<float, 2> & traj_y,
  const nav2_costmap_2d::Costmap2D & costmap,
  float weight,
  int power);

/**
 * @brief Euclidean-ish distance (m) from every costmap cell to the nearest
 *        occupied cell, capped at max_dist.
 *
 * Multi-source Dijkstra with octile step costs (straight = resolution, diagonal
 * = resolution * sqrt(2)) seeded from all cells with cost >= cost_threshold.
 * Cells farther than max_dist (or with no obstacle within it) get max_dist.
 * This is the distance field behind the true APF repulsion (M1.x), replacing
 * the M1 cost-proxy. Returned in row-major (y * size_x + x) order.
 */
std::vector<float> obstacleDistanceField(
  const nav2_costmap_2d::Costmap2D & costmap,
  unsigned char cost_threshold,
  float max_dist);

/**
 * @brief Per-trajectory artificial-potential-field repulsion from a true
 *        obstacle distance field (M1.x).
 *
 * For each rolled-out point at obstacle distance d, applies the classic APF
 * repulsive potential
 *     U_rep(d) = 0.5 * eta * (1/d - 1/d0)^2   for 0 < d < d0,   else 0,
 * with d0 = influence_dist. Distances are clamped to >= 0.5*resolution to keep
 * U finite on/at obstacle cells. Each trajectory's cost is the mean U over its
 * on-map points (length-invariant), raised to `power`. Off-map points are
 * skipped. Returns an array of length batch_size.
 *
 * Unlike the cost-proxy form, this follows the metric 1/d law and so escapes
 * non-convex (U-shaped) traps more reliably. cost_threshold selects which cells
 * count as obstacles (default 253 = INSCRIBED_INFLATED_OBSTACLE).
 */
xt::xtensor<float, 1> computeApfRepulsionCosts(
  const xt::xtensor<float, 2> & traj_x,
  const xt::xtensor<float, 2> & traj_y,
  const nav2_costmap_2d::Costmap2D & costmap,
  float influence_dist,
  float eta,
  int power,
  unsigned char cost_threshold = 253);

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__REPULSION_HPP_
