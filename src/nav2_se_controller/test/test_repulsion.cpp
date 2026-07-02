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

#include <gtest/gtest.h>

#include <cmath>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/repulsion.hpp"

using nav2_se_controller::computeApfRepulsionCosts;
using nav2_se_controller::computeRepulsionCosts;
using nav2_se_controller::obstacleDistanceField;

TEST(Repulsion, ObstacleTrajectoryCostsMoreThanFree)
{
  // 10x10 cells, 0.1 m resolution => 1 m square, origin (0,0), default cost 0.
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  // Lethal column at map x = 5 (world x ~ 0.5).
  for (unsigned int my = 0; my < 10; ++my) {
    costmap.setCost(5, my, 254);
  }

  // batch=2, time=4. Traj 0 passes through the obstacle column; traj 1 stays
  // in free space on the left.
  xt::xtensor<float, 2> tx = {
    {0.45f, 0.50f, 0.55f, 0.50f},
    {0.10f, 0.15f, 0.20f, 0.15f}};
  xt::xtensor<float, 2> ty = {
    {0.50f, 0.50f, 0.50f, 0.50f},
    {0.50f, 0.50f, 0.50f, 0.50f}};

  auto costs = computeRepulsionCosts(tx, ty, costmap, 1.0f, 1);
  ASSERT_EQ(costs.shape(0), 2u);
  EXPECT_GT(costs(0), costs(1));
  EXPECT_FLOAT_EQ(costs(1), 0.0f);  // free space => zero repulsion
}

TEST(Repulsion, OffMapPointsAreSkipped)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  // All points off-map (beyond the 1 m extent): no penalty, no crash.
  xt::xtensor<float, 2> tx = {{5.0f, 6.0f}};
  xt::xtensor<float, 2> ty = {{5.0f, 6.0f}};

  auto costs = computeRepulsionCosts(tx, ty, costmap, 1.0f, 1);
  EXPECT_FLOAT_EQ(costs(0), 0.0f);
}

TEST(Repulsion, WeightScalesLinearly)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  for (unsigned int my = 0; my < 10; ++my) {
    costmap.setCost(5, my, 254);
  }
  xt::xtensor<float, 2> tx = {{0.50f, 0.50f}};
  xt::xtensor<float, 2> ty = {{0.50f, 0.50f}};

  auto c1 = computeRepulsionCosts(tx, ty, costmap, 1.0f, 1);
  auto c2 = computeRepulsionCosts(tx, ty, costmap, 2.0f, 1);
  EXPECT_FLOAT_EQ(c2(0), 2.0f * c1(0));
}

TEST(ObstacleDistanceField, DistancesFromSingleObstacle)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  costmap.setCost(5, 5, 254);  // single lethal cell

  auto field = obstacleDistanceField(costmap, 253, 1.0f);
  const auto at = [&](unsigned int x, unsigned int y) {return field[y * 10 + x];};

  EXPECT_FLOAT_EQ(at(5, 5), 0.0f);            // on the obstacle
  EXPECT_NEAR(at(6, 5), 0.1f, 1e-4);          // 1 cell away (straight)
  EXPECT_NEAR(at(5, 7), 0.2f, 1e-4);          // 2 cells away
  EXPECT_NEAR(at(6, 6), 0.1f * std::sqrt(2.0f), 1e-4);  // diagonal
}

TEST(ObstacleDistanceField, UnknownCellsAreNotObstacles)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  costmap.setCost(5, 5, 255);  // NO_INFORMATION must not seed the field
  auto field = obstacleDistanceField(costmap, 253, 1.0f);
  EXPECT_FLOAT_EQ(field[5 * 10 + 5], 1.0f);  // no obstacle => everything at max_dist
  EXPECT_FLOAT_EQ(field[0], 1.0f);
}

TEST(ObstacleDistanceField, CapsAtMaxDist)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  costmap.setCost(0, 0, 254);
  auto field = obstacleDistanceField(costmap, 253, 0.3f);
  EXPECT_FLOAT_EQ(field[9 * 10 + 9], 0.3f);  // far corner clamped to max_dist
}

TEST(ApfRepulsion, MonotonicDecreaseWithDistance)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, 0.0, 0.0, 0);
  for (unsigned int my = 0; my < 20; ++my) {
    costmap.setCost(10, my, 254);  // wall at map x=10 (world ~1.0)
  }
  // Single-point trajectories at increasing distance from the wall.
  auto cost_at = [&](float x) {
    xt::xtensor<float, 2> tx = {{x}};
    xt::xtensor<float, 2> ty = {{1.0f}};
    return computeApfRepulsionCosts(tx, ty, costmap, 0.6f, 1.0f, 1)(0);
  };
  const float near = cost_at(0.75f);   // ~0.25 m from wall
  const float mid = cost_at(0.55f);    // ~0.45 m
  const float far = cost_at(0.30f);    // ~0.70 m -> beyond influence (0.6)
  EXPECT_GT(near, mid);
  EXPECT_GT(mid, 0.0f);
  EXPECT_FLOAT_EQ(far, 0.0f);
}

TEST(ApfRepulsion, ZeroWhenNoObstacles)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, 0.0, 0.0, 0);
  xt::xtensor<float, 2> tx = {{1.0f, 1.1f}};
  xt::xtensor<float, 2> ty = {{1.0f, 1.0f}};
  auto c = computeApfRepulsionCosts(tx, ty, costmap, 0.6f, 1.0f, 1);
  EXPECT_FLOAT_EQ(c(0), 0.0f);
}
