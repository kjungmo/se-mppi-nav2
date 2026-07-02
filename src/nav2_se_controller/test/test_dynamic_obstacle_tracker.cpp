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

#include <algorithm>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"
#include "nav2_se_controller/obstacle_clustering.hpp"

using nav2_se_controller::clusterObstacles;
using nav2_se_controller::DynamicObstacleTracker;
using nav2_se_controller::TrackerConfig;

namespace
{
constexpr unsigned char kLethal = 254;

// Fill an inclusive cell rectangle with a cost value.
void fillBlock(
  nav2_costmap_2d::Costmap2D & map,
  unsigned int x0, unsigned int x1, unsigned int y0, unsigned int y1,
  unsigned char cost)
{
  for (unsigned int y = y0; y <= y1; ++y) {
    for (unsigned int x = x0; x <= x1; ++x) {
      map.setCost(x, y, cost);
    }
  }
}
}  // namespace

TEST(ObstacleClustering, EmptyCostmapHasNoClusters)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, 0.0, 0.0, 0);
  EXPECT_TRUE(clusterObstacles(map, 253, 1).empty());
}

TEST(ObstacleClustering, TwoSeparatedBlocks)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(map, 2, 3, 2, 3, kLethal);     // centroid ~ (0.30, 0.30)
  fillBlock(map, 15, 16, 15, 16, kLethal);  // centroid ~ (1.60, 1.60)

  auto clusters = clusterObstacles(map, 253, 1);
  ASSERT_EQ(clusters.size(), 2u);
  std::sort(
    clusters.begin(), clusters.end(),
    [](const auto & a, const auto & b) {return a.centroid.x() < b.centroid.x();});

  EXPECT_NEAR(clusters[0].centroid.x(), 0.30, 1e-6);
  EXPECT_NEAR(clusters[0].centroid.y(), 0.30, 1e-6);
  EXPECT_EQ(clusters[0].cell_count, 4);
  EXPECT_NEAR(clusters[1].centroid.x(), 1.60, 1e-6);
  EXPECT_NEAR(clusters[1].centroid.y(), 1.60, 1e-6);
}

TEST(ObstacleClustering, MinCellsDropsNoise)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, 0.0, 0.0, 0);
  map.setCost(5, 5, kLethal);             // single-cell noise
  fillBlock(map, 10, 12, 10, 12, kLethal);  // 3x3 = 9 cells

  auto clusters = clusterObstacles(map, 253, 3);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].cell_count, 9);
}

TEST(DynamicObstacleTracker, FirstFrameHasZeroVelocity)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(map, 5, 6, 5, 6, kLethal);

  DynamicObstacleTracker tracker;
  tracker.configure(TrackerConfig{});
  auto obs = tracker.update(map, 0.0);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_DOUBLE_EQ(obs[0].velocity.norm(), 0.0);
}

TEST(ObstacleClustering, UnknownCellsExcluded)
{
  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(map, 5, 7, 5, 7, 255);  // NO_INFORMATION block must not cluster
  EXPECT_TRUE(clusterObstacles(map, 253, 1).empty());
}

TEST(DynamicObstacleTracker, OneToOneAssociationNoPhantomVelocity)
{
  DynamicObstacleTracker tracker;
  tracker.configure(TrackerConfig{});

  // Frame 1: a single blob.
  nav2_costmap_2d::Costmap2D m1(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(m1, 10, 11, 10, 11, kLethal);
  tracker.update(m1, 0.0);

  // Frame 2 (dt=0.5): the blob split into two, both inside the gate of the one
  // previous cluster. Only one may claim it; the other must get zero velocity.
  nav2_costmap_2d::Costmap2D m2(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(m2, 12, 13, 10, 11, kLethal);
  fillBlock(m2, 15, 16, 10, 11, kLethal);
  auto obs = tracker.update(m2, 0.5);

  ASSERT_EQ(obs.size(), 2u);
  int moving = 0;
  for (const auto & o : obs) {
    if (o.velocity.norm() > 1e-6) {
      ++moving;
    }
  }
  EXPECT_EQ(moving, 1);  // exactly one association, no phantom second velocity
}

TEST(DynamicObstacleTracker, EstimatesConstantVelocity)
{
  DynamicObstacleTracker tracker;
  tracker.configure(TrackerConfig{});

  // Frame 1: blob centred at x ~ 0.60.
  nav2_costmap_2d::Costmap2D m1(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(m1, 5, 6, 5, 6, kLethal);
  tracker.update(m1, 0.0);

  // Frame 2 (dt = 0.5 s): blob shifted +2 cells in x => +0.20 m.
  nav2_costmap_2d::Costmap2D m2(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(m2, 7, 8, 5, 6, kLethal);
  auto obs = tracker.update(m2, 0.5);

  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].velocity.x(), 0.40, 1e-6);  // 0.20 m / 0.5 s
  EXPECT_NEAR(obs[0].velocity.y(), 0.0, 1e-6);
}
