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
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"
#include "nav2_se_controller/static_occupancy_filter.hpp"

using nav2_se_controller::DynamicObstacleTracker;
using nav2_se_controller::StaticFilterConfig;
using nav2_se_controller::StaticOccupancyFilter;
using nav2_se_controller::TrackedObstacle;
using nav2_se_controller::TrackerConfig;

namespace
{
constexpr unsigned char kLethal = 254;
constexpr unsigned char kNoInfo = 255;

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

TEST(StaticOccupancyFilter, AccumulatesEvidenceToStatic)
{
  StaticOccupancyFilter filter;
  StaticFilterConfig cfg;
  cfg.static_min_frames = 3;
  filter.configure(cfg);

  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(map, 5, 6, 5, 6, kLethal);  // block around world (0.55, 0.55)

  filter.update(map);
  EXPECT_FALSE(filter.isStatic(0.55, 0.55));  // 1 frame < 3
  filter.update(map);
  EXPECT_FALSE(filter.isStatic(0.55, 0.55));  // 2 frames
  filter.update(map);
  EXPECT_TRUE(filter.isStatic(0.55, 0.55));   // 3 frames -> static
  EXPECT_FALSE(filter.isStatic(1.50, 1.50));  // free cell never static
}

TEST(StaticOccupancyFilter, FreedCellDecaysBackToDynamic)
{
  StaticOccupancyFilter filter;
  StaticFilterConfig cfg;
  cfg.static_min_frames = 3;
  filter.configure(cfg);

  nav2_costmap_2d::Costmap2D occupied(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(occupied, 5, 6, 5, 6, kLethal);
  for (int i = 0; i < 4; ++i) {
    filter.update(occupied);
  }
  ASSERT_TRUE(filter.isStatic(0.55, 0.55));

  // The block leaves (cells observed FREE): evidence decays below threshold.
  nav2_costmap_2d::Costmap2D empty(20, 20, 0.1, 0.0, 0.0, 0);
  filter.update(empty);
  filter.update(empty);
  EXPECT_FALSE(filter.isStatic(0.55, 0.55));
}

TEST(StaticOccupancyFilter, UnknownCellsAreNotObservations)
{
  StaticOccupancyFilter filter;
  StaticFilterConfig cfg;
  cfg.static_min_frames = 2;
  filter.configure(cfg);

  nav2_costmap_2d::Costmap2D occupied(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(occupied, 5, 6, 5, 6, kLethal);
  filter.update(occupied);
  filter.update(occupied);
  ASSERT_TRUE(filter.isStatic(0.55, 0.55));

  // NO_INFORMATION frames neither accumulate nor decay the evidence.
  nav2_costmap_2d::Costmap2D unknown(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(unknown, 0, 19, 0, 19, kNoInfo);
  filter.update(unknown);
  filter.update(unknown);
  filter.update(unknown);
  EXPECT_TRUE(filter.isStatic(0.55, 0.55));
}

TEST(StaticOccupancyFilter, WorldAnchoredAcrossRollingWindow)
{
  StaticOccupancyFilter filter;
  StaticFilterConfig cfg;
  cfg.static_min_frames = 3;
  filter.configure(cfg);

  // The same WORLD wall observed through costmap windows whose origin moves
  // (rolling local costmap following the robot). Evidence must accumulate on
  // the world cell regardless of the window shift.
  // Wall cells: world x in [1.0, 1.2), y in [1.0, 1.2).
  for (int shift = 0; shift < 3; ++shift) {
    const double origin = 0.1 * shift;  // origin moves one cell per frame
    nav2_costmap_2d::Costmap2D map(20, 20, 0.1, origin, origin, 0);
    // Wall world-rect maps to cell (1.0 - origin)/0.1 .. — fill exactly it
    // (lround: the division is inexact in floating point).
    const auto cx0 = static_cast<unsigned int>(std::lround((1.0 - origin) / 0.1));
    fillBlock(map, cx0, cx0 + 1, cx0, cx0 + 1, kLethal);
    filter.update(map);
  }
  EXPECT_TRUE(filter.isStatic(1.05, 1.05));
}

TEST(StaticOccupancyFilter, StaticFractionCountsPoints)
{
  StaticOccupancyFilter filter;
  StaticFilterConfig cfg;
  cfg.static_min_frames = 2;
  filter.configure(cfg);

  nav2_costmap_2d::Costmap2D map(20, 20, 0.1, 0.0, 0.0, 0);
  fillBlock(map, 5, 6, 5, 6, kLethal);
  filter.update(map);
  filter.update(map);

  const std::vector<Eigen::Vector2d> points = {
    {0.55, 0.55},   // static
    {0.65, 0.65},   // static
    {1.50, 1.50},   // free
    {1.60, 1.60},   // free
  };
  EXPECT_DOUBLE_EQ(filter.staticFraction(points), 0.5);
  EXPECT_DOUBLE_EQ(filter.staticFraction({}), 0.0);
}

// ---------------------------------------------------------------------------
// Tracker integration (SE-Predict N1): the wall-freeze regression.
// ---------------------------------------------------------------------------

TEST(DynamicObstacleTrackerN1, WallPhantomVelocitySuppressed)
{
  // THE live-run failure: a wall edge enters/leaves the rolling costmap, the
  // centroid jumps within the association gate, and the wall acquires a
  // phantom velocity above the dynamic-speed threshold -> the CBF brakes the
  // robot against a wall ("wall-freeze"). With occupancy persistence, the
  // wall is classified static and its velocity forced to zero.
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.static_min_frames = 3;
  cfg.association_gate = 1.0;  // generous gate so the jitter WOULD associate
  tracker.configure(cfg);

  // A wall column observed for 4 frames (builds static evidence).
  nav2_costmap_2d::Costmap2D wall(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(wall, 10, 11, 5, 24, kLethal);
  std::vector<TrackedObstacle> obs;
  for (int f = 0; f < 4; ++f) {
    obs = tracker.update(wall, 0.1 * f);
  }
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_FALSE(obs[0].is_dynamic);

  // Frame 5: one more column appears (sensor reveals more wall) -> centroid
  // jumps. The CV association computes a velocity, but the cluster still sits
  // mostly on persistent cells: it must stay static with zero velocity.
  nav2_costmap_2d::Costmap2D wall_grown(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(wall_grown, 10, 13, 5, 24, kLethal);
  obs = tracker.update(wall_grown, 0.4);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_FALSE(obs[0].is_dynamic);
  EXPECT_DOUBLE_EQ(obs[0].velocity.norm(), 0.0);
}

TEST(DynamicObstacleTrackerN1, MovingObstacleStaysDynamic)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.static_min_frames = 3;
  tracker.configure(cfg);

  // A blob moving one cell (0.1 m) per frame sweeps fresh cells; no cell is
  // ever occupied long enough to become static.
  std::vector<TrackedObstacle> obs;
  for (int f = 0; f < 6; ++f) {
    nav2_costmap_2d::Costmap2D map(40, 40, 0.1, 0.0, 0.0, 0);
    const auto x0 = static_cast<unsigned int>(5 + f);
    fillBlock(map, x0, x0 + 1, 10, 11, kLethal);
    obs = tracker.update(map, 0.5 * f);
  }
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_TRUE(obs[0].is_dynamic);
  EXPECT_NEAR(obs[0].velocity.x(), 0.2, 1e-6);  // 0.1 m / 0.5 s
}

TEST(DynamicObstacleTrackerN1, WallAndMoverClassifiedIndependently)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.static_min_frames = 3;
  tracker.configure(cfg);

  std::vector<TrackedObstacle> obs;
  for (int f = 0; f < 6; ++f) {
    nav2_costmap_2d::Costmap2D map(40, 40, 0.1, 0.0, 0.0, 0);
    fillBlock(map, 2, 3, 2, 30, kLethal);  // persistent wall
    const auto x0 = static_cast<unsigned int>(15 + f);
    fillBlock(map, x0, x0 + 1, 15, 16, kLethal);  // mover
    obs = tracker.update(map, 0.5 * f);
  }
  ASSERT_EQ(obs.size(), 2u);
  int n_static = 0;
  int n_dynamic = 0;
  for (const auto & o : obs) {
    if (o.is_dynamic) {
      ++n_dynamic;
      EXPECT_GT(o.velocity.norm(), 0.1);
    } else {
      ++n_static;
      EXPECT_DOUBLE_EQ(o.velocity.norm(), 0.0);
    }
  }
  EXPECT_EQ(n_static, 1);
  EXPECT_EQ(n_dynamic, 1);
}

TEST(DynamicObstacleTrackerN1, ClassificationDisabledKeepsLegacyBehavior)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.classify_static = false;
  tracker.configure(cfg);

  nav2_costmap_2d::Costmap2D wall(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(wall, 10, 11, 5, 24, kLethal);
  std::vector<TrackedObstacle> obs;
  for (int f = 0; f < 6; ++f) {
    obs = tracker.update(wall, 0.1 * f);
  }
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_TRUE(obs[0].is_dynamic);  // legacy: everything reported dynamic
  EXPECT_EQ(tracker.staticFilter().trackedCells(), 0u);
}

TEST(DynamicObstacleTrackerN1, FreshObstacleConservativelyDynamic)
{
  // First frames of ANY obstacle (before evidence accumulates) must stay
  // dynamic-eligible: the safe direction for the CBF.
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.static_min_frames = 5;
  tracker.configure(cfg);

  nav2_costmap_2d::Costmap2D map(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(map, 10, 11, 10, 11, kLethal);
  auto obs = tracker.update(map, 0.0);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_TRUE(obs[0].is_dynamic);
}
