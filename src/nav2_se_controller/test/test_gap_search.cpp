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
#include "nav2_se_controller/gap_search.hpp"

using nav2_se_controller::computeGapAttractionCosts;
using nav2_se_controller::EscapeGap;
using nav2_se_controller::findEscapeGap;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(GapSearch, OpenSpacePicksGoalBearing)
{
  // No obstacles: the gap closest to the goal bearing is the goal bearing itself.
  nav2_costmap_2d::Costmap2D costmap(40, 40, 0.1, -2.0, -2.0, 0);
  auto gap = findEscapeGap(costmap, 0.0, 0.0, 0.5, 36, 1.5, 0.6);
  ASSERT_TRUE(gap.found);
  EXPECT_NEAR(gap.bearing, 0.5, 2.0 * kPi / 36 + 1e-6);  // within one ray step
}

TEST(GapSearch, BlockedGoalDirectionDeviatesToOpening)
{
  // Wall directly ahead (goal bearing 0) blocks +x; +/-y is open. The chosen gap
  // bearing must deviate substantially from 0 to clear the wall.
  nav2_costmap_2d::Costmap2D costmap(60, 60, 0.1, -3.0, -3.0, 0);
  // Close, wide vertical wall at world x ~ 0.3, spanning y in [-1.5, 1.5], so
  // rays near the goal axis (bearing 0) hit it well within min_clearance.
  for (double x = 0.3; x <= 0.4; x += 0.05) {
    for (double y = -1.5; y <= 1.5; y += 0.05) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (costmap.worldToMap(x, y, mx, my)) {
        costmap.setCost(mx, my, 254);
      }
    }
  }
  auto gap = findEscapeGap(costmap, 0.0, 0.0, 0.0, 72, 2.0, 0.6);
  ASSERT_TRUE(gap.found);
  EXPECT_GT(std::abs(gap.bearing), 0.4);  // had to steer well off the blocked axis
  EXPECT_GE(gap.clearance, 0.6);
}

TEST(GapSearch, NoGapWhenFullyEnclosed)
{
  // Robot ringed by obstacles within min_clearance: no viable gap.
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0, 254);  // all lethal
  auto gap = findEscapeGap(costmap, 0.0, 0.0, 0.0, 36, 1.0, 0.6);
  EXPECT_FALSE(gap.found);
}

TEST(GapAttraction, AlignedTrajectoryCostsLess)
{
  // Two trajectories from the origin: one ending toward the gap bearing (pi/2),
  // one ending away (-pi/2). Aligned one must cost less.
  const double gap_bearing = kPi / 2.0;
  xt::xtensor<float, 2> tx = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  xt::xtensor<float, 2> ty = {{0.0f, 1.0f}, {0.0f, -1.0f}};  // +y vs -y endpoints
  auto costs = computeGapAttractionCosts(tx, ty, 0.0, 0.0, gap_bearing, 1.0f, 1);
  ASSERT_EQ(costs.shape(0), 2u);
  EXPECT_LT(costs(0), costs(1));
  EXPECT_NEAR(costs(0), 0.0f, 1e-5);  // perfectly aligned -> ~0
}
