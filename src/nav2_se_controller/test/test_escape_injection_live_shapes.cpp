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

// Regression harness for the LIVE escape-cost injection crash (2026-07-02):
// the first cycle in which EscapeCritic injected APF + gap costs SIGSEGV'd the
// whole nav2 container on the Gazebo benchmark (launch log: "escape-cost
// injection BEGIN" -> "signal 11"). The 2D unit tests only ever exercised
// batch<=2 / 10x10-costmap shapes, so this file replays the injection path
// with the live conditions: a 60x60 rolling-window local costmap whose origin
// is far from (0,0) (odom frame), MPPI-scale batches (1500 x 28), trajectories
// that leave the map, and NaN/Inf rows (a degenerate optimizer state).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/gap_search.hpp"
#include "nav2_se_controller/repulsion.hpp"

using nav2_se_controller::computeApfRepulsionCosts;
using nav2_se_controller::computeGapAttractionCosts;
using nav2_se_controller::computeRepulsionCosts;
using nav2_se_controller::findEscapeGap;

namespace
{

// Live-like local costmap: 3 m x 3 m rolling window (60x60 @ 0.05) centered
// near the U-trap crash pose, odom-frame origin far from zero. Walls on two
// sides plus an interior blob, mirroring the pocket.
nav2_costmap_2d::Costmap2D makeLiveCostmap()
{
  nav2_costmap_2d::Costmap2D cm(60, 60, 0.05, 2.30, -5.70, 0);
  for (unsigned int i = 0; i < 60; ++i) {
    cm.setCost(i, 5, 254);     // south wall band
    cm.setCost(40, i, 254);    // east wall band
  }
  for (unsigned int y = 25; y < 32; ++y) {
    for (unsigned int x = 20; x < 27; ++x) {
      cm.setCost(x, y, 253);   // inscribed blob
    }
  }
  return cm;
}

// MPPI-scale trajectory fan from the robot pose; a slice of the batch runs off
// the map and two rows are degenerate (NaN / Inf).
void makeLiveTrajectories(
  double rx, double ry, std::size_t batch, std::size_t steps,
  xt::xtensor<float, 2> & tx, xt::xtensor<float, 2> & ty)
{
  tx = xt::zeros<float>({batch, steps});
  ty = xt::zeros<float>({batch, steps});
  for (std::size_t i = 0; i < batch; ++i) {
    const double bearing = -M_PI + (2.0 * M_PI * static_cast<double>(i)) /
      static_cast<double>(batch);
    for (std::size_t t = 0; t < steps; ++t) {
      const double r = 0.05 * static_cast<double>(t + 1) *
        (i % 7 == 0 ? 4.0 : 1.0);   // every 7th ray overshoots the window
      tx(i, t) = static_cast<float>(rx + r * std::cos(bearing));
      ty(i, t) = static_cast<float>(ry + r * std::sin(bearing));
    }
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (std::size_t t = 0; t < steps; ++t) {
    tx(1, t) = nan;
    ty(1, t) = nan;
    tx(2, t) = inf;
    ty(2, t) = -inf;
  }
}

}  // namespace

TEST(EscapeInjectionLiveShapes, ApfRepulsionSurvivesLiveBatch)
{
  const auto cm = makeLiveCostmap();
  xt::xtensor<float, 2> tx, ty;
  makeLiveTrajectories(3.79, -4.25, 1500, 28, tx, ty);

  const auto costs = computeApfRepulsionCosts(tx, ty, cm, 0.6f, 1.0f, 1);
  ASSERT_EQ(costs.shape(0), 1500u);
  for (std::size_t i = 0; i < costs.shape(0); ++i) {
    ASSERT_TRUE(std::isfinite(costs(i))) << "cost " << i << " not finite";
    ASSERT_GE(costs(i), 0.0f);
  }
}

TEST(EscapeInjectionLiveShapes, CostProxyRepulsionSurvivesLiveBatch)
{
  const auto cm = makeLiveCostmap();
  xt::xtensor<float, 2> tx, ty;
  makeLiveTrajectories(3.79, -4.25, 1500, 28, tx, ty);

  const auto costs = computeRepulsionCosts(tx, ty, cm, 2.0f, 1);
  ASSERT_EQ(costs.shape(0), 1500u);
  for (std::size_t i = 0; i < costs.shape(0); ++i) {
    ASSERT_TRUE(std::isfinite(costs(i)));
  }
}

TEST(EscapeInjectionLiveShapes, GapSearchSurvivesRobotAgainstWall)
{
  const auto cm = makeLiveCostmap();
  // Robot pinned against the south wall band (the crash pose), goal east.
  const auto gap = findEscapeGap(cm, 3.79, -4.25, 0.0, 36, 2.0, 0.6);
  (void)gap;  // any result is fine; the property under test is no-crash

  // Also from outside the window entirely: worldToMap fails at the very first
  // sample of every ray, so clearance == first step (~res/2) < min_clearance
  // and no gap is reported. The property under test is graceful no-crash.
  const auto gap2 = findEscapeGap(cm, -50.0, 77.0, 0.0, 36, 2.0, 0.6);
  EXPECT_FALSE(gap2.found);
}

TEST(EscapeInjectionLiveShapes, GapAttractionSurvivesNanRows)
{
  xt::xtensor<float, 2> tx, ty;
  makeLiveTrajectories(3.79, -4.25, 1500, 28, tx, ty);
  const auto costs = computeGapAttractionCosts(tx, ty, 3.79, -4.25, 2.5, 4.0f, 1);
  ASSERT_EQ(costs.shape(0), 1500u);
  // NaN rows may produce NaN attraction terms, but must not crash.
}

TEST(EscapeInjectionLiveShapes, CombinedInjectionAccumulatesLikeTheCritic)
{
  // Mirror EscapeCritic::score's arithmetic: costs += APF; costs += gap.
  const auto cm = makeLiveCostmap();
  xt::xtensor<float, 2> tx, ty;
  makeLiveTrajectories(3.79, -4.25, 1500, 28, tx, ty);

  xt::xtensor<float, 1> costs = xt::zeros<float>({std::size_t{1500}});
  costs += computeApfRepulsionCosts(tx, ty, cm, 0.6f, 1.0f, 1);
  const auto gap = findEscapeGap(cm, 3.79, -4.25, 0.0, 36, 2.0, 0.6);
  if (gap.found) {
    costs += computeGapAttractionCosts(tx, ty, 3.79, -4.25, gap.bearing, 4.0f, 1);
  }
  ASSERT_EQ(costs.shape(0), 1500u);
}
