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

// Opening selection at the stall pose of the campaign-1 U-trap trials
// (recorded scenario s1_utrap; it is not a development geometry and no
// controller tuning uses it). The robot sits at map (0.29, 0.10), 0.34 m from
// the inflated back-wall band, in a 3 m x 3 m rolling window. The legacy rule
// (closest viable ray to the goal bearing) picks a sideways ray; the horizon
// rule picks the mouth.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/gap_search.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Box
{
  double x0, x1, y0, y1;
};

double boxDistance(const Box & b, double x, double y)
{
  const double dx = std::max({b.x0 - x, 0.0, x - b.x1});
  const double dy = std::max({b.y0 - y, 0.0, y - b.y1});
  return std::hypot(dx, dy);
}

// s1_utrap: back wall x in [0.85, 1.15], side walls y in +-[0.85, 1.15], all
// spanning the pocket (x in [-1, 1]); inner pocket x in [-1, 0.85], |y| <= 0.85.
nav2_costmap_2d::Costmap2D s1Window(double cx, double cy)
{
  const std::vector<Box> walls{
    {0.85, 1.15, -1.0, 1.0}, {-1.0, 1.0, 0.85, 1.15}, {-1.0, 1.0, -1.15, -0.85}};
  const double res = 0.05;
  const double ox = std::floor((cx - 1.5) / res + 0.5) * res;
  const double oy = std::floor((cy - 1.5) / res + 0.5) * res;
  nav2_costmap_2d::Costmap2D cm(60, 60, res, ox, oy, 0);
  for (int j = 0; j < 60; ++j) {
    for (int i = 0; i < 60; ++i) {
      const double x = ox + (i + 0.5) * res;
      const double y = oy + (j + 0.5) * res;
      double d = 1e9;
      for (const auto & w : walls) {
        d = std::min(d, boxDistance(w, x, y));
      }
      unsigned char c = 0;
      if (d <= 0.0) {
        c = 254;
      } else if (d <= 0.22) {
        c = 253;
      } else if (d < 0.45) {
        c = static_cast<unsigned char>(std::max(1.0, std::round(252.0 * (0.45 - d) / 0.23)));
      }
      if (c != 0) {
        cm.setCost(i, j, c);
      }
    }
  }
  return cm;
}

double deg(double rad) {return rad * 180.0 / kPi;}

}  // namespace

// Independent reference for the ray casting (same sampling rule as the
// implementation: half-cell steps, a cell at or above 253 stops the ray, the
// map edge ends it as free): used to print the clearances and to cross-check.
struct RefRay
{
  double bearing;
  double clearance;
  bool horizon;
};

std::vector<RefRay> refRays(
  const nav2_costmap_2d::Costmap2D & cm, double rx, double ry, int n, double max_range)
{
  std::vector<RefRay> out;
  const double step = 0.5 * cm.getResolution();
  for (int k = 0; k < n; ++k) {
    const double b = -kPi + 2.0 * kPi * k / n;
    RefRay ray{b, max_range, true};
    for (double r = step; r <= max_range; r += step) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!cm.worldToMap(rx + r * std::cos(b), ry + r * std::sin(b), mx, my)) {
        ray = {b, r, true};
        break;
      }
      if (cm.getCost(mx, my) >= 253) {
        ray = {b, r, false};
        break;
      }
    }
    out.push_back(ray);
  }
  return out;
}

void printRays(const std::vector<RefRay> & rays)
{
  for (const auto & r : rays) {
    std::printf(
      "GAP ray %4.0f deg  clearance %.2f m  %s\n", deg(r.bearing), r.clearance,
      r.horizon ? "horizon" : "blocked");
  }
}

// Legacy rule (kept for the C_gap ablation): the diagnosis of ROOT_CAUSE.md
// section 4. Prints all 36 clearances.
TEST(GapPocket, LegacySelectionPicksASidewaysRayAtTheStallPose)
{
  const auto cm = s1Window(0.29, 0.10);
  std::printf("GAP stall pose (0.29, 0.10), goal bearing 0 deg, 36 rays, max_range 2.0\n");
  printRays(refRays(cm, 0.29, 0.10, 36, 2.0));
  const auto gap = nav2_se_controller::findEscapeGap(cm, 0.29, 0.10, 0.0, 36, 2.0, 0.6);
  ASSERT_TRUE(gap.found);
  std::printf(
    "GAP legacy selection: bearing %.0f deg, clearance %.2f m\n", deg(gap.bearing), gap.clearance);
  EXPECT_NEAR(std::abs(deg(gap.bearing)), 60.0, 10.0)
    << "the legacy rule chooses a sideways ray, not the mouth";
}

// ---------------------------------------------------------------------------
// Horizon rule
// ---------------------------------------------------------------------------
using nav2_se_controller::castGapRays;
using nav2_se_controller::findHorizonGap;
using nav2_se_controller::GapRay;
using nav2_se_controller::HorizonGap;
using nav2_se_controller::segmentBlocked;
using nav2_se_controller::selectHorizonGap;

TEST(GapPocket, CastGapRaysAgreesWithTheReferenceAtSeveralPoses)
{
  const std::vector<std::pair<double, double>> poses{
    {0.29, 0.10}, {-1.0, 0.0}, {-1.8, 0.4}, {0.5, 1.6}, {-0.2, -1.5}, {1.5, 0.0}};
  for (const auto & [x, y] : poses) {
    const auto cm = s1Window(x, y);
    const auto ref = refRays(cm, x, y, 36, 2.0);
    const auto got = castGapRays(cm, x, y, 36, 2.0);
    ASSERT_EQ(got.size(), ref.size());
    for (std::size_t i = 0; i < ref.size(); ++i) {
      EXPECT_NEAR(got[i].bearing, ref[i].bearing, 1e-12) << x << "," << y << " ray " << i;
      EXPECT_NEAR(got[i].clearance, ref[i].clearance, 1e-9) << x << "," << y << " ray " << i;
      EXPECT_EQ(got[i].horizon, ref[i].horizon) << x << "," << y << " ray " << i;
    }
  }
}

TEST(GapPocket, HorizonSelectionPicksTheMouthAtTheStallPose)
{
  const auto cm = s1Window(0.29, 0.10);
  const HorizonGap gap = findHorizonGap(cm, 0.29, 0.10, 0.0, 36, 2.0, 0.6, std::nullopt, 0.5);
  ASSERT_TRUE(gap.found);
  std::printf(
    "GAP horizon selection at the stall pose: bearing %.0f deg, clearance %.2f m, "
    "%zu horizon rays\n", deg(gap.bearing), gap.clearance, gap.candidates);
  EXPECT_TRUE(gap.horizon);
  // The mouth cone: horizon rays at 160, 170, 180, -170, -160 deg. The rule
  // takes the one closest to the goal bearing (an edge of the cone, 20 degrees
  // from the pocket axis at 180 deg).
  EXPECT_NEAR(std::abs(deg(gap.bearing)), 180.0, 20.0 + 1e-6);
  EXPECT_EQ(gap.candidates, 5u);
  EXPECT_GE(gap.clearance, 1.4);
}

// Sensing from other places on the way out (goal bearing 0 = east): the
// selection must never choose a blocked ray while a horizon ray exists.
TEST(GapPocket, HorizonSelectionOnTheWayOut)
{
  struct Case
  {
    double x, y;
  };
  for (const Case c : {Case{-1.0, 0.0}, Case{-1.6, 0.0}, Case{-1.4, 1.5}, Case{0.2, 1.7},
      Case{0.9, 1.7}})
  {
    const auto cm = s1Window(c.x, c.y);
    const auto rays = castGapRays(cm, c.x, c.y, 36, 2.0);
    const HorizonGap gap = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5);
    ASSERT_TRUE(gap.found) << c.x << "," << c.y;
    std::printf(
      "GAP at (%.1f, %.1f): bearing %.0f deg, clearance %.2f m, horizon %d, %zu horizon rays\n",
      c.x, c.y, deg(gap.bearing), gap.clearance, gap.horizon ? 1 : 0, gap.candidates);
    if (gap.candidates > 0) {
      EXPECT_TRUE(gap.horizon);
    }
  }
}

namespace
{
std::vector<GapRay> ring(const std::vector<std::pair<double, bool>> & clearance_horizon)
{
  std::vector<GapRay> out;
  const int n = static_cast<int>(clearance_horizon.size());
  for (int k = 0; k < n; ++k) {
    GapRay r;
    r.bearing = -kPi + 2.0 * kPi * k / n;
    r.clearance = clearance_horizon[k].first;
    r.horizon = clearance_horizon[k].second;
    out.push_back(r);
  }
  return out;
}
}  // namespace

TEST(GapSelection, ClosestHorizonRayToTheGoalBearingWins)
{
  // 8 rays at -180, -135, -90, -45, 0, 45, 90, 135. Horizon: -135 and 90.
  auto rays = ring(
    {{1.0, false}, {2.0, true}, {1.0, false}, {1.0, false}, {1.0, false}, {1.0, false},
      {2.0, true}, {1.0, false}});
  // Goal bearing 0: 90 (align 90) beats -135 (align 135).
  auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5);
  ASSERT_TRUE(g.found);
  EXPECT_NEAR(g.bearing, kPi / 2.0, 1e-9);
  EXPECT_TRUE(g.horizon);
  EXPECT_EQ(g.candidates, 2u);
  // Goal bearing -180: -135 (align 45) beats 90 (align 90).
  g = selectHorizonGap(rays, -kPi, 0.6, std::nullopt, 0.5);
  EXPECT_NEAR(g.bearing, -0.75 * kPi, 1e-9);
}

TEST(GapSelection, ALongerBlockedRayDoesNotBeatAHorizonRay)
{
  // The longest ray (1.9 m) is blocked; a horizon ray of 1.5 m exists.
  auto rays = ring({{1.9, false}, {1.5, true}, {0.5, false}, {0.5, false}});
  const auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5);
  ASSERT_TRUE(g.found);
  EXPECT_NEAR(g.clearance, 1.5, 1e-9);
  EXPECT_TRUE(g.horizon);
}

TEST(GapSelection, WithoutAHorizonRayTheLongestRayIsTaken)
{
  auto rays = ring({{0.7, false}, {1.2, false}, {0.9, false}, {0.2, false}});
  const auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5);
  ASSERT_TRUE(g.found);
  EXPECT_NEAR(g.clearance, 1.2, 1e-9);
  EXPECT_FALSE(g.horizon);
  EXPECT_EQ(g.candidates, 0u);
}

TEST(GapSelection, NothingUsableIsNotFound)
{
  EXPECT_FALSE(selectHorizonGap({}, 0.0, 0.6, std::nullopt, 0.5).found);
  auto rays = ring({{0.3, false}, {0.4, false}, {0.5, false}, {0.2, false}});
  EXPECT_FALSE(selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5).found);
  // A horizon ray shorter than the minimum clearance (robot at the map edge) is no candidate.
  auto edge = ring({{0.3, true}, {0.4, false}, {0.5, false}, {0.2, false}});
  EXPECT_FALSE(selectHorizonGap(edge, 0.0, 0.6, std::nullopt, 0.5).found);
}

TEST(GapSelection, ClearanceAtTheMinimumCounts)
{
  auto rays = ring({{0.6, true}, {0.1, false}, {0.1, false}, {0.1, false}});
  EXPECT_TRUE(selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5).found);
  rays = ring({{0.59, true}, {0.1, false}, {0.1, false}, {0.1, false}});
  EXPECT_FALSE(selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5).found);
}

TEST(GapSelection, HysteresisKeepsThePreviousBearingUnlessClearlyBetter)
{
  // 8 rays; horizon at -135 (align 135 to goal 0), 90 (align 90), 135 (align 135).
  auto rays = ring(
    {{1.0, false}, {2.0, true}, {1.0, false}, {1.0, false}, {1.0, false}, {1.0, false},
      {2.0, true}, {2.0, true}});
  // Previous = 135 deg (align 135); the best is 90 (align 90): better by 0.785 rad > 0.5 -> switch.
  auto g = selectHorizonGap(rays, 0.0, 0.6, 0.75 * kPi, 0.5);
  EXPECT_NEAR(g.bearing, kPi / 2.0, 1e-9);
  EXPECT_FALSE(g.kept_previous);
  // With hysteresis 1.0 rad the previous bearing (135) is kept.
  g = selectHorizonGap(rays, 0.0, 0.6, 0.75 * kPi, 1.0);
  EXPECT_NEAR(g.bearing, 0.75 * kPi, 1e-9);
  EXPECT_TRUE(g.kept_previous);
  // Previous = 90 (the best): kept.
  g = selectHorizonGap(rays, 0.0, 0.6, kPi / 2.0, 0.5);
  EXPECT_NEAR(g.bearing, kPi / 2.0, 1e-9);
  // Previous ray no longer reaches the horizon: switch to the best regardless.
  rays[7].horizon = false;
  g = selectHorizonGap(rays, 0.0, 0.6, 0.75 * kPi, 10.0);
  EXPECT_NEAR(g.bearing, kPi / 2.0, 1e-9);
  EXPECT_FALSE(g.kept_previous);
}

TEST(GapSelection, HysteresisBoundaryIsStrict)
{
  // ray 0 = -180 deg (align pi), ray 2 = -90 deg (align pi/2); the difference is pi/2.
  auto rays = ring(
    {{2.0, true}, {1.0, false}, {2.0, true}, {1.0, false}, {1.0, false}, {1.0, false},
      {1.0, false}, {1.0, false}});
  auto g = selectHorizonGap(rays, 0.0, 0.6, -kPi, kPi / 2.0 - 1e-6);
  EXPECT_NEAR(g.bearing, -kPi / 2.0, 1e-9) << "better by more than the hysteresis: switch";
  g = selectHorizonGap(rays, 0.0, 0.6, -kPi, kPi / 2.0 + 1e-6);
  EXPECT_NEAR(g.bearing, -kPi, 1e-9) << "better by less than the hysteresis: keep";
}

TEST(GapPocket, SegmentBlockedSeesTheBandAndIgnoresTheStart)
{
  const auto cm = s1Window(0.29, 0.10);
  EXPECT_TRUE(segmentBlocked(cm, 0.29, 0.10, 0.9, 0.10)) << "toward the back wall";
  EXPECT_FALSE(segmentBlocked(cm, 0.29, 0.10, -0.8, 0.10)) << "toward the mouth";
  EXPECT_FALSE(segmentBlocked(cm, 0.29, 0.10, 0.29, 0.10)) << "zero length";
  EXPECT_FALSE(segmentBlocked(cm, 5.0, 5.0, 6.0, 6.0)) << "off the map";
}

// ---------------------------------------------------------------------------
// Avoiding the place where entrapment was declared
// ---------------------------------------------------------------------------
using nav2_se_controller::AvoidPoint;

// At the mouth of the pocket the ray back into the pocket reaches the window
// edge (the back wall is outside the 3 m window), so it is a horizon ray and
// the closest to the goal bearing: without the exclusion the escape would send
// the robot straight back to the trap.
TEST(GapPocket, AtTheMouthTheRayBackToTheStallPoseIsExcluded)
{
  const double sx = 0.29;
  const double sy = 0.10;  // where entrapment was declared
  for (const double rx : {-1.0, -1.6}) {
    const double ry = 0.0;
    const auto cm = s1Window(rx, ry);
    const auto rays = castGapRays(cm, rx, ry, 36, 2.0);
    const HorizonGap without = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5);
    ASSERT_TRUE(without.found);
    EXPECT_NEAR(without.bearing, 0.0, 1e-9) << "without exclusion: back into the pocket";
    AvoidPoint avoid;
    avoid.bearing = std::atan2(sy - ry, sx - rx);
    avoid.distance = std::hypot(sx - rx, sy - ry);
    avoid.radius = 0.6;
    const HorizonGap with = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid});
    ASSERT_TRUE(with.found);
    std::printf(
      "GAP at the mouth (%.1f, 0.0) avoiding the stall pose: bearing %.0f deg, clearance "
      "%.2f m (without: %.0f deg)\n", rx, deg(with.bearing), with.clearance, deg(without.bearing));
    const double offset =
      avoid.distance * std::sin(
      std::abs(
        std::atan2(
          std::sin(with.bearing - avoid.bearing), std::cos(with.bearing - avoid.bearing))));
    EXPECT_GE(offset, avoid.radius) << "the selected ray passes the stall pose at its radius";
    EXPECT_TRUE(with.horizon);
  }
}

namespace
{
std::vector<GapRay> allHorizon8()
{
  return ring(
    {{2.0, true}, {2.0, true}, {2.0, true}, {2.0, true}, {2.0, true}, {2.0, true},
      {2.0, true}, {2.0, true}});
}
}  // namespace

TEST(GapSelection, AvoidedPlaceExcludesTheRaysThatPassNearIt)
{
  // 8 rays at -180, -135, -90, -45, 0, 45, 90, 135; goal bearing 0.
  const auto rays = allHorizon8();
  AvoidPoint avoid;
  avoid.bearing = 0.0;
  avoid.distance = 1.0;
  avoid.radius = 0.5;
  auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid});
  ASSERT_TRUE(g.found);
  EXPECT_NEAR(std::abs(g.bearing), kPi / 4.0, 1e-9) << "0 deg excluded; 45 deg passes at 0.707 m";
  EXPECT_EQ(g.candidates, 7u);
  // A larger radius also excludes 45 deg (1.0 * sin 45 = 0.707 < 0.8).
  avoid.radius = 0.8;
  g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid});
  EXPECT_NEAR(std::abs(g.bearing), kPi / 2.0, 1e-9);
  EXPECT_EQ(g.candidates, 5u);
}

TEST(GapSelection, AvoidedPlaceBehindTheRobotOrBeyondTheRayDoesNotExclude)
{
  const auto rays = allHorizon8();
  AvoidPoint avoid;
  avoid.bearing = kPi;  // behind: the ray at 180 deg would lead to it, but the goal is at 0
  avoid.distance = 1.0;
  avoid.radius = 0.5;
  auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid});
  EXPECT_NEAR(g.bearing, 0.0, 1e-9) << "rays toward the goal are untouched";
  // The place at 1.0 m with rays of 0.3 m: beyond the ray plus the radius.
  auto short_rays = ring(
    {{0.3, true}, {0.3, true}, {0.3, true}, {0.3, true}, {0.3, true}, {0.3, true},
      {0.3, true}, {0.3, true}});
  avoid.bearing = 0.0;
  g = selectHorizonGap(short_rays, 0.0, 0.2, std::nullopt, 0.5, {avoid});
  EXPECT_NEAR(g.bearing, 0.0, 1e-9) << "the ray ends before the place";
  // Within the radius of the place the exclusion is not applied.
  avoid.distance = 0.4;
  avoid.radius = 0.6;
  g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid});
  EXPECT_NEAR(g.bearing, 0.0, 1e-9) << "the robot is at the stall pose";
}

TEST(GapSelection, AvoidedPlaceIsIgnoredWhenItLeavesNothing)
{
  // The only horizon ray points at the avoided place.
  auto rays = ring(
    {{0.1, false}, {0.1, false}, {0.1, false}, {0.1, false}, {2.0, true}, {0.1, false},
      {0.1, false}, {0.1, false}});
  AvoidPoint avoid;
  avoid.bearing = 0.0;
  avoid.distance = 1.0;
  avoid.radius = 0.5;
  const auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid});
  ASSERT_TRUE(g.found);
  EXPECT_NEAR(g.bearing, 0.0, 1e-9);
}

TEST(GapSelection, AvoidedPlaceAlsoGuardsThePreviousBearingAndTheLongestRayFallback)
{
  // Previous bearing 0 deg is a horizon ray that leads to the place: not kept.
  auto rays = allHorizon8();
  AvoidPoint avoid;
  avoid.bearing = 0.0;
  avoid.distance = 1.0;
  avoid.radius = 0.5;
  auto g = selectHorizonGap(rays, 0.0, 0.6, 0.0, 10.0, {avoid});
  EXPECT_FALSE(g.kept_previous);
  EXPECT_NEAR(std::abs(g.bearing), kPi / 4.0, 1e-9);
  // No horizon rays: the longest ray is taken, but not the one toward the place.
  auto blocked = ring(
    {{0.7, false}, {0.8, false}, {0.9, false}, {1.0, false}, {1.9, false}, {1.1, false},
      {0.9, false}, {0.8, false}});
  g = selectHorizonGap(blocked, 0.0, 0.6, std::nullopt, 0.5, {avoid});
  ASSERT_TRUE(g.found);
  EXPECT_NEAR(g.clearance, 1.1, 1e-9) << "the 1.9 m ray at 0 deg leads to the place";
}

TEST(GapSelection, SeveralAvoidedPlacesAreAllApplied)
{
  const auto rays = allHorizon8();
  AvoidPoint a;
  a.bearing = 0.0;
  a.distance = 1.0;
  a.radius = 0.5;
  AvoidPoint b;
  b.bearing = kPi / 4.0;
  b.distance = 1.0;
  b.radius = 0.5;
  const auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {a, b});
  ASSERT_TRUE(g.found);
  // 0 deg (a), 45 deg (b: passes at 0), and -45 deg (a: 0.707 > 0.5 ok) -> -45 deg is
  // the closest to the goal that is left; 90 deg passes b at 0.707 (not excluded).
  EXPECT_NEAR(g.bearing, -kPi / 4.0, 1e-9);
  EXPECT_EQ(g.candidates, 6u);
}

// ---------------------------------------------------------------------------
// Boundaries of the ray rules
// ---------------------------------------------------------------------------
TEST(GapSelection, AvoidedPlaceOnlyExcludesRaysAheadOfTheRobot)
{
  // A radius of 0.9 m at 1.0 m excludes the rays within 64 degrees of the place
  // (0, +-45 deg); the rays at 135 deg pass it at 0.71 m but head away from it
  // (90 degrees or more off is not ahead), so they stay.
  const auto rays = allHorizon8();
  AvoidPoint avoid;
  avoid.bearing = 0.0;
  avoid.distance = 1.0;
  avoid.radius = 0.9;
  const auto g = selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid});
  ASSERT_TRUE(g.found);
  EXPECT_EQ(g.candidates, 5u) << "-180, -135, -90, 90, 135 deg remain";
  EXPECT_NEAR(std::abs(g.bearing), kPi / 2.0, 1e-9) << "90 deg is not ahead";
}

TEST(GapSelection, AvoidedPlaceIsIgnoredWhileTheRobotIsWithinItsRadius)
{
  const auto rays = allHorizon8();
  AvoidPoint avoid;
  avoid.bearing = 0.0;
  avoid.radius = 0.5;
  avoid.distance = 0.5;  // exactly at the radius: the robot is at the place
  EXPECT_NEAR(selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid}).bearing, 0.0, 1e-9);
  avoid.distance = 0.5000001;  // just outside: the ray toward it is excluded
  EXPECT_GT(std::abs(selectHorizonGap(rays, 0.0, 0.6, std::nullopt, 0.5, {avoid}).bearing), 0.1);
}

TEST(GapSelection, AvoidedPlaceBeyondTheRayPlusRadiusDoesNotCount)
{
  // Rays of 0.5 m; the place at 1.0 m with radius 0.5: the place is exactly
  // ray + radius away: still counted (the ray ends within the radius of it).
  auto rays = ring(
    {{0.5, true}, {0.5, true}, {0.5, true}, {0.5, true}, {0.5, true}, {0.5, true},
      {0.5, true}, {0.5, true}});
  AvoidPoint avoid;
  avoid.bearing = 0.0;
  avoid.radius = 0.5;
  avoid.distance = 1.0;
  EXPECT_GT(std::abs(selectHorizonGap(rays, 0.0, 0.4, std::nullopt, 0.5, {avoid}).bearing), 0.1);
  avoid.distance = 1.0000001;  // just beyond: the ray does not reach it
  EXPECT_NEAR(selectHorizonGap(rays, 0.0, 0.4, std::nullopt, 0.5, {avoid}).bearing, 0.0, 1e-9);
}

TEST(GapPocket, ARayThatRunsToMaxRangeInsideTheMapIsAHorizonRay)
{
  nav2_costmap_2d::Costmap2D cm(120, 120, 0.05, -3.0, -3.0, 0);  // 6 m, all free
  const auto rays = castGapRays(cm, 0.0, 0.0, 8, 2.0);
  for (const auto & r : rays) {
    EXPECT_TRUE(r.horizon) << r.bearing;
    EXPECT_NEAR(r.clearance, 2.0, 1e-9);
  }
  // A wall at x = 1.0 (one cell column): the ray east is blocked and not a horizon ray.
  for (int j = 0; j < 120; ++j) {
    cm.setCost(80, j, 254);
  }
  const auto blocked = castGapRays(cm, 0.0, 0.0, 8, 2.0);
  for (const auto & r : blocked) {
    if (std::abs(r.bearing) < 1e-9) {
      EXPECT_FALSE(r.horizon);
      EXPECT_LT(r.clearance, 1.1);
    }
  }
}

TEST(GapPocket, SegmentBlockedUsesTheInscribedThresholdInclusively)
{
  nav2_costmap_2d::Costmap2D cm(40, 40, 0.1, -2.0, -2.0, 0);
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(cm.worldToMap(1.0, 0.0, mx, my));
  cm.setCost(mx, my, 252);
  EXPECT_FALSE(segmentBlocked(cm, 0.0, 0.0, 1.5, 0.0)) << "252 is below the threshold";
  cm.setCost(mx, my, 253);
  EXPECT_TRUE(segmentBlocked(cm, 0.0, 0.0, 1.5, 0.0)) << "253 blocks";
  cm.setCost(mx, my, 255);
  EXPECT_TRUE(segmentBlocked(cm, 0.0, 0.0, 1.5, 0.0)) << "unknown cells block";
  // Beyond the end of the segment does not count.
  EXPECT_FALSE(segmentBlocked(cm, 0.0, 0.0, 0.5, 0.0));
}

TEST(GapPocket, SegmentBlockedDoesNotTestItsStartPoint)
{
  // Cells are 0.1 m; the segment starts at x = 0.09, inside the cell [0.0, 0.1).
  // The first sample is half a cell on (x = 0.14, the next cell).
  nav2_costmap_2d::Costmap2D cm(40, 40, 0.1, -2.0, -2.0, 0);
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(cm.worldToMap(0.09, 0.0, mx, my));
  cm.setCost(mx, my, 254);  // the cell the segment starts in is lethal
  EXPECT_FALSE(segmentBlocked(cm, 0.09, 0.0, 1.0, 0.0));
  ASSERT_TRUE(cm.worldToMap(0.5, 0.0, mx, my));
  cm.setCost(mx, my, 254);
  EXPECT_TRUE(segmentBlocked(cm, 0.09, 0.0, 1.0, 0.0));
}
