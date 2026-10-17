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

#include "nav2_se_controller/path_progress.hpp"

using nav2_se_controller::nearestPathIndex;

namespace
{
nav_msgs::msg::Path straightPath()
{
  nav_msgs::msg::Path path;
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = static_cast<double>(i);  // poses at x = 0,1,2,3,4
    p.pose.position.y = 0.0;
    path.poses.push_back(p);
  }
  return path;
}
}  // namespace

TEST(PathProgress, EmptyPathReturnsZero)
{
  nav_msgs::msg::Path path;
  EXPECT_EQ(nearestPathIndex(path, 1.0, 1.0), 0u);
}

TEST(PathProgress, FindsNearestIndex)
{
  const auto path = straightPath();
  EXPECT_EQ(nearestPathIndex(path, 2.1, 0.0), 2u);
  EXPECT_EQ(nearestPathIndex(path, 3.6, 0.2), 4u);
  EXPECT_EQ(nearestPathIndex(path, -1.0, 0.0), 0u);
}

// ---------------------------------------------------------------------------
// PathProgressTracker: progress on a per-task anchor path, replans as candidates.
// ---------------------------------------------------------------------------
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using nav2_se_controller::PathProgressTracker;
using nav2_se_controller::ProgressConfig;

namespace
{
using Pts = std::vector<std::pair<double, double>>;

nav_msgs::msg::Path polyline(const Pts & corners, double spacing = 0.05)
{
  nav_msgs::msg::Path path;
  auto add = [&path](double x, double y) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = x;
      p.pose.position.y = y;
      path.poses.push_back(p);
    };
  if (corners.empty()) {
    return path;
  }
  add(corners[0].first, corners[0].second);
  for (std::size_t i = 1; i < corners.size(); ++i) {
    const double dx = corners[i].first - corners[i - 1].first;
    const double dy = corners[i].second - corners[i - 1].second;
    const double len = std::hypot(dx, dy);
    const int n = std::max(1, static_cast<int>(std::lround(len / spacing)));
    for (int k = 1; k <= n; ++k) {
      add(corners[i - 1].first + dx * k / n, corners[i - 1].second + dy * k / n);
    }
  }
  return path;
}

PathProgressTracker makeTracker(const nav_msgs::msg::Path & anchor)
{
  PathProgressTracker t;
  ProgressConfig c;
  c.min_advance = 0.10;
  c.adopt_advance = 0.5;
  t.configure(c);
  t.startTask(anchor);
  return t;
}
}  // namespace

TEST(PathProgressTracker, NoAnchorNeverProducesEvents)
{
  PathProgressTracker t;
  EXPECT_FALSE(t.hasAnchor());
  EXPECT_EQ(t.update(1.0, 0.0), 0u);
  t.startTask(polyline({}));
  EXPECT_FALSE(t.hasAnchor());
  EXPECT_EQ(t.update(2.0, 0.0), 0u);
}

TEST(PathProgressTracker, FirstPositionIsABaselineNotProgress)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  EXPECT_EQ(t.update(2.0, 0.0), 0u);
  EXPECT_NEAR(t.bestArc(), 2.0, 1e-9);
}

TEST(PathProgressTracker, EventsFireOnlyAtTheMinimumAdvanceBothSides)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(0.5, 0.0);
  EXPECT_EQ(t.update(0.59, 0.0), 0u) << "0.09 m is below 0.10 m";
  EXPECT_EQ(t.update(0.61, 0.0), 1u) << "0.11 m is above 0.10 m";
  EXPECT_NEAR(t.bestArc(), 0.61, 1e-9);
  EXPECT_EQ(t.update(0.70, 0.0), 1u) << "0.09 m past the new best";
  EXPECT_EQ(t.update(0.72, 0.0), 2u);
}

TEST(PathProgressTracker, CreepingAccumulatesAgainstTheBestNotThePreviousCycle)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(0.0, 0.0);
  std::size_t events = 0;
  for (int k = 1; k <= 20; ++k) {
    events = t.update(0.03 * k, 0.0);  // 0.6 m in total
  }
  EXPECT_EQ(events, 5u);  // at 0.12, 0.24, 0.36, 0.48, 0.60
}

TEST(PathProgressTracker, MovingBackwardIsNotProgressUntilTheBestIsPassed)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  EXPECT_EQ(t.update(1.5, 0.0), 1u);
  EXPECT_EQ(t.update(0.6, 0.0), 1u);
  EXPECT_EQ(t.update(1.55, 0.0), 1u) << "0.05 past the best";
  EXPECT_EQ(t.update(1.65, 0.0), 2u);
}

TEST(PathProgressTracker, ASameGoalReplanFromTheRobotDoesNotRestartProgress)
{
  // The defect: progress re-anchored on each replan counted one path cell.
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  t.offerReplan(polyline({{1.0, 0.0}, {5, 0}}));
  EXPECT_EQ(t.update(1.0, 0.0), 0u);
  EXPECT_EQ(t.update(1.03, 0.0), 0u);
  EXPECT_EQ(t.update(1.05, 0.0), 0u);
  EXPECT_EQ(t.adoptions(), 0u);
}

TEST(PathProgressTracker, AReplanIsAdoptedAfterTheAdoptAdvanceNotBefore)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  // The replan leaves the anchor: up along x = 1 to y = 2, then across to the goal.
  t.offerReplan(polyline({{1.0, 0.0}, {1.0, 2.0}, {5.0, 2.0}, {5.0, 0.0}}));
  EXPECT_EQ(t.update(1.0, 0.0), 0u);
  EXPECT_EQ(t.update(1.0, 0.49), 0u) << "0.49 m along the replan";
  EXPECT_EQ(t.adoptions(), 0u);
  EXPECT_EQ(t.update(1.0, 0.51), 1u) << "0.51 m along the replan";
  EXPECT_EQ(t.adoptions(), 1u);
  // The adopted path is the anchor now: progress is measured along it.
  EXPECT_NEAR(t.bestArc(), 0.51, 1e-9);
  EXPECT_EQ(t.update(1.0, 0.55), 1u);
  EXPECT_EQ(t.update(1.0, 0.65), 2u);
}

TEST(PathProgressTracker, TheChainCarriesAcrossReplansAtOneHertz)
{
  // A detour followed at 0.3 m/s with a replan every second still adopts.
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  double y = 0.0;
  for (int second = 0; second < 4; ++second) {
    t.offerReplan(polyline({{1.0, y}, {1.0, 2.0}, {5.0, 2.0}, {5.0, 0.0}}));
    for (int k = 0; k < 10; ++k) {
      y += 0.03;
      t.update(1.0, y);
    }
  }
  EXPECT_EQ(t.adoptions(), 1u) << "1.2 m followed over four replans";
}

TEST(PathProgressTracker, MotionBackAndForthNeverAdopts)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  for (int round = 0; round < 40; ++round) {
    const double base = (round % 2 == 0) ? 1.0 : 0.7;
    t.offerReplan(polyline({{base, 0.0}, {5, 0}}));
    t.update(base, 0.0);
    t.update(base + ((round % 2 == 0) ? -0.3 : 0.3), 0.0);
  }
  EXPECT_EQ(t.adoptions(), 0u);
}

TEST(PathProgressTracker, AnAnchorEventRestartsTheChain)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  t.offerReplan(polyline({{1.0, 0.0}, {5, 0}}));
  t.update(1.0, 0.0);
  t.update(1.3, 0.0);  // an anchor event
  EXPECT_EQ(t.events(), 1u);
  EXPECT_NEAR(t.chainAdvance(), 0.0, 1e-9);
  EXPECT_EQ(t.adoptions(), 0u);
}

TEST(PathProgressTracker, StartTaskForgetsTheOldTask)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  t.update(2.0, 0.0);
  ASSERT_EQ(t.events(), 1u);
  t.startTask(polyline({{0, 0}, {0, 5}}));
  EXPECT_EQ(t.events(), 0u);
  EXPECT_EQ(t.update(0.0, 1.0), 0u) << "baseline on the new path";
  EXPECT_EQ(t.update(0.0, 1.2), 1u);
}

TEST(PathProgressTracker, DegenerateInputsAreHarmless)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  t.update(1.0, 0.0);
  EXPECT_EQ(t.update(nan, 0.0), 0u);
  EXPECT_EQ(t.update(1.0, inf), 0u);
  t.offerReplan(polyline({}));  // an empty replan is ignored
  EXPECT_EQ(t.update(1.0, 0.0), 0u);
  // A single-pose anchor never produces progress.
  auto single = makeTracker(polyline({{2.0, 2.0}}));
  EXPECT_TRUE(single.hasAnchor());
  EXPECT_EQ(single.update(2.0, 2.0), 0u);
  EXPECT_EQ(single.update(9.0, 9.0), 0u);
  // Non-finite and duplicate poses in a path are tolerated.
  auto path = polyline({{0, 0}, {1, 0}});
  geometry_msgs::msg::PoseStamped bad;
  bad.pose.position.x = nan;
  path.poses.insert(path.poses.begin() + 2, bad);
  path.poses.insert(path.poses.begin() + 3, path.poses[3]);  // duplicate
  auto tolerant = makeTracker(path);
  tolerant.update(0.2, 0.0);
  EXPECT_EQ(tolerant.update(0.35, 0.0), 1u);
}

TEST(PathProgressTracker, ProgressUsesTheNearestPointOnTheAnchorNotTheDistanceToTheGoal)
{
  // A robot that drives AWAY from the goal along a winding anchor still makes progress.
  auto t = makeTracker(polyline({{0, 0}, {0, 3}, {2, 3}, {2, 0}}));
  t.update(0.0, 0.5);
  EXPECT_EQ(t.update(0.0, 1.0), 1u);
  EXPECT_EQ(t.update(0.0, 2.0), 2u);
  EXPECT_EQ(t.update(1.0, 3.0), 3u);  // the goal is at (2, 0); the robot is not closer to it
}

TEST(PathProgressTracker, TheArcContinuesBackwardsBehindTheFirstPose)
{
  // The anchor starts at the robot; backing up is negative advance, and the
  // robot must then regain it before progress counts.
  auto t = makeTracker(polyline({{1.0, 0.0}, {5.0, 0.0}}));
  t.update(1.0, 0.0);
  t.update(0.4, 0.0);
  EXPECT_NEAR(t.bestArc(), 0.0, 1e-9);
  EXPECT_EQ(t.update(1.05, 0.0), 0u);
  EXPECT_EQ(t.update(1.11, 0.0), 1u);
}

TEST(PathProgressTracker, CandidateAdvanceIsSignedBehindItsFirstPose)
{
  auto t = makeTracker(polyline({{0, 0}, {5, 0}}));
  t.update(1.0, 0.0);
  t.offerReplan(polyline({{1.0, 0.0}, {5, 0}}));
  t.update(1.0, 0.0);
  t.update(0.7, 0.0);
  EXPECT_NEAR(t.chainAdvance(), -0.3, 1e-9);
}

TEST(PathProgressTracker, ThresholdsAreInclusiveAtExactlyRepresentableValues)
{
  // Spacing and positions are binary fractions, so the comparisons are exact.
  PathProgressTracker t;
  ProgressConfig c;
  c.min_advance = 0.25;
  c.adopt_advance = 1.0;
  t.configure(c);
  t.startTask(polyline({{0, 0}, {8, 0}}, 0.25));
  t.update(1.0, 0.0);
  EXPECT_EQ(t.update(1.1875, 0.0), 0u);
  EXPECT_EQ(t.update(1.25, 0.0), 1u) << "exactly the minimum advance counts";
  // Adoption exactly at the adoption distance.
  t.offerReplan(polyline({{1.25, 0.0}, {1.25, 4.0}, {8.0, 4.0}, {8.0, 0.0}}, 0.25));
  t.update(1.25, 0.0);
  EXPECT_EQ(t.update(1.25, 0.75), 1u);
  EXPECT_EQ(t.adoptions(), 0u);
  EXPECT_EQ(t.update(1.25, 1.0), 2u) << "exactly the adoption distance adopts";
  EXPECT_EQ(t.adoptions(), 1u);
}
