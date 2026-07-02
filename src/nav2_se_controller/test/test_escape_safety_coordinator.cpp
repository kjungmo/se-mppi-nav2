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

#include <vector>

#include "nav2_se_controller/escape_safety_coordinator.hpp"

using nav2_se_controller::CoordinationConfig;
using nav2_se_controller::EscapeSafetyCoordinator;
using nav2_se_controller::minTimeToCollision;
using nav2_se_controller::RobotState;
using nav2_se_controller::TrackedObstacle;

namespace
{
CoordinationConfig makeConfig()
{
  CoordinationConfig c;
  c.alpha_base = 2.0;
  c.alpha_escape = 6.0;
  c.ttc_override_threshold = 1.5;
  return c;
}

TrackedObstacle makeObstacle(double px, double py, double vx, double vy, double r)
{
  TrackedObstacle o;
  o.position = Eigen::Vector2d(px, py);
  o.velocity = Eigen::Vector2d(vx, vy);
  o.radius = r;
  return o;
}
}  // namespace

TEST(EscapeSafetyCoordinator, NotEntrappedUsesBaseAlpha)
{
  EscapeSafetyCoordinator c;
  c.configure(makeConfig());
  EXPECT_DOUBLE_EQ(c.alpha(false, 100.0), 2.0);
  EXPECT_FALSE(c.escapeSanctioned(false, 100.0));
}

TEST(EscapeSafetyCoordinator, EntrappedWithSafeTtcUsesEscapeAlpha)
{
  EscapeSafetyCoordinator c;
  c.configure(makeConfig());
  EXPECT_DOUBLE_EQ(c.alpha(true, 3.0), 6.0);  // TTC above threshold
  EXPECT_TRUE(c.escapeSanctioned(true, 3.0));
}

TEST(EscapeSafetyCoordinator, EntrappedWithImminentTtcOverridesToBase)
{
  EscapeSafetyCoordinator c;
  c.configure(makeConfig());
  EXPECT_DOUBLE_EQ(c.alpha(true, 0.3), 2.0);  // dynamic safety wins
  EXPECT_FALSE(c.escapeSanctioned(true, 0.3));
}

TEST(MinTimeToCollision, InfiniteWithNoObstacles)
{
  const double ttc = minTimeToCollision(RobotState{0, 0, 0}, 0.5, {}, 0.22);
  EXPECT_TRUE(std::isinf(ttc));
}

TEST(MinTimeToCollision, ApproachingStaticObstacleAhead)
{
  // Robot at origin facing +x at 0.5 m/s; static obstacle 2 m ahead.
  std::vector<TrackedObstacle> obs = {makeObstacle(2.0, 0.0, 0.0, 0.0, 0.1)};
  const double ttc = minTimeToCollision(RobotState{0, 0, 0}, 0.5, obs, 0.22);
  // clearance = 2 - 0.32 = 1.68; closing = 0.5 => ttc = 3.36 s
  EXPECT_NEAR(ttc, 1.68 / 0.5, 1e-6);
}

TEST(MinTimeToCollision, ObstacleMovingAwayIsInfinite)
{
  // Obstacle ahead but receding faster than the robot advances.
  std::vector<TrackedObstacle> obs = {makeObstacle(2.0, 0.0, 1.0, 0.0, 0.1)};
  const double ttc = minTimeToCollision(RobotState{0, 0, 0}, 0.5, obs, 0.22);
  EXPECT_TRUE(std::isinf(ttc));
}

TEST(EscapeSafetyCoordinator, ResolveAlphaIntegratesTtc)
{
  EscapeSafetyCoordinator c;
  c.configure(makeConfig());
  // Fast head-on obstacle => short TTC => override to base even when entrapped.
  std::vector<TrackedObstacle> obs = {makeObstacle(0.5, 0.0, -1.0, 0.0, 0.1)};
  const double a = c.resolveAlpha(true, RobotState{0, 0, 0}, 0.5, obs, 0.22);
  EXPECT_DOUBLE_EQ(a, 2.0);

  // Distant static obstacle => safe TTC => escape alpha.
  std::vector<TrackedObstacle> far = {makeObstacle(5.0, 0.0, 0.0, 0.0, 0.1)};
  const double a2 = c.resolveAlpha(true, RobotState{0, 0, 0}, 0.5, far, 0.22);
  EXPECT_DOUBLE_EQ(a2, 6.0);
}
