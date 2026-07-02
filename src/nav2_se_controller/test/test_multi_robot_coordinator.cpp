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

#include "nav2_se_controller/multi_robot_coordinator.hpp"

using nav2_se_controller::MultiRobotConfig;
using nav2_se_controller::MultiRobotCoordinator;
using nav2_se_controller::NeighborRobot;
using nav2_se_controller::RobotState;
using nav2_se_controller::TrackedObstacle;

using Role = MultiRobotCoordinator::Role;

namespace
{
NeighborRobot makeNeighbor(
  int id, double x, double y, double vx = 0.0,
  double vy = 0.0)
{
  NeighborRobot n;
  n.id = id;
  n.position = Eigen::Vector2d(x, y);
  n.velocity = Eigen::Vector2d(vx, vy);
  n.valid = true;
  return n;
}

TrackedObstacle makeObstacle(double x, double y)
{
  TrackedObstacle o;
  o.position = Eigen::Vector2d(x, y);
  o.radius = 0.22;
  return o;
}
}  // namespace

TEST(MultiRobotCoordinator, MarksOnlyMatchingObstacles)
{
  MultiRobotCoordinator c;
  c.configure(MultiRobotConfig{});

  std::vector<TrackedObstacle> obstacles = {
    makeObstacle(1.0, 0.0),     // a neighbor robot is here
    makeObstacle(3.0, 2.0),     // an unrelated dynamic obstacle
  };
  const std::vector<NeighborRobot> neighbors = {makeNeighbor(1, 1.1, 0.1)};

  EXPECT_EQ(c.markNeighbors(obstacles, neighbors), 1);
  EXPECT_DOUBLE_EQ(obstacles[0].responsibility, 0.5);   // reciprocal share
  EXPECT_DOUBLE_EQ(obstacles[1].responsibility, 1.0);   // single-robot default
}

TEST(MultiRobotCoordinator, InvalidNeighborsNeverMatch)
{
  MultiRobotCoordinator c;
  c.configure(MultiRobotConfig{});
  std::vector<TrackedObstacle> obstacles = {makeObstacle(1.0, 0.0)};
  std::vector<NeighborRobot> neighbors = {makeNeighbor(1, 1.0, 0.0)};
  neighbors[0].valid = false;   // no odom received yet
  EXPECT_EQ(c.markNeighbors(obstacles, neighbors), 0);
  EXPECT_DOUBLE_EQ(obstacles[0].responsibility, 1.0);
}

TEST(MultiRobotCoordinator, LowerIdPassesHigherYields)
{
  MultiRobotConfig cfg;
  MultiRobotCoordinator me;       // id 0 vs blocker id 1 -> I pass
  me.configure(cfg);
  MultiRobotCoordinator other;    // id 1 vs blocker id 0 -> I yield
  other.configure(cfg);

  const RobotState pos{0.0, 0.0, 0.0};
  const Eigen::Vector2d goal(5.0, 0.0);
  const auto blocker1 = makeNeighbor(1, 1.0, 0.0);   // stalled ahead
  const auto blocker0 = makeNeighbor(0, 1.0, 0.0);

  EXPECT_EQ(me.update(0, pos, goal, 0.01, true, {blocker1}), Role::kPass);
  EXPECT_EQ(other.update(1, pos, goal, 0.01, true, {blocker0}), Role::kYield);
  EXPECT_DOUBLE_EQ(me.lambdaForRole(), cfg.pass_lambda);
  EXPECT_DOUBLE_EQ(other.lambdaForRole(), cfg.yield_lambda);
  // The pair's budget shares sum to 1: the joint barrier stays class-K.
  EXPECT_DOUBLE_EQ(me.lambdaForRole() + other.lambdaForRole(), 1.0);
}

TEST(MultiRobotCoordinator, NoDeadlockWithoutStalledNeighbor)
{
  MultiRobotCoordinator c;
  c.configure(MultiRobotConfig{});
  const RobotState pos{0.0, 0.0, 0.0};
  const Eigen::Vector2d goal(5.0, 0.0);

  // Entrapped but the neighbor is MOVING -> not a mutual deadlock.
  const auto moving = makeNeighbor(1, 1.0, 0.0, 0.5, 0.0);
  EXPECT_EQ(c.update(0, pos, goal, 0.01, true, {moving}), Role::kNone);

  // Stalled neighbor but I am MOVING -> not a deadlock either.
  const auto stalled = makeNeighbor(1, 1.0, 0.0);
  EXPECT_EQ(c.update(0, pos, goal, 0.4, true, {stalled}), Role::kNone);

  // Stalled neighbor but FAR away -> no deadlock.
  const auto far = makeNeighbor(1, 4.0, 0.0);
  EXPECT_EQ(c.update(0, pos, goal, 0.01, true, {far}), Role::kNone);

  // Not entrapped -> never a deadlock.
  EXPECT_EQ(c.update(0, pos, goal, 0.01, false, {stalled}), Role::kNone);
}

TEST(MultiRobotCoordinator, RoleLatchesUntilConflictClears)
{
  MultiRobotCoordinator c;
  c.configure(MultiRobotConfig{});
  const Eigen::Vector2d goal(5.0, 0.0);
  const auto blocker = makeNeighbor(1, 1.0, 0.0);

  // Deadlock -> pass role.
  RobotState pos{0.0, 0.0, 0.0};
  ASSERT_EQ(c.update(0, pos, goal, 0.01, true, {blocker}), Role::kPass);

  // Mid-maneuver the robot regains speed and entrapment clears, but the
  // blocker is still ahead within range: the role must LATCH (an unlatched
  // role re-triggers the oscillation it exists to remove).
  EXPECT_EQ(c.update(0, pos, goal, 0.4, false, {blocker}), Role::kPass);

  // Once the blocker is BEHIND (passed), the conflict clears.
  RobotState past{2.0, 0.0, 0.0};
  EXPECT_EQ(c.update(0, past, goal, 0.4, false, {blocker}), Role::kNone);
}

TEST(MultiRobotCoordinator, MarkingUsesRoleLambda)
{
  MultiRobotConfig cfg;
  MultiRobotCoordinator c;
  c.configure(cfg);
  const Eigen::Vector2d goal(5.0, 0.0);
  const auto blocker = makeNeighbor(1, 1.0, 0.0);
  RobotState pos{0.0, 0.0, 0.0};
  ASSERT_EQ(c.update(0, pos, goal, 0.01, true, {blocker}), Role::kPass);

  std::vector<TrackedObstacle> obstacles = {makeObstacle(1.0, 0.0)};
  c.markNeighbors(obstacles, {blocker});
  EXPECT_DOUBLE_EQ(obstacles[0].responsibility, cfg.pass_lambda);
}

TEST(MultiRobotCoordinator, ResetClearsRole)
{
  MultiRobotCoordinator c;
  c.configure(MultiRobotConfig{});
  const Eigen::Vector2d goal(5.0, 0.0);
  RobotState pos{0.0, 0.0, 0.0};
  ASSERT_EQ(
    c.update(0, pos, goal, 0.01, true, {makeNeighbor(1, 1.0, 0.0)}),
    Role::kPass);
  c.reset();
  EXPECT_EQ(c.role(), Role::kNone);
}
