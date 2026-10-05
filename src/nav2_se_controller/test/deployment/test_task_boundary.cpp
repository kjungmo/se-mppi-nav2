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

// Deployment test: the idle-loop task boundary (ROS-free).

#include <gtest/gtest.h>

#include "nav2_se_controller/task_boundary.hpp"

using nav2_se_controller::TaskBoundary;

TEST(TaskBoundary, PlanUpdateInsideTheLoopIsNotANewTask)
{
  TaskBoundary b;
  b.configure(1.0, 20.0);
  for (double t = 0.0; t < 3.0; t += 0.05) {
    b.controlReturned(t);
  }
  EXPECT_FALSE(b.newPlanStartsTask(3.0));  // 1 Hz replanning while driving
}

TEST(TaskBoundary, PlanAfterAnIdleGapIsANewTask)
{
  TaskBoundary b;
  b.configure(1.0, 20.0);
  b.controlReturned(10.0);
  EXPECT_FALSE(b.newPlanStartsTask(10.9));
  EXPECT_TRUE(b.newPlanStartsTask(11.2));  // retry after the goal ended
}

TEST(TaskBoundary, GapWithoutANewPlanResetsNothing)
{
  TaskBoundary b;
  b.configure(1.0, 20.0);
  b.controlReturned(0.0);
  b.controlReturned(5.0);  // the loop resumed (e.g. a stall) with no new plan
  EXPECT_FALSE(b.newPlanStartsTask(5.05));
}

TEST(TaskBoundary, ThresholdIsNeverBelowThreeControlPeriods)
{
  TaskBoundary b;
  b.configure(0.01, 20.0);
  EXPECT_DOUBLE_EQ(b.effectiveThreshold(), 0.15);
  b.configure(0.01, 2.0);
  EXPECT_DOUBLE_EQ(b.effectiveThreshold(), 1.5);
  b.controlReturned(0.0);
  EXPECT_FALSE(b.newPlanStartsTask(1.0));  // two slow cycles are not idle
}

TEST(TaskBoundary, NoControlYetIsNotANewTask)
{
  TaskBoundary b;
  b.configure(1.0, 20.0);
  EXPECT_FALSE(b.newPlanStartsTask(100.0));  // the first plan is handled as a new goal
}
