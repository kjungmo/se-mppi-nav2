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
