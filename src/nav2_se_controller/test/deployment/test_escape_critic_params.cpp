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

// Deployment test: a runtime set of EscapeCritic.progress_stall_window reaches
// the critic's (fallback) entrapment detector. It used to be bound to a local
// in initialize(), so the set wrote through a dangling reference.

#include <gtest/gtest.h>

#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_se_controller/escape_critic.hpp"

namespace
{

class ProbeCritic : public mppi::critics::EscapeCritic
{
public:
  int stallWindow() const
  {
    nav2_se_controller::EntrapmentDetector d = detector_;
    d.reset();
    d.update(0);
    int n = 0;
    do {
      ++n;
    } while (!d.update(0) && n < 100000);
    return n;
  }
};

TEST(EscapeCriticParams, StallWindowSetAtRuntimeReachesDetector)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State{});
  auto handler = std::make_unique<mppi::ParametersHandler>(node);

  ProbeCritic critic;
  critic.on_configure(node, "FollowPath", "FollowPath.EscapeCritic", costmap_ros, handler.get());
  handler->start();
  ASSERT_EQ(critic.stallWindow(), 30);

  const auto r = node->set_parameter(
    rclcpp::Parameter("FollowPath.EscapeCritic.progress_stall_window", 7));
  EXPECT_TRUE(r.successful) << r.reason;
  EXPECT_EQ(critic.stallWindow(), 7);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
