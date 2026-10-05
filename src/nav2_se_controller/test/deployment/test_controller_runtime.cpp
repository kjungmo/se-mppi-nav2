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

// Deployment test: the SafeEscapeController configured on a lifecycle node the
// way controller_server does it, with the stock MPPI optimizer loading the
// EscapeCritic through pluginlib.
//
// 1. Runs computeVelocityCommands with the critic injecting escape costs every
//    cycle. The critic writes MPPI's xtensor buffers across the plugin
//    boundary, so a SIMD/xtensor ABI mismatch between libescape_critic and the
//    installed libmppi_controller crashes here (CMakeLists.txt, boundary 1).
// 2. Runtime parameter sets: a member-bound parameter takes effect, a
//    config-struct parameter takes effect in its consumer, and a parameter that
//    cannot be re-applied is rejected with a reason. Never "successful" with no
//    effect.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_se_controller/safe_escape_controller.hpp"

namespace
{

// Exposes the consumers' applied configuration (read-only).
class ProbeController : public nav2_se_controller::SafeEscapeController
{
public:
  bool seEnabled() const {return se_enabled_;}
  double dynamicSpeedThreshold() const {return dynamic_speed_threshold_;}
  const nav2_se_controller::CoordinationConfig & coordination() const
  {
    return coordinator_.config();
  }
  const nav2_se_controller::CbfConfig & cbf() const {return filter_.config();}
  const nav2_se_controller::TrackerConfig & tracker() const {return tracker_.config();}
  const nav2_se_controller::MultiRobotConfig & multi() const {return multi_.config();}

  // Cycles without progress until a copy of the detector declares entrapment
  // (= the progress_stall_window it was configured with).
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

class ControllerRuntime : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {
        {"FollowPath.batch_size", 300},
        {"FollowPath.time_steps", 20},
        {"controller_frequency", 20.0},
        {"FollowPath.model_dt", 0.05},
        {"FollowPath.motion_model", std::string("DiffDrive")},
        {"FollowPath.critics", std::vector<std::string>{
            "ConstraintCritic", "CostCritic", "GoalCritic", "PathFollowCritic",
            "EscapeCritic"}},
        // Inject the escape cost every cycle so the critic writes data.costs.
        {"FollowPath.EscapeCritic.always_on", true},
        {"FollowPath.se_multirobot", true},
        {"FollowPath.se_neighbor_odom_topics",
          std::vector<std::string>{"/probe_neighbor/odom"}},
      });
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State{});
    // 6 m x 6 m around the origin with a lethal block next to the path.
    nav2_costmap_2d::Costmap2D grid(120, 120, 0.05, -3.0, -3.0, 0);
    for (unsigned int x = 70; x < 76; ++x) {
      for (unsigned int y = 64; y < 70; ++y) {
        grid.setCost(x, y, 254);
      }
    }
    *(costmap_ros_->getCostmap()) = grid;

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    controller_ = std::make_shared<ProbeController>();
    controller_->configure(node_, "FollowPath", tf_, costmap_ros_);
    controller_->activate();
    controller_active_ = true;

    nav_msgs::msg::Path path;
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    path.header.stamp = node_->now();
    for (int i = 0; i <= 50; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = 0.05 * i;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    controller_->setPlan(path);
  }

  void TearDown() override
  {
    if (controller_ && controller_active_) {
      controller_->deactivate();
      controller_->cleanup();
    }
    controller_.reset();
    costmap_ros_.reset();
    node_.reset();
  }

  geometry_msgs::msg::TwistStamped step()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();
    pose.header.stamp = node_->now();
    pose.pose.orientation.w = 1.0;
    geometry_msgs::msg::Twist speed;
    return controller_->computeVelocityCommands(pose, speed, nullptr);
  }

  rcl_interfaces::msg::SetParametersResult set(const rclcpp::Parameter & p)
  {
    return node_->set_parameter(p);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<ProbeController> controller_;
  bool controller_active_{false};
};

TEST_F(ControllerRuntime, RunsInsideStockMppiOptimizerWithEscapeCritic)
{
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::TwistStamped cmd;
    ASSERT_NO_THROW(cmd = step());
    EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
    EXPECT_TRUE(std::isfinite(cmd.twist.angular.z));
  }
}

TEST_F(ControllerRuntime, MemberBoundParameterTakesEffect)
{
  ASSERT_TRUE(controller_->seEnabled());
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_enabled", false)).successful);
  EXPECT_FALSE(controller_->seEnabled());
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_dynamic_speed_threshold", 0.3)).successful);
  EXPECT_DOUBLE_EQ(controller_->dynamicSpeedThreshold(), 0.3);
}

TEST_F(ControllerRuntime, ConfigParameterTakesEffectInItsConsumer)
{
  ASSERT_DOUBLE_EQ(controller_->coordination().alpha_base, 2.0);
  ASSERT_DOUBLE_EQ(controller_->cbf().alpha, 2.0);
  ASSERT_EQ(controller_->stallWindow(), 30);

  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_alpha_base", 3.5)).successful);
  EXPECT_DOUBLE_EQ(controller_->coordination().alpha_base, 3.5);
  EXPECT_DOUBLE_EQ(controller_->cbf().alpha, 3.5);  // the filter's default gain follows

  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_cbf_lookahead", 0.3)).successful);
  EXPECT_DOUBLE_EQ(controller_->cbf().lookahead, 0.3);

  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_progress_stall_window", 12)).successful);
  EXPECT_EQ(controller_->stallWindow(), 12);

  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_track_history", 7)).successful);
  EXPECT_EQ(controller_->tracker().history_length, 7);
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_obstacle_cost_threshold", 300)).successful);
  EXPECT_EQ(controller_->tracker().cost_threshold, 254);  // clamped as at configure
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_predict_model", "cvca")).successful);
  EXPECT_EQ(
    controller_->tracker().predictor.model,
    nav2_se_controller::PredictorConfig::Model::kConstantAcceleration);

  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.se_yield_v_max", 0.05)).successful);
  EXPECT_DOUBLE_EQ(controller_->multi().yield_v_max, 0.05);

  // The controller keeps running with the new values.
  EXPECT_NO_THROW(step());
}

TEST_F(ControllerRuntime, ParameterThatCannotBeReappliedIsRejected)
{
  const auto r = set(
    rclcpp::Parameter(
      "FollowPath.se_neighbor_odom_topics", std::vector<std::string>{"/other/odom"}));
  EXPECT_FALSE(r.successful);
  EXPECT_NE(r.reason.find("configure"), std::string::npos) << r.reason;
  EXPECT_EQ(
    node_->get_parameter("FollowPath.se_neighbor_odom_topics").as_string_array(),
    std::vector<std::string>{"/probe_neighbor/odom"});
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
