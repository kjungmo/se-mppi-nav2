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

// Deployment tests for controller behaviour on a real robot stack: frames,
// velocity limits, footprint, replanning and costmap rate. The controller runs
// on a lifecycle node with the stock MPPI optimizer, as in controller_server.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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

class ProbeController : public nav2_se_controller::SafeEscapeController
{
public:
  const nav2_se_controller::CbfConfig & cbf() const {return filter_.config();}
  double robotRadius() const {return robot_radius_;}
  bool entrapped() const {return prev_entrapped_;}
  std::size_t trackCount() const {return tracker_.trackCount();}
};

struct Options
{
  std::vector<rclcpp::Parameter> overrides;
  std::string costmap_frame{"map"};
  std::string footprint;  // empty: circular robot_radius default
};

class ControllerBehaviour : public ::testing::Test
{
protected:
  void start(const Options & opt)
  {
    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("controller_frequency", 20.0),
      rclcpp::Parameter("FollowPath.batch_size", 300),
      rclcpp::Parameter("FollowPath.time_steps", 20),
      rclcpp::Parameter("FollowPath.model_dt", 0.05),
      rclcpp::Parameter("FollowPath.motion_model", std::string("DiffDrive")),
      rclcpp::Parameter(
        "FollowPath.critics", std::vector<std::string>{
          "ConstraintCritic", "CostCritic", "GoalCritic", "PathFollowCritic"}),
    };
    params.insert(params.end(), opt.overrides.begin(), opt.overrides.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_ros_->set_parameter(rclcpp::Parameter("global_frame", opt.costmap_frame));
    if (!opt.footprint.empty()) {
      costmap_ros_->set_parameter(rclcpp::Parameter("footprint", opt.footprint));
    }
    costmap_ros_->on_configure(rclcpp_lifecycle::State{});
    clearCostmap();

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    controller_ = std::make_shared<ProbeController>();
    controller_->configure(node_, "FollowPath", tf_, costmap_ros_);
    controller_->activate();
    active_ = true;
  }

  void TearDown() override
  {
    if (controller_ && active_) {
      controller_->deactivate();
      controller_->cleanup();
    }
    controller_.reset();
    costmap_ros_.reset();
    node_.reset();
  }

  // 6 m x 6 m around the origin of the costmap frame.
  void clearCostmap()
  {
    *(costmap_ros_->getCostmap()) = nav2_costmap_2d::Costmap2D(120, 120, 0.05, -3.0, -3.0, 0);
  }

  void block(unsigned int x0, unsigned int y0, unsigned int size)
  {
    auto * c = costmap_ros_->getCostmap();
    for (unsigned int x = x0; x < x0 + size; ++x) {
      for (unsigned int y = y0; y < y0 + size; ++y) {
        c->setCost(x, y, 254);
      }
    }
  }

  // Straight path along +x in `frame`, from (x0, y) to (x1, y).
  nav_msgs::msg::Path straightPath(const std::string & frame, double x0, double x1, double y)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = node_->now();
    const int n = static_cast<int>(std::round((x1 - x0) / 0.05));
    for (int i = 0; i <= n; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x0 + 0.05 * i;
      p.pose.position.y = y;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    return path;
  }

  geometry_msgs::msg::TwistStamped step(double x, double y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();
    pose.header.stamp = node_->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    geometry_msgs::msg::Twist speed;
    return controller_->computeVelocityCommands(pose, speed, nullptr);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<ProbeController> controller_;
  bool active_{false};
};

// Audit finding 3: the CBF filter clamped every command to hard-coded limits
// (v in [-0.35, 0.5], |w| <= 1.9) whatever MPPI was configured with.
TEST_F(ControllerBehaviour, CbfVelocityLimitsFollowMppiLimits)
{
  Options opt;
  opt.overrides = {
    rclcpp::Parameter("FollowPath.vx_max", 1.0),
    rclcpp::Parameter("FollowPath.vx_min", -0.2),
    rclcpp::Parameter("FollowPath.wz_max", 1.2)};
  start(opt);
  EXPECT_DOUBLE_EQ(controller_->cbf().v_max, 1.0);
  EXPECT_DOUBLE_EQ(controller_->cbf().v_min, -0.2);
  EXPECT_DOUBLE_EQ(controller_->cbf().w_max, 1.2);

  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.vx_max", 0.8)).successful);
  EXPECT_DOUBLE_EQ(controller_->cbf().v_max, 0.8);
  // A forward-only MPPI (vx_min > 0) must still let the filter brake to a stop.
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.vx_min", 0.1)).successful);
  EXPECT_DOUBLE_EQ(controller_->cbf().v_min, 0.0);
}

// Audit finding 8: the CBF disc used the footprint's INSCRIBED radius, so the
// corners of a rectangular robot were outside the certified safe set.
TEST_F(ControllerBehaviour, CbfRadiusCoversAPolygonFootprint)
{
  Options opt;
  opt.footprint = "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]";
  start(opt);
  // The costmap pads the footprint (footprint_padding), so take its value and
  // check it covers the corners.
  const double circumscribed = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  ASSERT_GE(circumscribed, std::hypot(0.3, 0.2));
  EXPECT_NEAR(controller_->robotRadius(), circumscribed, 1e-6);
  EXPECT_NEAR(controller_->cbf().robot_radius, circumscribed, 1e-6);
}

TEST_F(ControllerBehaviour, CbfRadiusCanBeSetExplicitly)
{
  Options opt;
  opt.footprint = "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]";
  opt.overrides = {rclcpp::Parameter("FollowPath.se_cbf_robot_radius", 0.42)};
  start(opt);
  EXPECT_DOUBLE_EQ(controller_->robotRadius(), 0.42);
  EXPECT_DOUBLE_EQ(controller_->cbf().robot_radius, 0.42);
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
