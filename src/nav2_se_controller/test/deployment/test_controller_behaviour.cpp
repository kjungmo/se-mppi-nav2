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
  int cbfObstacles() const {return counters_.cbf_obstacles.load();}
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
}  // namespace

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

// Review finding on the limits fix: Nav2's speed filter limits the controller
// through setSpeedLimit(). MPPI scales its own limits; the CBF box must follow,
// or the min-deviation QP could raise v up to vx_max inside a slow zone.
TEST_F(ControllerBehaviour, CbfBoxFollowsNav2SpeedLimit)
{
  Options opt;
  opt.overrides = {
    rclcpp::Parameter("FollowPath.vx_max", 1.0),
    rclcpp::Parameter("FollowPath.vx_min", -0.4),
    rclcpp::Parameter("FollowPath.wz_max", 2.0)};
  start(opt);

  controller_->setSpeedLimit(25.0, true);  // percent of the maximum
  EXPECT_DOUBLE_EQ(controller_->cbf().v_max, 0.25);
  EXPECT_DOUBLE_EQ(controller_->cbf().v_min, -0.1);
  EXPECT_DOUBLE_EQ(controller_->cbf().w_max, 0.5);

  controller_->setSpeedLimit(0.5, false);  // absolute m/s: ratio 0.5 / vx_max
  EXPECT_DOUBLE_EQ(controller_->cbf().v_max, 0.5);
  EXPECT_DOUBLE_EQ(controller_->cbf().v_min, -0.2);
  EXPECT_DOUBLE_EQ(controller_->cbf().w_max, 1.0);

  controller_->setSpeedLimit(0.0, false);  // nav2_costmap_2d::NO_SPEED_LIMIT
  EXPECT_DOUBLE_EQ(controller_->cbf().v_max, 1.0);
  EXPECT_DOUBLE_EQ(controller_->cbf().v_min, -0.4);
  EXPECT_DOUBLE_EQ(controller_->cbf().w_max, 2.0);
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

// Audit finding 2: the global plan arrives in the planner frame (map) while
// the robot pose is in the local costmap frame (odom). Entrapment progress and
// the near-goal suppression compared the two without a transform, so any
// map->odom offset broke them.
class ControllerFrames : public ControllerBehaviour
{
protected:
  void SetUp() override
  {
    Options opt;
    opt.costmap_frame = "odom";
    opt.overrides = {rclcpp::Parameter("FollowPath.se_progress_stall_window", 5)};
    start(opt);
    // odom's origin sits at (10, 0) in map: p_map = p_odom + (10, 0).
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.header.stamp = node_->now();
    t.child_frame_id = "odom";
    t.transform.translation.x = 10.0;
    t.transform.rotation.w = 1.0;
    tf_->setTransform(t, "test", true);
    controller_->setPlan(straightPath("map", 10.0, 12.0, 0.0));
  }
};

TEST_F(ControllerFrames, ProgressIsMeasuredInThePlanFrame)
{
  for (int k = 0; k < 30; ++k) {
    step(0.05 * k, 0.0);  // odom; drives along the plan
    EXPECT_FALSE(controller_->entrapped()) << "cycle " << k;
  }
}

TEST_F(ControllerFrames, StallMidPathIsStillDetected)
{
  bool entrapped = false;
  for (int k = 0; k < 20 && !entrapped; ++k) {
    step(0.5, 0.0);  // odom; stuck a quarter of the way along the plan
    entrapped = controller_->entrapped();
  }
  EXPECT_TRUE(entrapped);
}

TEST_F(ControllerFrames, NoEscapeWhenParkedAtTheGoal)
{
  for (int k = 0; k < 20; ++k) {
    step(2.0, 0.0);  // odom (2, 0) == map (12, 0), the goal
    EXPECT_FALSE(controller_->entrapped()) << "cycle " << k;
  }
}

// Audit finding 5: every setPlan() reset the entrapment detector, the obstacle
// tracker and its static-structure evidence. Nav2's default behaviour tree
// replans at 1 Hz towards the same goal, so a stall longer than ~1 s was never
// declared and tracks were rebuilt every second.
TEST_F(ControllerBehaviour, ReplanToTheSameGoalKeepsStallCountAndTracks)
{
  Options opt;
  opt.overrides = {rclcpp::Parameter("FollowPath.se_progress_stall_window", 5)};
  start(opt);
  block(80, 60, 4);  // an obstacle cluster away from the path
  controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  for (int k = 0; k < 3; ++k) {
    step(0.5, 0.0);  // stuck
  }
  ASSERT_FALSE(controller_->entrapped());
  ASSERT_EQ(controller_->trackCount(), 1u);

  // Replanned path from the robot to the SAME goal.
  controller_->setPlan(straightPath("map", 0.5, 2.0, 0.0));
  EXPECT_EQ(controller_->trackCount(), 1u);
  bool entrapped = false;
  for (int k = 0; k < 4; ++k) {
    step(0.5, 0.0);
    entrapped = entrapped || controller_->entrapped();
  }
  EXPECT_TRUE(entrapped);
}

TEST_F(ControllerBehaviour, NewGoalResetsTheTaskState)
{
  Options opt;
  opt.overrides = {rclcpp::Parameter("FollowPath.se_progress_stall_window", 5)};
  start(opt);
  block(80, 60, 4);
  controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  for (int k = 0; k < 8; ++k) {
    step(0.5, 0.0);
  }
  ASSERT_TRUE(controller_->entrapped());

  controller_->setPlan(straightPath("map", 0.5, 2.0, 1.0));  // different goal
  EXPECT_EQ(controller_->trackCount(), 0u);
  step(0.5, 0.0);
  EXPECT_FALSE(controller_->entrapped());
}

// Audit finding 4: the tracker was advanced every control cycle with the
// cycle's clock. When the controller runs faster than the costmap (typical:
// 20 Hz controller, 5 Hz local costmap) the repeated grid gave every moving
// obstacle zero velocity on the repeated cycles (and double velocity on the
// others), so it dropped out of the CBF on those cycles.
TEST_F(ControllerBehaviour, MovingObstacleStaysInTheCbfWhenTheCostmapIsSlower)
{
  start(Options{});
  controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  int missing = 0;
  for (int k = 0; k < 24; ++k) {
    if (k % 2 == 0) {  // the costmap updates every second control cycle
      clearCostmap();
      block(20 + k / 2, 60, 4);  // one 0.05 m cell per update, ~0.5 m/s
    }
    step(0.0, 0.0);
    if (k >= 4 && controller_->cbfObstacles() == 0) {
      ++missing;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(missing, 0) << "cycles on which the moving obstacle was not in the CBF";
}

// Review finding on the costmap-rate fix: while the grid stays byte-identical
// (a sensor dropout with the costmap still "current", or a still scene) the
// last tracked velocities must not be reused forever. An obstacle that moved
// and then stopped must leave the CBF once the grid has been frozen longer
// than the stale-grid timeout (default: two local-costmap update periods).
TEST_F(ControllerBehaviour, StoppedObstacleLeavesTheCbfWhenTheGridFreezes)
{
  start(Options{});  // local costmap update_frequency 5 Hz -> timeout 0.4 s
  controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  for (int k = 0; k < 12; ++k) {  // moving, the costmap refreshing every 2nd cycle
    if (k % 2 == 0) {
      clearCostmap();
      block(20 + k / 2, 60, 4);
    }
    step(0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_EQ(controller_->cbfObstacles(), 1) << "the moving obstacle must be in the CBF";

  // The obstacle stops and the grid freezes for 1.2 s (3x the timeout).
  int still_in_cbf_after_timeout = 0;
  const auto frozen_at = std::chrono::steady_clock::now();
  for (int k = 0; k < 24; ++k) {
    step(0.0, 0.0);
    const double frozen_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - frozen_at).count();
    if (frozen_s > 0.4 + 0.2 && controller_->cbfObstacles() != 0) {
      ++still_in_cbf_after_timeout;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(still_in_cbf_after_timeout, 0)
    << "cycles on which the stopped obstacle kept its old velocity in the CBF";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
