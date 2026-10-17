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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"
#include "utrap_dev_geometries.hpp"
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
  std::uint64_t baseFailures() const {return counters_.base_failures.load();}
  std::uint64_t escapeEntries() const {return counters_.escape_entries.load();}
  std::size_t progressEvents() const {return progress_.events();}
  std::size_t adoptions() const {return progress_.adoptions();}
  bool hasAnchor() const {return progress_.hasAnchor();}
  // Escape path substitution.
  std::vector<nav_msgs::msg::Path> handed;  // every path given to MPPI
  void handNominalPath(const nav_msgs::msg::Path & path) override
  {
    handed.push_back(path);
    nav2_se_controller::SafeEscapeController::handNominalPath(path);
  }
  bool escapeActive() const {return escape_.active;}
  int escapeHop() const {return escape_.hop;}
  std::uint64_t hops() const {return counters_.escape_hops.load();}
  double escapeBearing() const {return escape_.bearing;}
  bool vxMinBlocks() const {return vxMinBlocksReversal();}
  // Scripted nominal command: throw instead of calling the optimizer.
  enum class Throw { kNone, kNoValidControl, kRuntimeError };
  Throw throw_mode{Throw::kNone};
  geometry_msgs::msg::TwistStamped nominalCommand(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override
  {
    if (throw_mode == Throw::kNoValidControl) {
      throw nav2_core::NoValidControl("scripted: no valid control");
    }
    if (throw_mode == Throw::kRuntimeError) {
      throw std::runtime_error("scripted: another failure");
    }
    return nav2_se_controller::SafeEscapeController::nominalCommand(pose, speed, goal_checker);
  }
  double trackedSpeed() const
  {
    return last_tracked_.empty() ? 0.0 : last_tracked_.front().velocity.norm();
  }
  // Stand-in for a cycle whose CBF QP failed (forcing a real QP failure needs
  // a degenerate problem), and the rule behind the ERROR status.
  void recordQpFailure() {qp_failure_streak_.record(true, steady_clock_.now().nanoseconds());}
  bool qpErrorWouldFire()
  {
    return qp_failure_streak_.longerThan(
      steady_clock_.now().nanoseconds(),
      static_cast<std::int64_t>(kQpFailureErrorAfterSec * 1e9),
      static_cast<std::int64_t>(kQpFailureStaleSec * 1e9));
  }
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

// Review finding: a 1 mm tolerance treated every replan of a planner with a
// goal tolerance (or a blocked goal) as a new goal. The tolerance is now two
// costmap cells; the frame must match too.
TEST_F(ControllerBehaviour, GoalMovedByOneCellIsTheSameGoal)
{
  Options opt;
  start(opt);
  block(80, 60, 4);
  controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  step(0.5, 0.0);
  ASSERT_EQ(controller_->trackCount(), 1u);

  controller_->setPlan(straightPath("map", 0.5, 2.05, 0.0));  // endpoint moved 0.05 m
  EXPECT_EQ(controller_->trackCount(), 1u) << "one-cell goal jitter must not reset";

  controller_->setPlan(straightPath("odom", 0.5, 2.05, 0.0));  // same numbers, other frame
  EXPECT_EQ(controller_->trackCount(), 0u) << "a goal in another frame is a new goal";
}

// Review finding: Humble's controller_server never calls reset(), so with
// "reset only on goal change" a retry to the same goal after the previous task
// ended would inherit its stall count and tracks. A plan arriving after the
// loop was idle longer than se_task_idle_threshold starts a new task.
TEST_F(ControllerBehaviour, SameGoalAfterAnIdleLoopIsANewTask)
{
  Options opt;
  opt.overrides = {
    rclcpp::Parameter("FollowPath.se_progress_stall_window", 5),
    rclcpp::Parameter("FollowPath.se_task_idle_threshold", 0.3)};
  start(opt);
  block(80, 60, 4);
  controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  for (int k = 0; k < 8; ++k) {
    step(0.5, 0.0);
  }
  ASSERT_TRUE(controller_->entrapped());
  ASSERT_EQ(controller_->trackCount(), 1u);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // the task ended
  controller_->setPlan(straightPath("map", 0.5, 2.0, 0.0));     // retry, same goal
  EXPECT_EQ(controller_->trackCount(), 0u);
  step(0.5, 0.0);
  EXPECT_FALSE(controller_->entrapped());
}

// Review finding: clearing the QP-failure streak on every new goal hid a
// persistent failure when the goal moves at replanning rate (follow /
// moving-goal trees). Only a task boundary (idle loop) ends the streak.
TEST_F(ControllerBehaviour, GoalUpdatesDuringATaskKeepTheQpFailureStreak)
{
  Options opt;
  opt.overrides = {rclcpp::Parameter("FollowPath.se_task_idle_threshold", 10.0)};
  start(opt);
  bool error = false;
  const auto t0 = std::chrono::steady_clock::now();
  for (int k = 0; std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(2600);
    ++k)
  {
    if (k % 10 == 0) {  // a new goal every 0.5 s, 0.2 m apart
      controller_->setPlan(straightPath("map", 0.0, 2.0 + 0.2 * (k / 10), 0.0));
    }
    controller_->recordQpFailure();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  error = controller_->qpErrorWouldFire();
  EXPECT_TRUE(error) << "the 2 s ERROR must survive goal updates within a task";
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

// The costmap-rate tests below drive the controller's ROS clock directly
// (rcl time override) instead of sleeping, so a loaded CI runner cannot
// stretch a 50 ms control period past the 0.4 s stale-grid timeout.
class ControllerCostmapRate : public ControllerBehaviour
{
protected:
  void SetUp() override
  {
    start(Options{});  // local costmap update_frequency 5 Hz -> stale-grid timeout 0.4 s
    rcl_clock_t * clock = node_->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    setTime(100.0);
    controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  }

  void setTime(double t)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(
        node_->get_clock()->get_clock_handle(),
        static_cast<rcl_time_point_value_t>(std::llround(t * 1e9))),
      RCL_RET_OK);
  }

  // The obstacle as a 4x4 block whose left cell is `cell`, drawn into a fresh grid.
  void drawObstacleAt(unsigned int cell)
  {
    clearCostmap();
    block(cell, 60, 4);
  }
};

// Audit finding 4: the tracker was advanced every control cycle with the
// cycle's clock. When the controller runs faster than the costmap (typical:
// 20 Hz controller, 5 Hz local costmap) the repeated grid gave every moving
// obstacle zero velocity on the repeated cycles (and double velocity on the
// others), so it dropped out of the CBF on those cycles.
TEST_F(ControllerCostmapRate, MovingObstacleStaysInTheCbfWhenTheCostmapIsSlower)
{
  int missing = 0;
  for (int k = 0; k < 24; ++k) {
    setTime(100.0 + 0.05 * k);
    if (k % 2 == 0) {  // the costmap updates every second control cycle
      drawObstacleAt(20 + k / 2);  // one 0.05 m cell per update, 0.5 m/s
    }
    step(0.0, 0.0);
    if (k >= 4 && controller_->cbfObstacles() == 0) {
      ++missing;
    }
  }
  EXPECT_EQ(missing, 0) << "cycles on which the moving obstacle was not in the CBF";
}

// Review finding on the costmap-rate fix: while the grid stays byte-identical
// (a sensor dropout with the costmap still "current", or a still scene) the
// last tracked velocities must not be reused forever. An obstacle that moved
// and then stopped must leave the CBF once the grid has been frozen longer
// than the stale-grid timeout (default: two local-costmap update periods).
TEST_F(ControllerCostmapRate, StoppedObstacleLeavesTheCbfWhenTheGridFreezes)
{
  double t = 100.0;
  for (int k = 0; k < 12; ++k, t += 0.05) {  // moving, the costmap refreshing every 2nd cycle
    setTime(t);
    if (k % 2 == 0) {
      drawObstacleAt(20 + k / 2);
    }
    step(0.0, 0.0);
  }
  ASSERT_EQ(controller_->cbfObstacles(), 1) << "the moving obstacle must be in the CBF";

  // The obstacle stops and the grid freezes for 1.2 s (3x the timeout).
  const double frozen_at = t;
  int still_in_cbf_after_timeout = 0;
  for (int k = 0; k < 24; ++k, t += 0.05) {
    setTime(t);
    step(0.0, 0.0);
    if (t - frozen_at > 0.4 + 0.05 && controller_->cbfObstacles() != 0) {
      ++still_in_cbf_after_timeout;
    }
  }
  EXPECT_EQ(still_in_cbf_after_timeout, 0)
    << "cycles on which the stopped obstacle kept its old velocity in the CBF";
}

// Known band of the stale-grid rule (review): a slow obstacle that changes
// cells less often than the timeout. At 0.11 m/s with 0.05 m cells it crosses a
// cell every ~0.45 s; after 0.4 s of an unchanged grid the tracker is fed the
// repeated grid and reads zero velocity until the next cell change, so the
// obstacle leaves the CBF for those cycles. This documents the band and checks
// it is no worse than the b21b0a5 behaviour (tracker advanced every cycle),
// emulated here with a second tracker fed the same grids.
TEST_F(ControllerCostmapRate, SlowMoverIsInTheCbfMoreOftenThanBefore)
{
  nav2_se_controller::DynamicObstacleTracker before;
  before.configure(nav2_se_controller::TrackerConfig{});
  const double speed = 0.11;
  int cycles = 0;
  int out_now = 0;
  int out_before = 0;
  double speed_sum = 0.0;
  double speed_max = 0.0;
  int speed_n = 0;
  for (int k = 0; k < 200; ++k) {  // 10 s at 20 Hz
    const double t = 100.0 + 0.05 * k;
    setTime(t);
    if (k % 4 == 0) {  // 5 Hz costmap: redraw at the current (cell-quantized) position
      drawObstacleAt(20 + static_cast<unsigned int>(speed * 0.05 * k / 0.05));
    }
    step(0.0, 0.0);
    // b21b0a5 behaviour: every cycle, same grid, same stamp; same CBF admission
    // rule as the controller (dynamic, >= 0.1 m/s, radius <= 1 m).
    bool in_before = false;
    for (const auto & o : before.update(*costmap_ros_->getCostmap(), t)) {
      in_before = in_before || (o.is_dynamic && o.velocity.norm() >= 0.1 && o.radius <= 1.0);
    }
    if (k >= 20) {  // after 1 s of warm-up
      ++cycles;
      out_now += controller_->cbfObstacles() == 0 ? 1 : 0;
      if (controller_->cbfObstacles() != 0) {
        speed_sum += controller_->trackedSpeed();
        speed_max = std::max(speed_max, controller_->trackedSpeed());
        ++speed_n;
      }
      out_before += in_before ? 0 : 1;
    }
  }
  const double ratio_now = static_cast<double>(out_now) / cycles;
  const double ratio_before = static_cast<double>(out_before) / cycles;
  std::printf(
    "slow mover 0.11 m/s: out of the CBF on %.0f %% of cycles now, %.0f %% with b21b0a5's rule\n",
    100.0 * ratio_now, 100.0 * ratio_before);
  // While in the CBF its speed is over-estimated (conservative: extra braking).
  std::printf(
    "slow mover tracked speed while in the CBF: mean %.2f m/s, max %.2f m/s (true %.2f)\n",
    speed_n > 0 ? speed_sum / speed_n : 0.0, speed_max, speed);
  EXPECT_GE(speed_sum / std::max(speed_n, 1), speed * 0.9);
  EXPECT_LE(ratio_now, ratio_before);
  EXPECT_LE(ratio_now, 0.25);
}

// ---------------------------------------------------------------------------
// Defect 2 (campaign 1, ROOT_CAUSE.md): the optimizer throws NoValidControl
// when every rollout collides, and the entrapment detector, which ran after the
// base call, never saw the stall.
// ---------------------------------------------------------------------------
class ControllerNoValidControl : public ControllerBehaviour
{
protected:
  void SetUp() override
  {
    Options opt;
    opt.overrides = {rclcpp::Parameter("FollowPath.se_progress_stall_window", 5)};
    start(opt);
    controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  }
  // true if the cycle threw NoValidControl.
  bool stepThrows(double x, double y)
  {
    try {
      step(x, y);
    } catch (const nav2_core::NoValidControl &) {
      return true;
    }
    return false;
  }
};

// The real optimizer: the whole costmap lethal, so every rollout collides.
TEST_F(ControllerNoValidControl, LethalCostmapEveryCycleThrowsAndEntrapmentIsDeclared)
{
  block(0, 0, 120);
  int threw = 0;
  int first_entrapped = -1;
  for (int k = 0; k < 60; ++k) {
    threw += stepThrows(0.5, 0.0) ? 1 : 0;
    if (first_entrapped < 0 && controller_->entrapped()) {
      first_entrapped = k;
    }
  }
  EXPECT_EQ(threw, 60) << "the exception must still reach controller_server";
  EXPECT_TRUE(controller_->entrapped());
  // The first cycle only records the baseline; the window of 5 stalled cycles
  // then ends on the sixth cycle (index 5).
  EXPECT_EQ(first_entrapped, 5);
  EXPECT_EQ(controller_->baseFailures(), 60u);
  EXPECT_EQ(controller_->escapeEntries(), 1u);
}

// The hook: a scripted NoValidControl on every cycle.
TEST_F(ControllerNoValidControl, HookThrowingNoValidControlFeedsTheDetectorAndRethrows)
{
  controller_->throw_mode = ProbeController::Throw::kNoValidControl;
  for (int k = 0; k < 4; ++k) {
    EXPECT_THROW(step(0.5, 0.0), nav2_core::NoValidControl);
    EXPECT_FALSE(controller_->entrapped()) << "cycle " << k << " is inside the window";
  }
  for (int k = 4; k < 12; ++k) {
    EXPECT_THROW(step(0.5, 0.0), nav2_core::NoValidControl);
  }
  EXPECT_TRUE(controller_->entrapped());
  EXPECT_EQ(controller_->baseFailures(), 12u);
}

// Progress during failing cycles is still progress: a robot that advances
// along the plan is not declared entrapped, and entrapment clears.
TEST_F(ControllerNoValidControl, AdvancingWhileTheOptimizerFailsIsNotEntrapment)
{
  controller_->throw_mode = ProbeController::Throw::kNoValidControl;
  for (int k = 0; k < 30; ++k) {
    EXPECT_THROW(step(0.05 * k, 0.0), nav2_core::NoValidControl);
    EXPECT_FALSE(controller_->entrapped()) << "cycle " << k;
  }
}

TEST_F(ControllerNoValidControl, EntrapmentFromFailingCyclesClearsOnProgress)
{
  controller_->throw_mode = ProbeController::Throw::kNoValidControl;
  for (int k = 0; k < 10; ++k) {
    EXPECT_THROW(step(0.5, 0.0), nav2_core::NoValidControl);
  }
  ASSERT_TRUE(controller_->entrapped());
  EXPECT_THROW(step(0.65, 0.0), nav2_core::NoValidControl);  // 0.15 m further along
  EXPECT_FALSE(controller_->entrapped());
}

// Only NoValidControl is stall evidence; another failure passes through and
// leaves the detector alone.
TEST_F(ControllerNoValidControl, OtherExceptionsPassThroughWithoutTouchingTheDetector)
{
  controller_->throw_mode = ProbeController::Throw::kRuntimeError;
  for (int k = 0; k < 20; ++k) {
    EXPECT_THROW(step(0.5, 0.0), std::runtime_error);
  }
  EXPECT_FALSE(controller_->entrapped());
  EXPECT_EQ(controller_->baseFailures(), 0u);
}

TEST_F(ControllerNoValidControl, WithTheSeLayerDisabledTheExceptionPassesThroughUntouched)
{
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.se_enabled", false)).successful);
  controller_->throw_mode = ProbeController::Throw::kNoValidControl;
  for (int k = 0; k < 20; ++k) {
    EXPECT_THROW(step(0.5, 0.0), nav2_core::NoValidControl);
  }
  EXPECT_FALSE(controller_->entrapped());
}

// Near the goal a failing optimizer is not entrapment (the robot is finishing).
TEST_F(ControllerNoValidControl, FailingCyclesAtTheGoalAreNotEntrapment)
{
  controller_->throw_mode = ProbeController::Throw::kNoValidControl;
  for (int k = 0; k < 20; ++k) {
    EXPECT_THROW(step(2.0, 0.0), nav2_core::NoValidControl);
  }
  EXPECT_FALSE(controller_->entrapped());
}

// ---------------------------------------------------------------------------
// Defect 3 (campaign 1, ROOT_CAUSE.md): Nav2's behaviour tree replans a path
// that starts at the robot every second. Progress used to be re-anchored on
// each new path, so one path cell (0.025 m) of motion counted as progress and
// ended the escape. Progress is now measured on the task's first plan (the
// anchor); a replan becomes the anchor only after the robot has advanced
// se_progress_adopt_advance (0.5 m) along it.
// ---------------------------------------------------------------------------
class ControllerReplan : public ControllerBehaviour
{
protected:
  void begin(int stall_window)
  {
    Options opt;
    opt.overrides = {rclcpp::Parameter("FollowPath.se_progress_stall_window", stall_window)};
    start(opt);
    controller_->setPlan(straightPath("map", 0.0, 2.0, 0.0));
  }
  // A 1 Hz replan: a fresh straight path from the robot to the same goal.
  void replanFrom(double x) {controller_->setPlan(straightPath("map", x, 2.0, 0.0));}

  // A path from (x, 0) that leaves the original line: up to y = 1.5, across,
  // and back down to the same goal (2, 0). Poses every 0.05 m.
  nav_msgs::msg::Path detourPath(double x)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    auto add = [&](double px, double py) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = px;
      p.pose.position.y = py;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    };
    for (double y = 0.0; y < 1.5; y += 0.05) {add(x, y);}
    for (double px = x; px < 2.0; px += 0.05) {add(px, 1.5);}
    for (double y = 1.5; y > 0.0; y -= 0.05) {add(2.0, y);}
    add(2.0, 0.0);
    return path;
  }
};

// (i) A robot that does not move, with the path replaced every 10 cycles by a
// new path that starts at the robot: entrapped after the window, and still
// entrapped 50 cycles later.
TEST_F(ControllerReplan, ReplansFromAStationaryRobotDoNotCancelEntrapment)
{
  begin(5);
  int first_entrapped = -1;
  for (int k = 0; k < 70; ++k) {
    if (k % 10 == 0 && k > 0) {replanFrom(0.5);}
    step(0.5, 0.0);
    if (first_entrapped < 0 && controller_->entrapped()) {first_entrapped = k;}
    if (k >= 5) {EXPECT_TRUE(controller_->entrapped()) << "cycle " << k;}
  }
  EXPECT_EQ(first_entrapped, 5);
  EXPECT_EQ(controller_->adoptions(), 0u);
  EXPECT_EQ(controller_->progressEvents(), 0u);
}

// (ii) After entrapment, motion along the new paths that adds up to less than
// se_progress_min_advance (0.10 m) past the furthest point does not clear it;
// 0.15 m does.
TEST_F(ControllerReplan, ThreeCentimetresAfterAReplanDoNotClearEntrapmentFifteenDo)
{
  begin(5);
  for (int k = 0; k < 10; ++k) {
    step(0.5, 0.0);
  }
  ASSERT_TRUE(controller_->entrapped());
  replanFrom(0.5);
  step(0.5, 0.0);
  step(0.53, 0.0);
  EXPECT_TRUE(controller_->entrapped()) << "0.03 m along +x";
  replanFrom(0.53);
  step(0.53, 0.0);
  step(0.59, 0.0);
  EXPECT_TRUE(controller_->entrapped()) << "0.09 m past the furthest point (0.5)";
  step(0.65, 0.0);
  EXPECT_FALSE(controller_->entrapped()) << "0.15 m past the furthest point (0.5)";
  EXPECT_EQ(controller_->progressEvents(), 1u);
  EXPECT_EQ(controller_->adoptions(), 0u);
}

// ROOT_CAUSE.md section 2.1: the robot reached x = 1.0, backed to 0.6 and
// stalled; one cell of motion after a replan must not clear entrapment.
TEST_F(ControllerReplan, BackedOffRobotStaysEntrappedAfterAReplanAndOneCell)
{
  begin(5);
  for (int k = 0; k <= 20; ++k) {
    step(0.05 * k, 0.0);
  }
  for (int k = 0; k < 10; ++k) {
    step(0.6, 0.0);
  }
  ASSERT_TRUE(controller_->entrapped());
  replanFrom(0.6);
  step(0.6, 0.0);
  step(0.65, 0.0);
  EXPECT_TRUE(controller_->entrapped()) << "x = 0.65 is behind the furthest point 1.0";
  // Past the furthest point by the minimum advance: cleared.
  step(1.12, 0.0);
  EXPECT_FALSE(controller_->entrapped());
}

// (iii) Driving 0.03 m per cycle with a replan every 10 cycles is progress:
// never entrapped, although each replan restarts at the robot.
TEST_F(ControllerReplan, SteadyMotionWithReplansEveryTenCyclesIsNeverEntrapped)
{
  begin(5);
  for (int k = 0; k < 70; ++k) {
    const double x = 0.03 * k;
    if (k % 10 == 0 && k > 0) {replanFrom(x);}
    step(x, 0.0);
    EXPECT_FALSE(controller_->entrapped()) << "cycle " << k;
  }
}

// (iv) A legitimate detour: the replan leaves the original line and the robot
// follows it for 0.6 m. The new path is adopted as the anchor (0.5 m), so no
// entrapment is declared at any cycle (window 30 cycles, 0.03 m per cycle).
TEST_F(ControllerReplan, ADivergingReplanFollowedForSixDecimetresIsNotEntrapment)
{
  begin(30);
  for (int k = 0; k < 3; ++k) {
    step(0.5, 0.0);
  }
  controller_->setPlan(detourPath(0.5));
  for (int k = 0; k <= 20; ++k) {
    step(0.5, 0.03 * k);
    EXPECT_FALSE(controller_->entrapped()) << "cycle " << k;
  }
  EXPECT_EQ(controller_->adoptions(), 1u);
  // The detour keeps going; still progress.
  for (int k = 21; k < 60; ++k) {
    step(0.5, std::min(0.03 * k, 1.5));
    EXPECT_FALSE(controller_->entrapped()) << "cycle " << k;
  }
}

// The adoption distance is 0.5 m by default: 0.45 m along a diverging replan is
// not adopted, 0.55 m is. (Entrapment is declared in between: the stall window
// is shorter than the time the robot needs.)
TEST_F(ControllerReplan, ADivergingReplanIsAdoptedAtHalfAMetreNotBefore)
{
  begin(5);
  for (int k = 0; k < 3; ++k) {
    step(0.5, 0.0);
  }
  controller_->setPlan(detourPath(0.5));
  step(0.5, 0.0);
  step(0.5, 0.45);
  EXPECT_EQ(controller_->adoptions(), 0u) << "0.45 m along the replan";
  step(0.5, 0.55);
  EXPECT_EQ(controller_->adoptions(), 1u) << "0.55 m along the replan";
}

// reset() (a new task from Nav2) forgets the anchor path.
TEST_F(ControllerReplan, ResetForgetsTheAnchorPath)
{
  begin(5);
  EXPECT_TRUE(controller_->hasAnchor());
  controller_->reset();
  EXPECT_FALSE(controller_->hasAnchor());
}

// Both distances are runtime parameters and reach the tracker.
TEST_F(ControllerReplan, ProgressDistancesCanBeSetAtRuntime)
{
  begin(5);
  ASSERT_TRUE(
    node_->set_parameter(rclcpp::Parameter("FollowPath.se_progress_min_advance", 0.3)).successful);
  ASSERT_TRUE(
    node_->set_parameter(
      rclcpp::Parameter("FollowPath.se_progress_adopt_advance", 0.2)).successful);
  for (int k = 0; k < 10; ++k) {
    step(0.5, 0.0);
  }
  ASSERT_TRUE(controller_->entrapped());
  step(0.65, 0.0);
  EXPECT_TRUE(controller_->entrapped()) << "0.15 m is below the new minimum of 0.3 m";
  step(0.85, 0.0);
  EXPECT_FALSE(controller_->entrapped()) << "0.35 m is above it";
  controller_->setPlan(detourPath(0.85));
  step(0.85, 0.0);
  step(0.85, 0.25);
  EXPECT_EQ(controller_->adoptions(), 1u) << "0.25 m is above the new adoption distance 0.2 m";
}

// ---------------------------------------------------------------------------
// Defect 4 (campaign 1, ROOT_CAUSE.md section 4): the escape did not route out
// of the pocket. While entrapped, MPPI is handed a short path from the robot to
// a waypoint along the bearing of the opening chosen by the horizon-ray rule.
// The pocket is drawn from a development geometry (never the live scenarios).
// ---------------------------------------------------------------------------
class ControllerEscapePath : public ControllerBehaviour
{
protected:
  void begin(const std::vector<rclcpp::Parameter> & extra = {}, int stall_window = 5)
  {
    Options opt;
    opt.overrides = {rclcpp::Parameter("FollowPath.se_progress_stall_window", stall_window)};
    opt.overrides.insert(opt.overrides.end(), extra.begin(), extra.end());
    start(opt);
    drawPocket();
    bt_path_ = straightPath("map", -2.6, 3.0, 0.0);
    controller_->setPlan(bt_path_);
  }

  // A pocket of the development family (rotation 0): back face at x = 0.9. The
  // default w13_d18 has its mouth at x = -0.9 and a narrow cone of rays that
  // reach the horizon; the wide w22_d11 has a wide cone.
  double pocket_w_{1.3};
  double pocket_d_{1.8};
  void drawPocket()
  {
    utrap::Pocket p;
    p.w = pocket_w_;
    p.d = pocket_d_;
    p.theta = 0.0;
    p.ft = 1.0;
    p.fb = 1.0;
    const auto ws = utrap::walls(p);
    auto * cm = costmap_ros_->getCostmap();
    for (unsigned int j = 0; j < cm->getSizeInCellsY(); ++j) {
      for (unsigned int i = 0; i < cm->getSizeInCellsX(); ++i) {
        double wx, wy;
        cm->mapToWorld(i, j, wx, wy);
        const unsigned char c = utrap::cellCost(utrap::surfaceDistance(ws, wx, wy));
        if (c != 0) {
          cm->setCost(i, j, c);
        }
      }
    }
  }

  // The robot sits at the stall pose until entrapment is declared and the
  // escape path has been handed to MPPI (one more cycle).
  void stallUntilEscapePath()
  {
    // Entrapment is declared after the stall window (the sixth call); the cycle
    // after it selects the waypoint. Returns right after the selection.
    for (int k = 0; k < 30 && !controller_->escapeActive(); ++k) {step(kStallX, 0.0);}
    ASSERT_TRUE(controller_->entrapped());
    ASSERT_TRUE(controller_->escapeActive());
  }

  static double bearingOf(const nav_msgs::msg::Path & path)
  {
    const auto & a = path.poses.front().pose.position;
    const auto & b = path.poses.back().pose.position;
    return std::atan2(b.y - a.y, b.x - a.x);
  }
  static double lengthOf(const nav_msgs::msg::Path & path)
  {
    const auto & a = path.poses.front().pose.position;
    const auto & b = path.poses.back().pose.position;
    return std::hypot(b.x - a.x, b.y - a.y);
  }
  static double angleBetween(double a, double b)
  {
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
  }
  static bool samePath(const nav_msgs::msg::Path & a, const nav_msgs::msg::Path & b)
  {
    if (a.poses.size() != b.poses.size() || a.header.frame_id != b.header.frame_id) {
      return false;
    }
    for (std::size_t i = 0; i < a.poses.size(); ++i) {
      if (a.poses[i].pose.position.x != b.poses[i].pose.position.x ||
        a.poses[i].pose.position.y != b.poses[i].pose.position.y)
      {
        return false;
      }
    }
    return true;
  }

  static constexpr double kStallX = 0.35;  // 0.34 m in front of the inflated back wall
  nav_msgs::msg::Path bt_path_;
};

// Path capture: when entrapped in the pocket, the path given to MPPI starts at
// the robot, points within 20 degrees of the mouth bearing (180 degrees), and
// ends (robot radius + 0.05 m) short of the end of the free ray (2.0 m).
TEST_F(ControllerEscapePath, TheEscapePathGivenToMppiPointsAtTheMouth)
{
  begin();
  ASSERT_EQ(controller_->handed.size(), 1u) << "the BT path";
  stallUntilEscapePath();
  ASSERT_EQ(controller_->handed.size(), 2u);
  const auto & path = controller_->handed.back();
  ASSERT_GE(path.poses.size(), 10u);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_NEAR(path.poses.front().pose.position.x, kStallX, 1e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y, 0.0, 1e-9);
  const double bearing = bearingOf(path);
  std::printf(
    "escape path: %zu poses, bearing %.0f deg, length %.2f m\n", path.poses.size(),
    bearing * 180.0 / M_PI, lengthOf(path));
  EXPECT_LE(angleBetween(bearing, M_PI), 20.5 * M_PI / 180.0);
  EXPECT_NEAR(lengthOf(path), 2.0 - controller_->robotRadius() - 0.05, 0.03);
  // Poses every 0.05 m, oriented along the bearing.
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    EXPECT_NEAR(std::hypot(b.x - a.x, b.y - a.y), 0.05, 0.005) << i;
  }
  const auto & q = path.poses.back().pose.orientation;
  EXPECT_NEAR(2.0 * std::atan2(q.z, q.w), bearing, 1e-6);
  EXPECT_EQ(controller_->hops(), 1u);
  EXPECT_EQ(controller_->escapeHop(), 1);
}

// The waypoint is capped at se_escape_waypoint_cap.
TEST_F(ControllerEscapePath, TheWaypointDistanceIsCapped)
{
  begin({rclcpp::Parameter("FollowPath.se_escape_waypoint_cap", 0.8)});
  stallUntilEscapePath();
  EXPECT_NEAR(lengthOf(controller_->handed.back()), 0.8, 0.03);
}

// The BT keeps replanning while the escape path is substituted: the new path
// is kept for progress and the near-goal test but is not given to MPPI.
TEST_F(ControllerEscapePath, ABtReplanDuringTheEscapeIsNotGivenToMppi)
{
  begin();
  stallUntilEscapePath();
  const std::size_t before = controller_->handed.size();
  controller_->setPlan(straightPath("map", kStallX, 3.0, 0.0));
  EXPECT_EQ(controller_->handed.size(), before);
  EXPECT_TRUE(controller_->escapeActive());
  step(kStallX, 0.0);
  EXPECT_EQ(controller_->handed.size(), before) << "no re-hand while the waypoint stands";
}

// The escape ends when progress resumes: the latest BT path goes back to MPPI.
TEST_F(ControllerEscapePath, TheLatestBtPathIsGivenBackWhenTheEscapeEnds)
{
  begin();
  stallUntilEscapePath();
  const auto replan = straightPath("map", kStallX, 3.0, 0.0);
  controller_->setPlan(replan);
  // Enough motion along the anchor path to clear entrapment (0.15 m past the best).
  step(kStallX + 0.15, 0.0);
  ASSERT_FALSE(controller_->entrapped());
  const std::size_t before = controller_->handed.size();
  step(kStallX + 0.15, 0.0);
  ASSERT_EQ(controller_->handed.size(), before + 1);
  EXPECT_TRUE(samePath(controller_->handed.back(), replan));
  EXPECT_FALSE(controller_->escapeActive());
  step(kStallX + 0.15, 0.0);
  EXPECT_EQ(controller_->handed.size(), before + 1) << "given back once";
}

TEST_F(ControllerEscapePath, ResetGivesTheBtPathBack)
{
  begin();
  stallUntilEscapePath();
  const std::size_t before = controller_->handed.size();
  controller_->reset();
  ASSERT_EQ(controller_->handed.size(), before + 1);
  EXPECT_TRUE(samePath(controller_->handed.back(), bt_path_));
  EXPECT_FALSE(controller_->escapeActive());
}

TEST_F(ControllerEscapePath, ANewGoalReplacesTheEscapePathAtOnce)
{
  begin();
  stallUntilEscapePath();
  const std::size_t before = controller_->handed.size();
  const auto other = straightPath("map", kStallX, 2.0, 1.5);  // another goal
  controller_->setPlan(other);
  ASSERT_EQ(controller_->handed.size(), before + 1);
  EXPECT_TRUE(samePath(controller_->handed.back(), other));
  EXPECT_FALSE(controller_->escapeActive());
  EXPECT_FALSE(controller_->entrapped());
}

TEST_F(ControllerEscapePath, SwitchingTheSeLayerOffGivesTheBtPathBack)
{
  begin();
  stallUntilEscapePath();
  const std::size_t before = controller_->handed.size();
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.se_enabled", false)).successful);
  step(kStallX, 0.0);
  ASSERT_EQ(controller_->handed.size(), before + 1);
  EXPECT_TRUE(samePath(controller_->handed.back(), bt_path_));
  EXPECT_FALSE(controller_->escapeActive());
  step(kStallX, 0.0);
  EXPECT_EQ(controller_->handed.size(), before + 1);
}

TEST_F(ControllerEscapePath, SubstitutionCanBeSwitchedOff)
{
  begin({rclcpp::Parameter("FollowPath.se_escape_path_substitution", false)});
  for (int k = 0; k < 20; ++k) {
    step(kStallX, 0.0);
  }
  EXPECT_TRUE(controller_->entrapped());
  EXPECT_EQ(controller_->handed.size(), 1u) << "only the BT path";
  EXPECT_FALSE(controller_->escapeActive());
}

// Near the goal the entrapment is suppressed (the BT path still feeds this test),
// so nothing is substituted.
TEST_F(ControllerEscapePath, NothingIsSubstitutedNearTheGoal)
{
  begin();
  for (int k = 0; k < 20; ++k) {
    step(2.8, 0.0);
  }
  EXPECT_FALSE(controller_->entrapped());
  EXPECT_EQ(controller_->handed.size(), 1u);
}

// Waypoint reached: a new hop, a new path from the new position.
TEST_F(ControllerEscapePath, ReachingTheWaypointSelectsTheNextHop)
{
  begin();
  stallUntilEscapePath();
  const auto first = controller_->handed.back();
  const auto & wp = first.poses.back().pose.position;
  step(wp.x, wp.y);  // the robot is at the waypoint
  EXPECT_EQ(controller_->hops(), 2u);
  EXPECT_EQ(controller_->escapeHop(), 2);
  ASSERT_EQ(controller_->handed.size(), 3u);
  const auto & second = controller_->handed.back();
  EXPECT_NEAR(second.poses.front().pose.position.x, wp.x, 1e-9);
  // The next hop must not lead back to where the previous one started.
  const double back = std::atan2(kStallX - wp.x, 0.0 - wp.y);
  (void)back;
  EXPECT_GT(
    std::hypot(
      second.poses.back().pose.position.x - kStallX,
      second.poses.back().pose.position.y - 0.0), 0.6);
}

// Reselection radius: 0.39 m from the waypoint re-selects, 0.41 m does not.
TEST_F(ControllerEscapePath, TheWaypointRadiusBoundary)
{
  begin();
  stallUntilEscapePath();
  const auto & wp = controller_->handed.back().poses.back().pose.position;
  const auto & p0 = controller_->handed.back().poses.front().pose.position;
  const double len = std::hypot(wp.x - p0.x, wp.y - p0.y);
  const double ux = (wp.x - p0.x) / len;
  const double uy = (wp.y - p0.y) / len;
  step(wp.x - 0.41 * ux, wp.y - 0.41 * uy);
  EXPECT_EQ(controller_->hops(), 1u) << "0.41 m from the waypoint";
  step(wp.x - 0.39 * ux, wp.y - 0.39 * uy);
  EXPECT_EQ(controller_->hops(), 2u) << "0.39 m from the waypoint";
}

// The straight line to the waypoint is blocked: re-select.
TEST_F(ControllerEscapePath, ABlockedWaypointLineSelectsAnotherHop)
{
  begin({rclcpp::Parameter("FollowPath.se_escape_stall_time", 100.0)});
  stallUntilEscapePath();
  ASSERT_EQ(controller_->hops(), 1u);
  const auto & wp = controller_->handed.back().poses.back().pose.position;
  // A lethal block halfway between the robot and the waypoint.
  auto * cm = costmap_ros_->getCostmap();
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(cm->worldToMap(0.5 * (kStallX + wp.x), 0.5 * wp.y, mx, my));
  block(mx - 2, my - 2, 5);
  step(kStallX, 0.0);
  EXPECT_EQ(controller_->hops(), 2u);
}

// The timeout: a waypoint that stands for se_escape_reselect_period is re-selected.
TEST_F(ControllerEscapePath, AStandingWaypointIsReselectedAfterThePeriod)
{
  begin(
    {rclcpp::Parameter("FollowPath.se_escape_stall_time", 100.0),
      rclcpp::Parameter("FollowPath.se_escape_reselect_period", 1.0)});
  stallUntilEscapePath();
  ASSERT_EQ(controller_->hops(), 1u);
  // controller_frequency is 20 Hz: 20 cycles.
  for (int k = 0; k < 19; ++k) {
    step(kStallX, 0.0);
  }
  EXPECT_EQ(controller_->hops(), 1u) << "19 cycles";
  step(kStallX, 0.0);
  EXPECT_EQ(controller_->hops(), 2u) << "20 cycles";
}

// The movement watchdog: a robot that does not move 0.15 m in se_escape_stall_time
// gets another waypoint, and the failed one is not chosen again.
TEST_F(ControllerEscapePath, ARobotThatDoesNotMoveGetsAnotherWaypoint)
{
  pocket_w_ = 2.2;
  pocket_d_ = 1.1;
  begin({rclcpp::Parameter("FollowPath.se_escape_stall_time", 0.5)});
  stallUntilEscapePath();
  ASSERT_EQ(controller_->hops(), 1u);
  const double first_bearing = controller_->escapeBearing();
  const auto first_wp = controller_->handed.back().poses.back().pose.position;
  for (int k = 0; k < 9; ++k) {
    step(kStallX, 0.0);
  }
  EXPECT_EQ(controller_->hops(), 1u) << "9 cycles of 10";
  step(kStallX, 0.0);
  EXPECT_EQ(controller_->hops(), 2u) << "10 cycles at 20 Hz = 0.5 s";
  const auto second_wp = controller_->handed.back().poses.back().pose.position;
  std::printf(
    "watchdog: first bearing %.0f deg (%.2f, %.2f), second %.0f deg (%.2f, %.2f)\n",
    first_bearing * 180.0 / M_PI, first_wp.x, first_wp.y,
    controller_->escapeBearing() * 180.0 / M_PI, second_wp.x, second_wp.y);
  EXPECT_GT(std::hypot(second_wp.x - first_wp.x, second_wp.y - first_wp.y), 0.5)
    << "the failed waypoint is excluded";
}

TEST_F(ControllerEscapePath, MovingRobotIsNotStalled)
{
  begin({rclcpp::Parameter("FollowPath.se_escape_stall_time", 0.5)});
  stallUntilEscapePath();
  double x = kStallX;
  for (int round = 0; round < 4; ++round) {
    for (int k = 0; k < 5; ++k) {
      step(x, 0.0);
    }
    x -= 0.16;  // 0.16 m every 5 cycles (0.25 s)
  }
  EXPECT_EQ(controller_->hops(), 1u);
}

// A second episode starts from a clean memory.
TEST_F(ControllerEscapePath, AnEscapeEpisodeStartsWithHopOne)
{
  begin();
  stallUntilEscapePath();
  const auto wp = controller_->handed.back().poses.back().pose.position;
  step(wp.x, wp.y);
  ASSERT_EQ(controller_->escapeHop(), 2);
  // Progress ends the episode: 0.15 m past the furthest point (x = 0.35).
  controller_->setPlan(straightPath("map", wp.x, 3.0, 0.0));
  step(0.5, 0.0);
  ASSERT_FALSE(controller_->entrapped());
  step(0.5, 0.0);
  ASSERT_FALSE(controller_->escapeActive());
  // Stall again further on: hop 1 of the new episode.
  for (int k = 0; k < 30 && !controller_->escapeActive(); ++k) {
    step(0.5, 0.0);
  }
  EXPECT_TRUE(controller_->escapeActive());
  EXPECT_EQ(controller_->escapeHop(), 1);
}

// The escape path is published for RViz and for the campaign recorder.
TEST_F(ControllerEscapePath, TheEscapePathIsPublishedOnSeEscapePath)
{
  begin();
  auto listener = std::make_shared<rclcpp::Node>("escape_path_listener");
  std::vector<nav_msgs::msg::Path> received;
  auto sub = listener->create_subscription<nav_msgs::msg::Path>(
    "FollowPath/se_escape_path", rclcpp::QoS(10),
    [&received](nav_msgs::msg::Path::ConstSharedPtr msg) {received.push_back(*msg);});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(listener);
  // Discovery.
  for (int k = 0; k < 50; ++k) {
    exec.spin_some(std::chrono::milliseconds(10));
  }
  stallUntilEscapePath();
  for (int k = 0; k < 100 && received.empty(); ++k) {
    exec.spin_some(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(received.empty());
  EXPECT_TRUE(samePath(received.back(), controller_->handed.back()));
  // An empty path clears it when the escape ends.
  controller_->reset();
  for (int k = 0; k < 100 && received.back().poses.size() != 0u; ++k) {
    exec.spin_some(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(received.back().poses.empty());
}

// vx_min > 0 with the SE layer on: the robot cannot stop or reverse, so the
// escape cannot act (campaign 1, set P).
TEST_F(ControllerBehaviour, ForwardOnlyVxMinWithSeEnabledIsFlagged)
{
  Options opt;
  opt.overrides = {rclcpp::Parameter("FollowPath.vx_min", 0.1)};
  start(opt);
  EXPECT_TRUE(controller_->vxMinBlocks());
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.vx_min", -0.35)).successful);
  EXPECT_FALSE(controller_->vxMinBlocks());
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.vx_min", 0.0)).successful);
  EXPECT_FALSE(controller_->vxMinBlocks()) << "0.0 can stop";
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.vx_min", 0.05)).successful);
  EXPECT_TRUE(controller_->vxMinBlocks());
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.se_enabled", false)).successful);
  EXPECT_FALSE(controller_->vxMinBlocks()) << "no SE layer, nothing to block";
}

TEST_F(ControllerBehaviour, ReversibleVxMinIsNotFlagged)
{
  Options opt;
  opt.overrides = {rclcpp::Parameter("FollowPath.vx_min", -0.35)};
  start(opt);
  EXPECT_FALSE(controller_->vxMinBlocks());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
