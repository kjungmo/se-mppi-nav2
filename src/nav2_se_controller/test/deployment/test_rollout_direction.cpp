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

// Deployment test: the direction of MPPI's rollouts and of the command the
// controller returns, and the configure-time guard built on it.
//
// Some nav2_mppi_controller binaries negate cos/sin in the optimizer's batch
// rollouts (README, "Affected nav2_mppi_controller builds"). The optimizer then
// scores every sampled trajectory at a mirrored position and commands motion
// away from the goal. Nothing else in this package's tests looks at the sign of
// a command.
//
// Three groups:
//  - InstalledMppi.* and Headings/CommandDirection.*: what the library loaded
//    by this test does. They FAIL on an affected library.
//  - ClosedForm.*, RolloutComparison.*, RolloutCheck.*, FailureText.*: the
//    arithmetic of the self-check, on made-up data and on models of a correct
//    and of an affected library.
//  - RolloutGuard.*: what the controller does with the self-check's verdict
//    (refuse, run with ERROR, run normally). The verdict is injected, so these
//    do not depend on the library.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging.h"
#include "tf2_ros/buffer.h"

#include "nav2_se_controller/mppi_rollout_check.hpp"
#include "nav2_se_controller/safe_escape_controller.hpp"

namespace
{

using nav2_se_controller::RolloutCheckResult;
using nav2_se_controller::RolloutPoint;
using nav2_se_controller::kRolloutTolerance;

constexpr double kPi = 3.14159265358979323846;
const char kEnforceKey[] = "FollowPath.se_enforce_rollout_check";

// Headings on both sides of the two thresholds of the known defect (it negates
// cos for |yaw| < pi/4 and sin for |yaw| < 3 pi/4, and is exact beyond that),
// in all four quadrants. An affected library passes the rows beyond 3 pi/4:
// they are here so that a library negating everything fails too.
const double kYaws[] = {
  0.0, 0.5, 1.0, kPi / 2.0, 2.0, 2.3, 2.4, 3.0, kPi,
  -0.5, -1.0, -kPi / 2.0, -2.0, -2.4, -3.0};

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

// ---------------------------------------------------------------------------
// ERROR-and-above log messages, counted while a Capture is alive.
// ---------------------------------------------------------------------------
struct ErrorLog
{
  int count{0};
  std::string last;
};
ErrorLog * g_error_log = nullptr;
rcutils_logging_output_handler_t g_previous_handler = nullptr;

void countingHandler(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (g_error_log != nullptr && severity >= RCUTILS_LOG_SEVERITY_ERROR) {
    va_list copy;
    va_copy(copy, *args);
    std::vector<char> buffer(8192);
    std::vsnprintf(buffer.data(), buffer.size(), format, copy);
    va_end(copy);
    ++g_error_log->count;
    g_error_log->last = buffer.data();
  }
  if (g_previous_handler != nullptr) {
    g_previous_handler(location, severity, name, timestamp, format, args);
  }
}

class ErrorCapture
{
public:
  ErrorCapture()
  {
    g_error_log = &log_;
    g_previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(countingHandler);
  }
  ~ErrorCapture()
  {
    rcutils_logging_set_output_handler(g_previous_handler);
    g_error_log = nullptr;
  }
  const ErrorLog & log() const {return log_;}

private:
  ErrorLog log_;
};

// ---------------------------------------------------------------------------
// The controller with the self-check's verdict injectable.
// ---------------------------------------------------------------------------
class StubbedController : public nav2_se_controller::SafeEscapeController
{
public:
  std::optional<RolloutCheckResult> verdict;  // unset: ask the loaded library
  mutable int checks{0};

  void emitDiagnostics() {publishDiagnostics();}

protected:
  RolloutCheckResult runRolloutCheck() const override
  {
    ++checks;
    return verdict ? *verdict : SafeEscapeController::runRolloutCheck();
  }
};

RolloutCheckResult passingVerdict()
{
  RolloutCheckResult r;
  r.ok = true;
  r.points = 24576;
  r.straight_end_expected = 0.96;
  r.straight_end_library = 0.96;
  r.library_path = "/stub/libmppi_controller.so";
  return r;
}

// The verdict the RoboStack Humble 1.1.20 _18 binary gets.
RolloutCheckResult failingVerdict()
{
  RolloutCheckResult r;
  r.ok = false;
  r.points = 24576;
  r.wrong_points = 20992;
  r.worst_error = 1.68;
  r.straight_end_expected = 0.96;
  r.straight_end_library = -0.72;
  r.library_path = "/stub/libmppi_controller.so";
  return r;
}

// A lifecycle node, an empty 6 m x 6 m costmap around the origin and a TF
// buffer: what controller_server hands to a controller plugin.
struct Rig
{
  explicit Rig(std::vector<rclcpp::Parameter> overrides = {})
  {
    std::vector<rclcpp::Parameter> all = {
      {"controller_frequency", 20.0},
      {"FollowPath.batch_size", 1000},
      {"FollowPath.time_steps", 56},
      {"FollowPath.model_dt", 0.05},
      {"FollowPath.iteration_count", 1},
      {"FollowPath.vx_max", 0.5},
      {"FollowPath.vx_min", -0.35},
      {"FollowPath.wz_max", 1.9},
      {"FollowPath.motion_model", std::string("DiffDrive")},
      // Distance to the goal only: no orientation or path-direction term.
      {"FollowPath.critics", std::vector<std::string>{"GoalCritic"}},
      {"FollowPath.GoalCritic.threshold_to_consider", 100.0},
      // The robot never moves in these tests; keep the escape mode out of it.
      {"FollowPath.se_progress_stall_window", 100000},
      {"FollowPath.se_viz", false},
    };
    all.insert(all.end(), overrides.begin(), overrides.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(all);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
    costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_ros->on_configure(rclcpp_lifecycle::State{});
    *(costmap_ros->getCostmap()) = nav2_costmap_2d::Costmap2D(120, 120, 0.05, -3.0, -3.0, 0);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  }

  void configure(nav2_se_controller::SafeEscapeController & controller)
  {
    controller.configure(node, "FollowPath", tf, costmap_ros);
  }

  // A straight plan of `distance` from the origin in direction `direction`.
  nav_msgs::msg::Path straightPlan(double direction, double distance) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = costmap_ros->getGlobalFrameID();
    path.header.stamp = node->now();
    for (int i = 0; i <= 20; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = distance * i / 20.0 * std::cos(direction);
      p.pose.position.y = distance * i / 20.0 * std::sin(direction);
      p.pose.orientation = quaternionFromYaw(direction);
      path.poses.push_back(p);
    }
    return path;
  }

  geometry_msgs::msg::TwistStamped step(
    nav2_se_controller::SafeEscapeController & controller, double yaw) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = costmap_ros->getGlobalFrameID();
    pose.header.stamp = node->now();
    pose.pose.orientation = quaternionFromYaw(yaw);
    return controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), nullptr);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

// The controller's /diagnostics status, or nothing after 5 s.
std::optional<diagnostic_msgs::msg::DiagnosticStatus> diagnosticsOf(StubbedController & controller)
{
  auto listener = std::make_shared<rclcpp::Node>("rollout_diag_listener");
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr got;
  auto sub = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10),
    [&got](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {got = msg;});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!got && std::chrono::steady_clock::now() < deadline) {
    controller.emitDiagnostics();
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!got || got->status.size() != 1u) {
    return std::nullopt;
  }
  return got->status[0];
}

std::string valueOf(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return "<no such key>";
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

// MPPI's update rule summed analytically (not step by step as the code under
// test does): position after step t of a constant (vx, wz) from heading yaw0.
RolloutPoint analyticPoint(double yaw0, double vx, double wz, unsigned int t, double dt)
{
  const double n = static_cast<double>(t) + 1.0;
  const double theta = wz * dt;
  if (theta == 0.0) {
    return RolloutPoint{vx * dt * n * std::cos(yaw0), vx * dt * n * std::sin(yaw0)};
  }
  // sum_{k=0..t} cos(yaw0 + k theta) = sin(n theta / 2) / sin(theta / 2) * cos(yaw0 + t theta / 2)
  const double scale = vx * dt * std::sin(n * theta / 2.0) / std::sin(theta / 2.0);
  const double mid = yaw0 + static_cast<double>(t) * theta / 2.0;
  return RolloutPoint{scale * std::cos(mid), scale * std::sin(mid)};
}

}  // namespace

// ===========================================================================
// The arithmetic of the self-check, on made-up data.
// ===========================================================================

TEST(ClosedForm, StraightAndTurningStepsFollowMppisUpdateRule)
{
  const auto east = nav2_se_controller::closedFormRollout(0.0, 0.3, 0.0, 3, 0.05);
  ASSERT_EQ(east.size(), 3u);
  EXPECT_NEAR(east[0].x, 0.015, 1e-12);
  EXPECT_NEAR(east[2].x, 0.045, 1e-12);
  EXPECT_NEAR(east[2].y, 0.0, 1e-12);

  const auto north = nav2_se_controller::closedFormRollout(kPi / 2.0, 0.3, 0.0, 2, 0.05);
  EXPECT_NEAR(north[1].x, 0.0, 1e-12);
  EXPECT_NEAR(north[1].y, 0.030, 1e-12);

  const auto back = nav2_se_controller::closedFormRollout(0.0, -0.2, 0.0, 2, 0.1);
  EXPECT_NEAR(back[1].x, -0.040, 1e-12);

  // The first step uses the start heading; the turn shows from the second on.
  const auto turn = nav2_se_controller::closedFormRollout(0.0, 1.0, kPi / 2.0, 2, 1.0);
  EXPECT_NEAR(turn[0].x, 1.0, 1e-12);
  EXPECT_NEAR(turn[0].y, 0.0, 1e-12);
  EXPECT_NEAR(turn[1].x, 1.0, 1e-12);
  EXPECT_NEAR(turn[1].y, 1.0, 1e-12);

  EXPECT_TRUE(nav2_se_controller::closedFormRollout(0.0, 0.3, 0.0, 0, 0.05).empty());
}

TEST(ClosedForm, AgreesWithTheAnalyticSum)
{
  for (const double yaw : kYaws) {
    for (const double wz : {0.0, 1.0, -1.0, 1.9634954084936207}) {
      const auto steps = nav2_se_controller::closedFormRollout(yaw, 0.3, wz, 64, 0.05);
      for (unsigned int t = 0; t < 64; ++t) {
        const RolloutPoint want = analyticPoint(yaw, 0.3, wz, t, 0.05);
        ASSERT_NEAR(steps[t].x, want.x, 1e-9) << "yaw " << yaw << " wz " << wz << " t " << t;
        ASSERT_NEAR(steps[t].y, want.y, 1e-9) << "yaw " << yaw << " wz " << wz << " t " << t;
      }
    }
  }
}

TEST(RolloutComparison, CountsPointsBeyondTheToleranceAndNonFiniteOnes)
{
  const std::vector<RolloutPoint> expected(4, RolloutPoint{0.0, 0.0});
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  RolloutCheckResult exact;
  nav2_se_controller::accumulateRolloutComparison(
    std::vector<RolloutPoint>(8, RolloutPoint{0.0, 0.0}), expected, 2, exact);
  EXPECT_EQ(exact.points, 8u);
  EXPECT_EQ(exact.wrong_points, 0u);
  EXPECT_DOUBLE_EQ(exact.worst_error, 0.0);

  // Exactly at the tolerance is still right; just beyond it is wrong.
  RolloutCheckResult edge;
  nav2_se_controller::accumulateRolloutComparison(
    {RolloutPoint{kRolloutTolerance, 0.0}, RolloutPoint{0.0, -kRolloutTolerance},
      RolloutPoint{kRolloutTolerance * 1.001, 0.0}, RolloutPoint{0.0, -kRolloutTolerance * 1.001}},
    expected, 1, edge);
  EXPECT_EQ(edge.points, 4u);
  EXPECT_EQ(edge.wrong_points, 2u);
  EXPECT_DOUBLE_EQ(edge.worst_error, kRolloutTolerance * 1.001);

  RolloutCheckResult not_finite;
  nav2_se_controller::accumulateRolloutComparison(
    {RolloutPoint{nan, 0.0}, RolloutPoint{0.0, nan}, RolloutPoint{inf, 0.0},
      RolloutPoint{0.0, -inf}},
    expected, 1, not_finite);
  EXPECT_EQ(not_finite.wrong_points, 4u);
  EXPECT_DOUBLE_EQ(not_finite.worst_error, 0.0);  // the worst FINITE deviation

  // Results add up across calls.
  nav2_se_controller::accumulateRolloutComparison(
    {RolloutPoint{0.5, 0.0}, RolloutPoint{}, RolloutPoint{}, RolloutPoint{}},
    expected, 1, not_finite);
  EXPECT_EQ(not_finite.points, 8u);
  EXPECT_EQ(not_finite.wrong_points, 5u);
  EXPECT_DOUBLE_EQ(not_finite.worst_error, 0.5);
}

TEST(RolloutComparison, WrongSizeCountsEveryPointAsWrong)
{
  const std::vector<RolloutPoint> expected(4, RolloutPoint{0.0, 0.0});
  for (const std::size_t size : {std::size_t{0}, std::size_t{7}, std::size_t{9}}) {
    RolloutCheckResult r;
    nav2_se_controller::accumulateRolloutComparison(
      std::vector<RolloutPoint>(size, RolloutPoint{0.0, 0.0}), expected, 2, r);
    EXPECT_EQ(r.points, 8u) << size;
    EXPECT_EQ(r.wrong_points, 8u) << size;
  }
  RolloutCheckResult nothing;
  nav2_se_controller::accumulateRolloutComparison({}, {}, 3, nothing);
  EXPECT_EQ(nothing.points, 0u);
  EXPECT_EQ(nothing.wrong_points, 0u);
}

TEST(RolloutComparison, TheKnownDefectsMirroredRolloutIsCaught)
{
  // What an affected library returns at heading 0: the first step right, every
  // later step negated.
  const auto expected = nav2_se_controller::closedFormRollout(0.0, 0.3, 0.0, 64, 0.05);
  std::vector<RolloutPoint> mirrored(expected.size());
  double x = 0.0;
  for (std::size_t t = 0; t < expected.size(); ++t) {
    x += (t == 0 ? 1.0 : -1.0) * 0.3 * 0.05;
    mirrored[t] = RolloutPoint{x, 0.0};
  }
  RolloutCheckResult r;
  nav2_se_controller::accumulateRolloutComparison(mirrored, expected, 1, r);
  EXPECT_EQ(r.points, 64u);
  EXPECT_EQ(r.wrong_points, 63u);  // all but the first step
  EXPECT_NEAR(r.worst_error, 2.0 * 63 * 0.015, 1e-9);
}

// A model of an affected library, from the measured pattern: the first step is
// right; from the second step on cos is negated while the heading the step
// starts with, folded into (-pi, pi], is within pi/4 of 0, and sin while it is
// within 3 pi/4.
std::vector<RolloutPoint> affectedLibraryModel(
  double yaw0, double vx, double wz, unsigned int batch, unsigned int steps, double dt)
{
  std::vector<RolloutPoint> one(steps);
  double x = 0.0;
  double y = 0.0;
  double yaw = yaw0;
  for (unsigned int t = 0; t < steps; ++t) {
    const double folded = std::fabs(std::atan2(std::sin(yaw), std::cos(yaw)));
    const double cos_sign = (t > 0 && folded < kPi / 4.0) ? -1.0 : 1.0;
    const double sin_sign = (t > 0 && folded < 3.0 * kPi / 4.0) ? -1.0 : 1.0;
    x += cos_sign * vx * std::cos(yaw) * dt;
    y += sin_sign * vx * std::sin(yaw) * dt;
    yaw += wz * dt;
    one[t] = RolloutPoint{x, y};
  }
  std::vector<RolloutPoint> all;
  for (unsigned int b = 0; b < batch; ++b) {
    all.insert(all.end(), one.begin(), one.end());
  }
  return all;
}

std::vector<RolloutPoint> correctLibraryModel(
  double yaw0, double vx, double wz, unsigned int batch, unsigned int steps, double dt)
{
  const auto one = nav2_se_controller::closedFormRollout(yaw0, vx, wz, steps, dt);
  std::vector<RolloutPoint> all;
  for (unsigned int b = 0; b < batch; ++b) {
    all.insert(all.end(), one.begin(), one.end());
  }
  return all;
}

TEST(RolloutCheck, PassesAModelOfACorrectLibrary)
{
  const RolloutCheckResult r = nav2_se_controller::checkRolloutDirection(&correctLibraryModel);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.points, 24576u);  // 16 headings x 3 turn rates x 8 samples x 64 steps
  EXPECT_EQ(r.wrong_points, 0u);
  EXPECT_DOUBLE_EQ(r.worst_error, 0.0);
  EXPECT_NEAR(r.straight_end_expected, 0.3 * 0.05 * 64, 1e-12);
  EXPECT_DOUBLE_EQ(r.straight_end_library, r.straight_end_expected);
  EXPECT_TRUE(r.library_path.empty());
}

TEST(RolloutCheck, FailsAModelOfAnAffectedLibrary)
{
  const RolloutCheckResult r = nav2_se_controller::checkRolloutDirection(&affectedLibraryModel);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.points, 24576u);
  // The measured binaries (Jazzy 1.3.11 _16, Humble 1.1.20 _18) give 20992:
  // the model treats headings that sit exactly on a threshold differently.
  EXPECT_EQ(r.wrong_points, 21096u);
  EXPECT_NEAR(r.straight_end_expected, 0.96, 1e-12);
  EXPECT_NEAR(r.straight_end_library, 0.015 - 63 * 0.015, 1e-9);  // -0.93 m: backwards
  EXPECT_NEAR(r.worst_error, 0.96 + 0.93, 1e-9);
}

TEST(RolloutCheck, FailsOnASingleWrongPointNotFiniteOutputOrAWrongSize)
{
  // One point of one rollout off by 2 mm.
  int call = 0;
  const RolloutCheckResult one_off = nav2_se_controller::checkRolloutDirection(
    [&call](double yaw0, double vx, double wz, unsigned int batch, unsigned int steps,
    double dt) {
      auto all = correctLibraryModel(yaw0, vx, wz, batch, steps, dt);
      if (call++ == 17) {
        all[100].y += 0.002;
      }
      return all;
    });
  EXPECT_EQ(call, 48);
  EXPECT_FALSE(one_off.ok);
  EXPECT_EQ(one_off.wrong_points, 1u);
  EXPECT_NEAR(one_off.worst_error, 0.002, 1e-12);

  const RolloutCheckResult not_finite = nav2_se_controller::checkRolloutDirection(
    [](double yaw0, double vx, double wz, unsigned int batch, unsigned int steps, double dt) {
      auto all = correctLibraryModel(yaw0, vx, wz, batch, steps, dt);
      all.back().x = std::numeric_limits<double>::quiet_NaN();
      return all;
    });
  EXPECT_FALSE(not_finite.ok);
  EXPECT_EQ(not_finite.wrong_points, 48u);

  const RolloutCheckResult empty = nav2_se_controller::checkRolloutDirection(
    [](double, double, double, unsigned int, unsigned int, double) {
      return std::vector<RolloutPoint>();
    });
  EXPECT_FALSE(empty.ok);
  EXPECT_EQ(empty.points, 24576u);
  EXPECT_EQ(empty.wrong_points, 24576u);
  EXPECT_TRUE(std::isnan(empty.straight_end_library));
}

TEST(FailureText, NamesTheMeasurementTheCauseTheBuildsAndTheRemedy)
{
  const std::string text = nav2_se_controller::describeRolloutFailure(failingVerdict());
  const std::vector<std::string> parts = {
    "/stub/libmppi_controller.so",
    "ends at x = -0.720 m (expected +0.960 m)",
    "20992 of 24576 rollout points are more than 1e-03 m",
    "worst finite deviation 1.680 m",
    "-ffast-math",
    "ros-humble-nav2-mppi-controller 1.1.20 (builds _18 and _20)",
    "ros-jazzy-nav2-mppi-controller 1.3.11 (build _16)",
    "https://github.com/RoboStack/ros-jazzy/issues/290",
    "1.3.13 build _25",
    "scripts/build_mppi_overlay.sh"};
  for (const std::string & part : parts) {
    EXPECT_TRUE(contains(text, part)) << "missing: " << part << "\nin: " << text;
  }
  RolloutCheckResult no_path = failingVerdict();
  no_path.library_path.clear();
  EXPECT_TRUE(contains(nav2_se_controller::describeRolloutFailure(no_path), "(path not found)"));
}

// ===========================================================================
// What the library loaded by this test does. These fail on an affected one.
// ===========================================================================

TEST(InstalledMppi, BatchRolloutMatchesTheClosedFormAtEveryHeading)
{
  const unsigned int batch = 8;
  const unsigned int steps = 64;
  const double dt = 0.05;
  std::size_t wrong = 0;
  std::size_t total = 0;
  std::string first;
  for (const double yaw : kYaws) {
    for (const double vx : {0.3, -0.2}) {
      for (const double wz : {0.0, 1.0, -1.0}) {
        const auto got = nav2_se_controller::mppiBatchRollout(yaw, vx, wz, batch, steps, dt);
        ASSERT_EQ(got.size(), static_cast<std::size_t>(batch) * steps);
        for (unsigned int b = 0; b < batch; ++b) {
          for (unsigned int t = 0; t < steps; ++t) {
            const RolloutPoint & p = got[static_cast<std::size_t>(b) * steps + t];
            const RolloutPoint want = analyticPoint(yaw, vx, wz, t, dt);
            const double error = std::hypot(p.x - want.x, p.y - want.y);
            ++total;
            if (!(error <= kRolloutTolerance)) {
              if (wrong++ == 0) {
                first = "yaw " + std::to_string(yaw) + " vx " + std::to_string(vx) + " wz " +
                  std::to_string(wz) + " sample " + std::to_string(b) + " step " +
                  std::to_string(t) + ": library (" + std::to_string(p.x) + ", " +
                  std::to_string(p.y) + "), closed form (" + std::to_string(want.x) + ", " +
                  std::to_string(want.y) + ")";
              }
            }
          }
        }
      }
    }
  }
  EXPECT_EQ(total, 46080u);
  EXPECT_EQ(wrong, 0u) << wrong << " of " << total <<
    " rollout points are off the closed form; first: " << first;
}

TEST(InstalledMppi, StraightRolloutEndsAheadOfTheRobot)
{
  for (const double yaw : kYaws) {
    const auto got = nav2_se_controller::mppiBatchRollout(yaw, 0.3, 0.0, 8, 64, 0.05);
    ASSERT_EQ(got.size(), 512u);
    for (unsigned int b = 0; b < 8; ++b) {
      const RolloutPoint & end = got[b * 64 + 63];
      const double ahead = end.x * std::cos(yaw) + end.y * std::sin(yaw);
      const double sideways = -end.x * std::sin(yaw) + end.y * std::cos(yaw);
      EXPECT_NEAR(ahead, 0.3 * 0.05 * 64, 1e-3) << "yaw " << yaw << " sample " << b;
      EXPECT_NEAR(sideways, 0.0, 1e-3) << "yaw " << yaw << " sample " << b;
    }
  }
}

TEST(InstalledMppi, EdgeSizes)
{
  EXPECT_TRUE(nav2_se_controller::mppiBatchRollout(0.0, 0.3, 0.0, 0, 64, 0.05).empty());
  EXPECT_TRUE(nav2_se_controller::mppiBatchRollout(0.0, 0.3, 0.0, 8, 0, 0.05).empty());
  // One step is the scalar first step of the library, right on every build.
  const auto one = nav2_se_controller::mppiBatchRollout(1.0, 0.3, 0.0, 1, 1, 0.05);
  ASSERT_EQ(one.size(), 1u);
  EXPECT_NEAR(one[0].x, 0.015 * std::cos(1.0), 1e-6);
  EXPECT_NEAR(one[0].y, 0.015 * std::sin(1.0), 1e-6);
}

TEST(InstalledMppi, SelfCheckPasses)
{
  const RolloutCheckResult r = nav2_se_controller::checkMppiRolloutDirection();
  EXPECT_TRUE(r.ok) << nav2_se_controller::describeRolloutFailure(r);
  EXPECT_EQ(r.points, 24576u);  // 16 headings x 3 turn rates x 8 samples x 64 steps
  EXPECT_EQ(r.wrong_points, 0u);
  EXPECT_LT(r.worst_error, kRolloutTolerance);
  EXPECT_NEAR(r.straight_end_expected, 0.96, 1e-9);
  EXPECT_NEAR(r.straight_end_library, 0.96, 1e-3);
  EXPECT_TRUE(contains(r.library_path, "libmppi_controller")) << r.library_path;
  // Stateless: a second call gives the same verdict.
  const RolloutCheckResult again = nav2_se_controller::checkMppiRolloutDirection();
  EXPECT_EQ(again.ok, r.ok);
  EXPECT_EQ(again.wrong_points, r.wrong_points);
  EXPECT_DOUBLE_EQ(again.worst_error, r.worst_error);
}

TEST(InstalledMppi, ControllerConfiguresWithTheDefaultParameters)
{
  Rig rig;
  nav2_se_controller::SafeEscapeController controller;
  ErrorCapture errors;
  EXPECT_NO_THROW(rig.configure(controller));
  EXPECT_EQ(errors.log().count, 0) << errors.log().last;
  controller.cleanup();
}

// Mean command over the last 10 of 30 cycles for a robot held at the origin
// with heading `yaw` and a straight 1 m plan to a goal at `goal_bearing` (rad,
// relative to the heading: 0 ahead, pi behind). The guard is switched off so
// that an affected library shows its symptom here instead of a refusal.
class CommandDirection : public ::testing::TestWithParam<double>
{
protected:
  static constexpr double kGoalDistance = 1.0;  // m
  static constexpr int kCycles = 30;
  static constexpr int kAveraged = 10;
  // A correct library commands about 0.15 m/s towards the goal in this set-up
  // (0.13 to 0.18 over these headings), an affected one up to 0.35 m/s away.
  static constexpr double kMinSpeed = 0.05;  // m/s

  double meanVx(double yaw, double goal_bearing)
  {
    Rig rig({{kEnforceKey, false}});
    nav2_se_controller::SafeEscapeController controller;
    rig.configure(controller);
    controller.activate();
    controller.setPlan(rig.straightPlan(yaw + goal_bearing, kGoalDistance));
    double mean = 0.0;
    for (int k = 0; k < kCycles; ++k) {
      const auto cmd = rig.step(controller, yaw);
      if (k >= kCycles - kAveraged) {
        mean += cmd.twist.linear.x / kAveraged;
      }
    }
    controller.deactivate();
    controller.cleanup();
    return mean;
  }
};

TEST_P(CommandDirection, GoalAheadIsApproachedForwards)
{
  const double vx = meanVx(GetParam(), 0.0);
  EXPECT_GT(vx, kMinSpeed) <<
    "yaw " << GetParam() << " rad, goal 1 m straight ahead: commanded vx = " << vx <<
    " m/s. A command away from the goal is what a nav2_mppi_controller binary with negated "
    "rollout trigonometry gives (README, \"Affected nav2_mppi_controller builds\").";
}

TEST_P(CommandDirection, GoalBehindIsApproachedBackwards)
{
  const double vx = meanVx(GetParam(), kPi);
  EXPECT_LT(vx, -kMinSpeed) <<
    "yaw " << GetParam() << " rad, goal 1 m straight behind: commanded vx = " << vx <<
    " m/s. A command away from the goal is what a nav2_mppi_controller binary with negated "
    "rollout trigonometry gives (README, \"Affected nav2_mppi_controller builds\").";
}

INSTANTIATE_TEST_SUITE_P(Headings, CommandDirection, ::testing::ValuesIn(kYaws));

// ===========================================================================
// The guard: what the controller does with the verdict (injected).
// ===========================================================================

TEST(RolloutGuard, RefusesToConfigureByDefaultAndLeavesNothingBehind)
{
  Rig rig;
  StubbedController controller;
  controller.verdict = failingVerdict();
  {
    ErrorCapture errors;
    try {
      rig.configure(controller);
      FAIL() << "configure() accepted a library that fails the rollout check";
    } catch (const std::runtime_error & e) {
      EXPECT_TRUE(contains(e.what(), "rollout direction check")) << e.what();
      EXPECT_TRUE(contains(e.what(), "FollowPath.se_enforce_rollout_check: false")) << e.what();
    }
    // One ERROR, and it carries the cause, the builds, the remedy and the override.
    ASSERT_EQ(errors.log().count, 1);
    const std::vector<std::string> parts = {
      "SafeEscapeController[FollowPath] refuses to configure",
      "/stub/libmppi_controller.so", "ends at x = -0.720 m (expected +0.960 m)",
      "-ffast-math", "1.1.20 (builds _18 and _20)", "1.3.11 (build _16)",
      "1.3.13 build _25", "scripts/build_mppi_overlay.sh",
      "set FollowPath.se_enforce_rollout_check: false"};
    for (const std::string & part : parts) {
      EXPECT_TRUE(contains(errors.log().last, part)) << "missing: " << part;
    }
  }
  EXPECT_EQ(controller.checks, 1);
  // Nothing of MPPI or of the SE layer was created.
  EXPECT_TRUE(rig.node->has_parameter(kEnforceKey));
  EXPECT_TRUE(rig.node->get_parameter(kEnforceKey).as_bool());
  EXPECT_FALSE(rig.node->has_parameter("FollowPath.motion_model"));
  EXPECT_FALSE(rig.node->has_parameter("FollowPath.se_enabled"));
  EXPECT_FALSE(rig.node->has_parameter("FollowPath.se_viz"));

  // The same instance configures and runs once the library is right.
  controller.verdict = passingVerdict();
  {
    ErrorCapture errors;
    ASSERT_NO_THROW(rig.configure(controller));
    EXPECT_EQ(errors.log().count, 0) << errors.log().last;
  }
  EXPECT_EQ(controller.checks, 2);
  controller.activate();
  controller.setPlan(rig.straightPlan(0.0, 1.0));
  EXPECT_TRUE(std::isfinite(rig.step(controller, 0.0).twist.linear.x));
  const auto status = diagnosticsOf(controller);
  ASSERT_TRUE(status);
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status->message, "ok");
  EXPECT_EQ(valueOf(*status, "mppi_rollout_check"), "ok");
  EXPECT_EQ(valueOf(*status, "mppi_rollout_detail"), "<no such key>");
  controller.deactivate();
  controller.cleanup();
}

TEST(RolloutGuard, OverrideRunsAndReportsErrorOnDiagnostics)
{
  Rig rig({{kEnforceKey, false}});
  StubbedController controller;
  controller.verdict = failingVerdict();
  {
    ErrorCapture errors;
    ASSERT_NO_THROW(rig.configure(controller));
    ASSERT_EQ(errors.log().count, 1);
    const std::vector<std::string> parts = {
      "SafeEscapeController[FollowPath] runs although", "-ffast-math",
      "FollowPath.se_enforce_rollout_check is false", "/diagnostics reports ERROR"};
    for (const std::string & part : parts) {
      EXPECT_TRUE(contains(errors.log().last, part)) << "missing: " << part;
    }
  }
  controller.activate();
  controller.setPlan(rig.straightPlan(0.0, 1.0));
  EXPECT_TRUE(std::isfinite(rig.step(controller, 0.0).twist.linear.x));

  auto status = diagnosticsOf(controller);
  ASSERT_TRUE(status);
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status->message,
    "nav2_mppi_controller failed the rollout direction check "
    "(se_enforce_rollout_check=false): commands can point away from the goal");
  EXPECT_EQ(valueOf(*status, "mppi_rollout_check"), "failed");
  EXPECT_EQ(
    valueOf(*status, "mppi_rollout_detail"),
    nav2_se_controller::describeRolloutFailure(failingVerdict()));
  EXPECT_NE(valueOf(*status, "cycles"), "0");  // the usual counters are still there

  // It outranks the "SE layer disabled" status: the MPPI command itself is wrong.
  const rclcpp::Parameter se_off("FollowPath.se_enabled", false);
  ASSERT_TRUE(rig.node->set_parameter(se_off).successful);
  status = diagnosticsOf(controller);
  ASSERT_TRUE(status);
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  // Read at configure only: a runtime set is rejected with a reason, value kept.
  for (const bool value : {true, false}) {
    const auto r = rig.node->set_parameter(rclcpp::Parameter(kEnforceKey, value));
    EXPECT_FALSE(r.successful);
    EXPECT_TRUE(contains(r.reason, "configure")) << r.reason;
  }
  EXPECT_FALSE(rig.node->get_parameter(kEnforceKey).as_bool());
  controller.deactivate();
  controller.cleanup();
}

TEST(RolloutGuard, EveryConfigureChecksAgainAndNoVerdictSurvivesCleanup)
{
  Rig rig({{kEnforceKey, false}});
  StubbedController controller;

  // failed (override) -> cleanup -> passed: no ERROR left on /diagnostics.
  controller.verdict = failingVerdict();
  rig.configure(controller);
  controller.activate();
  auto status = diagnosticsOf(controller);
  ASSERT_TRUE(status);
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  controller.deactivate();
  controller.cleanup();

  controller.verdict = passingVerdict();
  {
    ErrorCapture errors;
    rig.configure(controller);
    EXPECT_EQ(errors.log().count, 0) << errors.log().last;
  }
  controller.activate();
  status = diagnosticsOf(controller);
  ASSERT_TRUE(status);
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(valueOf(*status, "mppi_rollout_check"), "ok");
  EXPECT_EQ(valueOf(*status, "mppi_rollout_detail"), "<no such key>");
  controller.deactivate();
  controller.cleanup();

  // passed -> cleanup -> failed again: reported again.
  controller.verdict = failingVerdict();
  {
    ErrorCapture errors;
    rig.configure(controller);
    EXPECT_EQ(errors.log().count, 1);
  }
  controller.activate();
  status = diagnosticsOf(controller);
  ASSERT_TRUE(status);
  EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  controller.deactivate();
  controller.cleanup();
  EXPECT_EQ(controller.checks, 3);
}

TEST(RolloutGuard, PassingLibraryIsUntouchedByTheOverride)
{
  // On a library that passes, se_enforce_rollout_check makes no difference.
  for (const bool enforce : {true, false}) {
    Rig rig({{kEnforceKey, enforce}});
    StubbedController controller;
    controller.verdict = passingVerdict();
    ErrorCapture errors;
    ASSERT_NO_THROW(rig.configure(controller)) << enforce;
    EXPECT_EQ(errors.log().count, 0) << errors.log().last;
    EXPECT_EQ(controller.checks, 1);
    EXPECT_EQ(rig.node->get_parameter(kEnforceKey).as_bool(), enforce);
    controller.activate();
    const auto status = diagnosticsOf(controller);
    ASSERT_TRUE(status);
    EXPECT_EQ(status->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << enforce;
    controller.deactivate();
    controller.cleanup();
  }
}

TEST(RolloutGuard, AWronglyTypedOverrideIsNotReadAsFalse)
{
  // A YAML typo such as se_enforce_rollout_check: "false" (a string) must not
  // switch the guard off: rclcpp rejects the type, configure fails.
  Rig rig({{kEnforceKey, std::string("false")}});
  StubbedController controller;
  controller.verdict = failingVerdict();
  EXPECT_THROW(rig.configure(controller), std::exception);
  EXPECT_FALSE(rig.node->has_parameter("FollowPath.motion_model"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
