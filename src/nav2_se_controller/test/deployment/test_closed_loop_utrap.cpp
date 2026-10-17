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

// Headless closed-loop U-pocket test. The real MPPI optimizer and the
// SafeEscapeController drive a unicycle into a pocket that is absent from the
// planner's map; a stand-in for the behaviour tree replans a straight line to
// the goal every second; controller_server's reaction to NoValidControl (zero
// velocity) is emulated. Geometries come from the written rule in
// UTRAP_DEV_GEOMETRIES.md. The live scenarios are not used here.

#include <gtest/gtest.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "rcutils/logging.h"
#include "rclcpp/rclcpp.hpp"

#include "nav2_se_controller/safe_escape_controller.hpp"
#include "utrap_dev_geometries.hpp"
#include "utrap_rig.hpp"

namespace
{

class ClosedLoopProbe : public nav2_se_controller::SafeEscapeController
{
public:
  int enters() const {return static_cast<int>(counters_.escape_entries.load());}
  int hops() const {return static_cast<int>(counters_.escape_hops.load());}
};

void report(const utrap::RunResult & r, int run)
{
  std::printf(
    "UTRAP geometry=%s run=%d enter_t=%.1f exit_t=%.1f goal_t=%.1f reached=%d contacts=%d "
    "throws=%d "
    "enters=%d hops=%d min_goal_dist=%.2f sim_t=%.1f\n",
    r.geometry.c_str(), run, r.enter_time, r.exit_time, r.goal_time, r.reached ? 1 : 0, r.contacts,
    r.throws, r.enters, r.hops, r.min_goal_dist, r.sim_time);
  std::fflush(stdout);
}

// Pass rule of UTRAP_DEV_GEOMETRIES.md for one run.
::testing::AssertionResult passes(const utrap::RunResult & r)
{
  if (!r.entered_pocket) {
    return ::testing::AssertionFailure()
           << r.geometry << ": the robot never entered the pocket (the test needs a trap)";
  }
  if (!r.left_pocket || r.exit_time > 60.0) {
    return ::testing::AssertionFailure()
           << r.geometry << ": the robot centre did not leave the pocket within 60 s";
  }
  if (!r.reached || r.goal_time > 150.0) {
    return ::testing::AssertionFailure()
           << r.geometry << ": the goal was not reached within 150 s (closest "
           << r.min_goal_dist << " m)";
  }
  if (r.contacts != 0) {
    return ::testing::AssertionFailure()
           << r.geometry << ": " << r.contacts << " wall contacts";
  }
  // The campaign runs with failure_tolerance 0.0: controller_server rethrows
  // NoValidControl, computeControl catches it and publishZeroVelocity() calls
  // reset() on every controller, which clears the detector in the same cycle. In C3 and C4
  // the first throw ends the goal. The rig keeps going after a throw (as
  // failure_tolerance -1 would), so a run with a throw would not be a C4 pass.
  if (r.throws != 0) {
    return ::testing::AssertionFailure()
           << r.geometry << ": " << r.throws << " NoValidControl throws; with the campaign's "
           << "failure_tolerance 0.0 the first one ends the goal (C4 would abort)";
  }
  return ::testing::AssertionSuccess();
}

utrap::RunResult runOnce(const utrap::Pocket & p)
{
  static int counter = 0;
  utrap::Rig<ClosedLoopProbe> rig;
  auto r = rig.run(p);
  // SE_UTRAP_TRACE_DIR: write the path of every run as CSV (t, x, y).
  if (const char * dir = std::getenv("SE_UTRAP_TRACE_DIR")) {
    const std::string file =
      std::string(dir) + "/" + r.geometry + "_" + std::to_string(counter++) + ".csv";
    if (FILE * f = std::fopen(file.c_str(), "w")) {
      for (const auto & row : r.trace) {
        std::fprintf(f, "%.1f,%.3f,%.3f\n", row[0], row[1], row[2]);
      }
      std::fclose(f);
    }
  }
  return r;
}

}  // namespace

// The primary geometries are valid members of the rule and differ from the
// live s1 pocket (1.7 m x 1.85 m).
TEST(ClosedLoopUtrap, PrimaryGeometriesFollowTheRule)
{
  for (const auto & p : utrap::primary()) {
    EXPECT_TRUE(utrap::valid(p)) << utrap::name(p);
  }
  EXPECT_EQ(utrap::family().size(), 56u);
}

TEST(ClosedLoopUtrap, PrimaryGeometriesLeaveThePocketAndReachTheGoalThreeOfThree)
{
  for (const auto & p : utrap::primary()) {
    int ok = 0;
    for (int run = 0; run < 3; ++run) {
      const auto r = runOnce(p);
      report(r, run);
      if (passes(r)) {
        ++ok;
      }
    }
    EXPECT_EQ(ok, 3) << utrap::name(p) << ": " << ok << " of 3 runs passed";
  }
}

// Informational: every 7th valid geometry, one run each. Not gating.
TEST(ClosedLoopUtrap, SweepReportsEveryGeometry)
{
  const char * mode = std::getenv("SE_UTRAP_SWEEP");
  if (mode == nullptr) {
    GTEST_SKIP() << "set SE_UTRAP_SWEEP=1 (every 7th geometry) or =all (the whole family)";
  }
  const auto list = std::string(mode) == "all" ? utrap::family() : utrap::sweep();
  int ok = 0;
  int n = 0;
  for (const auto & p : list) {
    const auto r = runOnce(p);
    report(r, 0);
    ok += passes(r) ? 1 : 0;
    ++n;
  }
  std::printf("UTRAP sweep passed %d of %d\n", ok, n);
}

namespace
{
std::vector<std::string> g_log_lines;
rcutils_logging_output_handler_t g_previous_handler = nullptr;

void captureLog(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  va_list copy;
  va_copy(copy, *args);
  char buffer[1024];
  std::vsnprintf(buffer, sizeof(buffer), format, copy);
  va_end(copy);
  g_log_lines.push_back(buffer);
  if (g_previous_handler != nullptr) {
    g_previous_handler(location, severity, name, timestamp, format, args);
  }
}
}  // namespace

// The selected bearing and clearance are logged: one line per hop from the
// controller (this is the escape heading), and a critic-side probe at injection BEGIN
// and every 2 s that is labelled as NOT the escape heading.
TEST(ClosedLoopUtrap, TheSelectedBearingAndClearanceAreLogged)
{
  g_log_lines.clear();
  g_previous_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(captureLog);
  utrap::Pocket p;
  p.w = 1.3;
  p.d = 1.8;  // w13_d18_r0_a10 of the family
  const auto r = runOnce(p);
  rcutils_logging_set_output_handler(g_previous_handler);
  EXPECT_TRUE(r.reached);
  int waypoint_lines = 0;
  int critic_lines = 0;
  for (const auto & line : g_log_lines) {
    if (line.find("SE escape waypoint ") != std::string::npos &&
      line.find("bearing ") != std::string::npos && line.find("clearance ") != std::string::npos)
    {
      ++waypoint_lines;
    }
    if (line.find("critic-side probe") != std::string::npos &&
      line.find("NOT the escape") != std::string::npos &&
      line.find("clearance ") != std::string::npos)
    {
      ++critic_lines;
    }
    EXPECT_EQ(line.find("selected opening"), std::string::npos)
      << "the critic's line must not read as the controller's selection: " << line;
  }
  EXPECT_EQ(waypoint_lines, r.hops) << "one log line per hop";
  EXPECT_GE(critic_lines, 1) << "the critic logs a labelled probe at injection BEGIN";
}

// Debugging aid: run one named geometry of the family, SE_UTRAP_RUNS times.
TEST(ClosedLoopUtrap, NamedGeometryFromTheEnvironment)
{
  const char * want = std::getenv("SE_UTRAP_GEOMETRY");
  if (want == nullptr) {
    GTEST_SKIP() << "set SE_UTRAP_GEOMETRY=<name> (and SE_UTRAP_RUNS) to run one geometry";
  }
  const char * runs_env = std::getenv("SE_UTRAP_RUNS");
  const int runs = runs_env != nullptr ? std::atoi(runs_env) : 1;
  bool found = false;
  auto all = utrap::family();
  for (const auto & p : utrap::primary()) {
    all.push_back(p);
  }
  for (const auto & p : all) {
    if (utrap::name(p) != want) {
      continue;
    }
    found = true;
    for (int run = 0; run < runs; ++run) {
      const auto r = runOnce(p);
      report(r, run);
    }
  }
  EXPECT_TRUE(found) << want << " is neither in the family nor a primary geometry";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
