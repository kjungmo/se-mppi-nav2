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

#include "nav2_se_controller/mppi_rollout_check.hpp"

#if defined(__linux__)
#include <link.h>
#endif

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/optimizer.hpp"

namespace nav2_se_controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// What checkRolloutDirection() drives through the rollout. 8 x 64 values per
// tensor: enough for the library's vectorised path to run on most of them.
constexpr unsigned int kBatch = 8;
constexpr unsigned int kSteps = 64;
constexpr double kModelDt = 0.05;        // s (MPPI's default)
constexpr double kSpeed = 0.3;           // m/s
constexpr int kHeadings = 16;            // start headings, every pi/8 from -pi
// One full turn over the rollout: every heading is used once more per start.
constexpr double kTurnRate = 2.0 * kPi / (kSteps * kModelDt);  // rad/s

// An mppi::Optimizer that is never initialised: only the members the batch
// integration reads are set (model_dt, the motion model's holonomic flag, the
// start pose and the velocities). The tensors are sized, filled and released
// with tensor-level calls only, so that everything allocated here is also
// freed here, with this file's xtensor storage type.
class BatchIntegrator : public mppi::Optimizer
{
public:
  explicit BatchIntegrator(double model_dt)
  {
    settings_.model_dt = static_cast<float>(model_dt);
    motion_model_ = std::make_shared<mppi::DiffDriveMotionModel>();
  }

  ~BatchIntegrator()
  {
    for (auto * t : {&state_.vx, &state_.vy, &state_.wz, &generated_trajectories_.x,
        &generated_trajectories_.y, &generated_trajectories_.yaws})
    {
      *t = xt::xtensor<float, 2>();
    }
  }

  std::vector<RolloutPoint> rollout(
    double yaw0, double vx, double wz, unsigned int batch, unsigned int steps)
  {
    const std::array<std::size_t, 2> shape{batch, steps};
    for (auto * t : {&state_.vx, &state_.vy, &state_.wz, &generated_trajectories_.x,
        &generated_trajectories_.y, &generated_trajectories_.yaws})
    {
      t->resize(shape);
      t->fill(0.0f);
    }
    state_.vx.fill(static_cast<float>(vx));
    state_.wz.fill(static_cast<float>(wz));
    state_.pose.pose.position.x = 0.0;
    state_.pose.pose.position.y = 0.0;
    state_.pose.pose.orientation.x = 0.0;
    state_.pose.pose.orientation.y = 0.0;
    state_.pose.pose.orientation.z = std::sin(yaw0 / 2.0);
    state_.pose.pose.orientation.w = std::cos(yaw0 / 2.0);

    integrateStateVelocities(generated_trajectories_, state_);

    std::vector<RolloutPoint> out(static_cast<std::size_t>(batch) * steps);
    for (unsigned int b = 0; b < batch; ++b) {
      for (unsigned int t = 0; t < steps; ++t) {
        out[static_cast<std::size_t>(b) * steps + t] = RolloutPoint{
          static_cast<double>(generated_trajectories_.x(b, t)),
          static_cast<double>(generated_trajectories_.y(b, t))};
      }
    }
    return out;
  }
};

#if defined(__linux__)
int findMppiLibrary(struct dl_phdr_info * info, std::size_t, void * out)
{
  if (info->dlpi_name != nullptr && std::strstr(info->dlpi_name, "libmppi_controller") != nullptr) {
    *static_cast<std::string *>(out) = info->dlpi_name;
    return 1;  // stop
  }
  return 0;
}
#endif

}  // namespace

std::vector<RolloutPoint> mppiBatchRollout(
  double yaw0, double vx, double wz, unsigned int batch, unsigned int steps, double model_dt)
{
  if (batch == 0 || steps == 0) {
    return {};
  }
  BatchIntegrator integrator(model_dt);
  return integrator.rollout(yaw0, vx, wz, batch, steps);
}

std::vector<RolloutPoint> closedFormRollout(
  double yaw0, double vx, double wz, unsigned int steps, double model_dt)
{
  std::vector<RolloutPoint> out(steps);
  double x = 0.0;
  double y = 0.0;
  double yaw = yaw0;  // the heading the step starts with
  for (unsigned int t = 0; t < steps; ++t) {
    x += vx * std::cos(yaw) * model_dt;
    y += vx * std::sin(yaw) * model_dt;
    yaw += wz * model_dt;
    out[t] = RolloutPoint{x, y};
  }
  return out;
}

void accumulateRolloutComparison(
  const std::vector<RolloutPoint> & got, const std::vector<RolloutPoint> & expected,
  unsigned int batch, RolloutCheckResult & result)
{
  const std::size_t steps = expected.size();
  const std::size_t total = static_cast<std::size_t>(batch) * steps;
  result.points += total;
  if (got.size() != total) {
    result.wrong_points += total;
    return;
  }
  for (std::size_t i = 0; i < total; ++i) {
    const RolloutPoint & want = expected[i % steps];
    const double error = std::hypot(got[i].x - want.x, got[i].y - want.y);
    // Written so that a NaN counts as wrong (an infinity does anyway).
    if (!(error <= kRolloutTolerance)) {
      ++result.wrong_points;
    }
    if (std::isfinite(error) && error > result.worst_error) {
      result.worst_error = error;
    }
  }
}

RolloutCheckResult checkRolloutDirection(const RolloutFunction & rollout)
{
  RolloutCheckResult result;
  for (int i = 0; i < kHeadings; ++i) {
    const double yaw0 = -kPi + i * (2.0 * kPi / kHeadings);
    for (const double wz : {0.0, kTurnRate, -kTurnRate}) {
      const std::vector<RolloutPoint> expected =
        closedFormRollout(yaw0, kSpeed, wz, kSteps, kModelDt);
      const std::vector<RolloutPoint> got = rollout(yaw0, kSpeed, wz, kBatch, kSteps, kModelDt);
      accumulateRolloutComparison(got, expected, kBatch, result);
      if (i == kHeadings / 2 && wz == 0.0) {  // heading 0, straight
        result.straight_end_expected = expected[kSteps - 1].x;
        // NaN if the rollout has the wrong size (then every point is wrong anyway).
        result.straight_end_library = got.size() >= kSteps ? got[kSteps - 1].x : std::nan("");
      }
    }
  }
  result.ok = result.wrong_points == 0;  // points is never 0: wrong sizes count as wrong
  return result;
}

RolloutCheckResult checkMppiRolloutDirection()
{
  RolloutCheckResult result = checkRolloutDirection(&mppiBatchRollout);
#if defined(__linux__)
  dl_iterate_phdr(&findMppiLibrary, &result.library_path);
#endif
  return result;
}

std::string describeRolloutFailure(const RolloutCheckResult & result)
{
  char measured[320];
  std::snprintf(
    measured, sizeof(measured),
    "a straight %.1f m/s rollout at heading 0 ends at x = %+.3f m (expected %+.3f m); "
    "%zu of %zu rollout points are more than %.0e m from the closed form "
    "(worst finite deviation %.3f m)",
    kSpeed, result.straight_end_library, result.straight_end_expected,
    result.wrong_points, result.points, kRolloutTolerance, result.worst_error);
  return std::string("the loaded nav2_mppi_controller library (") +
         (result.library_path.empty() ? "path not found" : result.library_path) +
         ") integrates sampled trajectories wrongly: " + measured +
         ". The optimizer would score its samples at wrong positions and can command motion "
         "away from the goal. Known cause: a binary built with -ffast-math, which negates "
         "cos/sin in the batch rollout: RoboStack ros-humble-nav2-mppi-controller 1.1.20 "
         "(builds _18 and _20) and ros-jazzy-nav2-mppi-controller 1.3.11 (build _16); see "
         "https://github.com/RoboStack/ros-jazzy/issues/290. Remedy: on Jazzy install "
         "nav2_mppi_controller 1.3.13 build _25 or later; on Humble build nav2_mppi_controller "
         "from source without -ffast-math (scripts/build_mppi_overlay.sh in the se-mppi-nav2 "
         "repository) and rebuild nav2_se_controller against it";
}

}  // namespace nav2_se_controller
