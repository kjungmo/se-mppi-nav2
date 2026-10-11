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

#ifndef NAV2_SE_CONTROLLER__MPPI_ROLLOUT_CHECK_HPP_
#define NAV2_SE_CONTROLLER__MPPI_ROLLOUT_CHECK_HPP_

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

// Self-check of the installed nav2_mppi_controller library's batch rollout.
//
// The MPPI optimizer scores each sampled control sequence at the positions
// that Optimizer::integrateStateVelocities(Trajectories &, const State &)
// computes for it. Some distributed binaries of the library negate cos/sin in
// that function (built with -ffast-math, whose -fno-signed-zeros breaks the
// SIMD trigonometry; README, "Affected nav2_mppi_controller builds"), so the
// optimizer sees every sample at a mirrored position and commands motion away
// from the goal. This header lets the controller and the tests ask the
// installed library directly.
//
// The functions use a throwaway mppi::Optimizer that is never initialised: no
// node, parameters, costmap, TF, critics or noise thread, and nothing is shared
// with the controller's own optimizer. They are deterministic (no sampling)
// and hold no state between calls. This header is plain C++ on purpose: the
// implementation is compiled with MPPI's xtensor flags (CMakeLists.txt,
// boundary 1), its callers are not.

namespace nav2_se_controller
{

struct RolloutPoint
{
  double x{0.0};
  double y{0.0};
};

/// The positions the installed library computes for `batch` identical samples
/// of the constant command (vx, wz) held for `steps` steps of `model_dt`,
/// robot starting at the origin with heading `yaw0`. Differential-drive model.
/// Row-major: sample b, step t is element [b * steps + t]. Empty if batch or
/// steps is 0.
std::vector<RolloutPoint> mppiBatchRollout(
  double yaw0, double vx, double wz, unsigned int batch, unsigned int steps, double model_dt);

/// The same rollout for one sample, from MPPI's update rule evaluated with
/// scalar double libm: x[t] = x[t-1] + vx cos(yaw[t-1]) dt (and sin for y),
/// yaw[t] = yaw[t-1] + wz dt, yaw[-1] = yaw0.
std::vector<RolloutPoint> closedFormRollout(
  double yaw0, double vx, double wz, unsigned int steps, double model_dt);

struct RolloutCheckResult
{
  bool ok{false};                // every compared point within kRolloutTolerance
  std::size_t points{0};         // rollout points compared
  std::size_t wrong_points{0};   // off by more than the tolerance, or not finite
  double worst_error{0.0};       // [m] largest finite deviation from the closed form
  // The plainest case, for the log: heading 0, straight, 0.3 m/s for 3.2 s.
  double straight_end_expected{0.0};  // [m] x of the closed form's end point
  double straight_end_library{0.0};   // [m] x of the library's end point
  std::string library_path;      // libmppi_controller.so as loaded; empty if not found
};

/// Deviation from the closed form above which a rollout point counts as wrong.
/// The library works in float32: over the 64 steps checked its legitimate
/// rounding stays below 1e-4 m, while one negated step is off by up to 0.03 m.
constexpr double kRolloutTolerance = 1.0e-3;  // m

/// Add the comparison of a batch rollout `got` (rows of expected.size() steps,
/// row-major) with the one-sample closed form `expected` to result.points,
/// result.wrong_points and result.worst_error. A point is wrong if it is more
/// than kRolloutTolerance from the closed form or not finite. If `got` is not
/// `batch` rows of expected.size() points, every expected point counts as wrong.
void accumulateRolloutComparison(
  const std::vector<RolloutPoint> & got, const std::vector<RolloutPoint> & expected,
  unsigned int batch, RolloutCheckResult & result);

/// A batch rollout with the signature of mppiBatchRollout().
using RolloutFunction = std::function<std::vector<RolloutPoint>(
      double yaw0, double vx, double wz, unsigned int batch, unsigned int steps,
      double model_dt)>;

/// Compare `rollout` with the closed form over 16 start headings (every pi/8
/// from -pi), each driven straight and through a full left and right turn:
/// 48 rollouts of 8 samples x 64 steps, 24,576 points. library_path is left
/// empty. The tests pass a model of the known defect here.
RolloutCheckResult checkRolloutDirection(const RolloutFunction & rollout);

/// checkRolloutDirection() on the installed library (mppiBatchRollout), with
/// library_path filled in. About 0.5 ms; allocates only inside the call.
RolloutCheckResult checkMppiRolloutDirection();

/// One paragraph for a log or a diagnostics message: what was measured, the
/// known cause and the remedy.
std::string describeRolloutFailure(const RolloutCheckResult & result);

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__MPPI_ROLLOUT_CHECK_HPP_
