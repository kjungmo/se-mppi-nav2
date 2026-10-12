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

// ros2 run nav2_se_controller check_mppi_rollout
//
// Runs the controller's configure-time self-check on the nav2_mppi_controller
// library this package loads, without a node. Exit status 0: the library rolls
// trajectories out correctly. 1: it does not (the reason goes to stderr).

#include <cstdio>

#include "nav2_se_controller/mppi_rollout_check.hpp"

int main()
{
  const nav2_se_controller::RolloutCheckResult result =
    nav2_se_controller::checkMppiRolloutDirection();
  if (!result.ok) {
    std::fprintf(
      stderr, "FAILED: %s.\n", nav2_se_controller::describeRolloutFailure(result).c_str());
    return 1;
  }
  std::printf(
    "OK: %s rolls trajectories out correctly (%zu rollout points within %.0e m of the closed "
    "form, worst %.1e m).\n",
    result.library_path.empty() ? "the loaded nav2_mppi_controller library" :
    result.library_path.c_str(),
    result.points, nav2_se_controller::kRolloutTolerance, result.worst_error);
  return 0;
}
