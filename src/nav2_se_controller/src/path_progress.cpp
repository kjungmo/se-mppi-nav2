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

#include "nav2_se_controller/path_progress.hpp"

#include <limits>

namespace nav2_se_controller
{

std::size_t nearestPathIndex(const nav_msgs::msg::Path & path, double x, double y)
{
  std::size_t best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - x;
    const double dy = path.poses[i].pose.position.y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

}  // namespace nav2_se_controller
