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

#ifndef NAV2_SE_CONTROLLER__PATH_PROGRESS_HPP_
#define NAV2_SE_CONTROLLER__PATH_PROGRESS_HPP_

#include <cstddef>

#include "nav_msgs/msg/path.hpp"

namespace nav2_se_controller
{

/**
 * @brief Index of the path pose nearest to (x, y) in the XY plane.
 *
 * Used as the controller-side global-path progress signal that drives the
 * entrapment detector: a non-increasing nearest index means the robot is not
 * advancing along the plan. Returns 0 for an empty path.
 */
std::size_t nearestPathIndex(const nav_msgs::msg::Path & path, double x, double y);

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__PATH_PROGRESS_HPP_
