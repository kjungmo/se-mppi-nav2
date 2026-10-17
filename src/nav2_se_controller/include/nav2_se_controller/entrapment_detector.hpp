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

#ifndef NAV2_SE_CONTROLLER__ENTRAPMENT_DETECTOR_HPP_
#define NAV2_SE_CONTROLLER__ENTRAPMENT_DETECTOR_HPP_

#include <cstddef>

namespace nav2_se_controller
{

/**
 * @struct EntrapmentConfig
 * @brief Tunables for the entrapment detector.
 */
struct EntrapmentConfig
{
  /// Cycles without global-path progress before declaring entrapment.
  int progress_stall_window{30};
};

/**
 * @class EntrapmentDetector
 * @brief Online detect-and-switch local-minima detector for SE-MPPI.
 *
 * Pure, side-effect-free logic (no ROS / costmap deps) so it is unit-testable
 * in isolation. Signal: stall in the (monotonic) furthest-reached global-path
 * index. Entrapment latches after progress_stall_window stalled cycles and
 * clears immediately when real progress resumes (a furthest-index increase),
 * so intermittent stop-and-go motion during escape cannot leave it stuck.
 *
 * See docs/architecture/2026-06_safe-escape-mppi-design.md §3.1.
 */
class EntrapmentDetector
{
public:
  void configure(const EntrapmentConfig & cfg) {cfg_ = cfg;}

  /**
   * @brief Feed one control cycle's global-path progress.
   * @param furthest_path_point Monotonic furthest reached pruned-path index.
   * @return true if currently entrapped.
   */
  bool update(std::size_t furthest_path_point)
  {
    if (!seen_) {
      seen_ = true;
      last_furthest_ = furthest_path_point;
      return entrapped_;
    }

    if (furthest_path_point > last_furthest_) {
      // Real progress: advance and clear any entrapment.
      last_furthest_ = furthest_path_point;
      stall_counter_ = 0;
      entrapped_ = false;
    } else if (!entrapped_) {
      // No progress this cycle.
      if (++stall_counter_ >= cfg_.progress_stall_window) {
        entrapped_ = true;
      }
    }
    return entrapped_;
  }

  void reset()
  {
    seen_ = false;
    entrapped_ = false;
    stall_counter_ = 0;
    last_furthest_ = 0;
  }

  bool entrapped() const {return entrapped_;}
  int stallCount() const {return stall_counter_;}

private:
  EntrapmentConfig cfg_{};
  bool seen_{false};
  bool entrapped_{false};
  int stall_counter_{0};
  std::size_t last_furthest_{0};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__ENTRAPMENT_DETECTOR_HPP_
