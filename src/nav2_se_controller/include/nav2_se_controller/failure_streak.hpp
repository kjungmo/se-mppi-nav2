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

#ifndef NAV2_SE_CONTROLLER__FAILURE_STREAK_HPP_
#define NAV2_SE_CONTROLLER__FAILURE_STREAK_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace nav2_se_controller
{

/**
 * @class FailureStreak
 * @brief How long a per-cycle failure (e.g. the CBF QP) has persisted without a
 *        success in between. Written by the control loop, read by the
 *        diagnostics timer (lock-free). Times are steady-clock nanoseconds.
 */
class FailureStreak
{
public:
  static constexpr double kMinPeriods = 3.0;

  /// A failing cycle that follows the previous recorded cycle by more than
  /// max(3 control periods, gap_s) starts a new streak: a gap means the loop
  /// was idle (goal ended), not that the failure persisted.
  void configure(double controller_frequency_hz, double gap_s = 0.0)
  {
    const double period = std::isfinite(controller_frequency_hz) &&
      controller_frequency_hz > 0.0 ? 1.0 / controller_frequency_hz : 0.05;
    gap_ns_ = static_cast<std::int64_t>(std::max(kMinPeriods * period, gap_s) * 1e9);
  }

  /// Record one control cycle's outcome.
  void record(bool failed, std::int64_t now_ns)
  {
    const std::int64_t previous = last_ns_.exchange(now_ns);
    if (!failed) {
      since_ns_.store(0);
      return;
    }
    if (previous == 0 || now_ns - previous > gap_ns_) {
      since_ns_.store(now_ns);  // first cycle, or the loop was idle: new streak
      return;
    }
    std::int64_t expected = 0;
    since_ns_.compare_exchange_strong(expected, now_ns);
  }

  void clear()
  {
    since_ns_.store(0);
    last_ns_.store(0);
  }

  /// True if failures have persisted for more than `duration_ns` and the most
  /// recent cycle is no older than `stale_ns` (an idle control loop does not
  /// keep a stale streak alive).
  bool longerThan(std::int64_t now_ns, std::int64_t duration_ns, std::int64_t stale_ns) const
  {
    const std::int64_t since = since_ns_.load();
    if (since == 0) {
      return false;
    }
    return now_ns - last_ns_.load() <= stale_ns && now_ns - since > duration_ns;
  }

private:
  std::int64_t gap_ns_{150000000};  // 3 periods at 20 Hz until configured
  std::atomic<std::int64_t> since_ns_{0};
  std::atomic<std::int64_t> last_ns_{0};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__FAILURE_STREAK_HPP_
