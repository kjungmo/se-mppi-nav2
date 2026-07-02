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

#ifndef NAV2_SE_CONTROLLER__ENTRAPMENT_STATE_HPP_
#define NAV2_SE_CONTROLLER__ENTRAPMENT_STATE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace nav2_se_controller
{

/**
 * @struct SharedEntrapment
 * @brief Process-local entrapment state shared between the SafeEscapeController
 *        and its EscapeCritic so that a single detector drives both halves.
 *
 * The escape layer (EscapeCritic, sampling-time repulsion/gap costs) and the
 * safety layer (CbfSafetyFilter alpha, via the coordinator) must agree on
 * whether the robot is entrapped — otherwise escape can be injected while the
 * CBF clamps it, or alpha is relaxed with no escape behaviour. The controller
 * is the single source of truth: it owns the EntrapmentDetector, writes
 * `entrapped` each cycle, and sets `driven=true` so the critic knows to follow
 * it (the critic falls back to its own detector with a stock MPPI controller).
 */
struct SharedEntrapment
{
  std::atomic<bool> driven{false};     ///< true once a SafeEscapeController drives this.
  std::atomic<bool> entrapped{false};  ///< current entrapment state from the controller.
};

/**
 * @class EntrapmentRegistry
 * @brief Rendezvous for SharedEntrapment, keyed by the controller plugin name.
 *
 * The controller registers under its own name (e.g. "FollowPath"); its critic
 * registers under its parent controller name (CriticFunction::parent_name_),
 * so both resolve to the same SharedEntrapment without any direct coupling.
 */
class EntrapmentRegistry
{
public:
  static std::shared_ptr<SharedEntrapment> get(const std::string & key)
  {
    static std::mutex mutex;
    static std::map<std::string, std::shared_ptr<SharedEntrapment>> registry;
    std::lock_guard<std::mutex> lock(mutex);
    auto & entry = registry[key];
    if (!entry) {
      entry = std::make_shared<SharedEntrapment>();
    }
    return entry;
  }
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__ENTRAPMENT_STATE_HPP_
