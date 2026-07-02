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

#ifndef NAV2_SE_CONTROLLER__ESCAPE_CRITIC_HPP_
#define NAV2_SE_CONTROLLER__ESCAPE_CRITIC_HPP_

#include <memory>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_se_controller/entrapment_detector.hpp"
#include "nav2_se_controller/entrapment_state.hpp"

// The stock nav2_mppi CriticManager resolves a critics-list entry NAME to the
// plugin class "mppi::critics::" + NAME, so a loadable MPPI critic MUST live in
// the mppi::critics namespace. (Entrapment helpers stay in nav2_se_controller.)
namespace mppi
{
namespace critics
{

/**
 * @class EscapeCritic
 * @brief Detect-and-switch local-minima escape critic for MPPI (SE-MPPI).
 *
 * Rides on the stock nav2_mppi_controller optimizer as a pluginlib critic.
 * Detects entrapment online (global-path progress stall) and — only when
 * triggered — adds a repulsive-potential cost term so the optimizer is biased
 * toward detouring into open space. Unlike the always-on PreferForward/Twirling
 * critics, this is an explicit detect-and-switch mechanism (extends DRPA-MPPI,
 * arXiv:2503.20134).
 *
 * STATUS: M1. Entrapment detection (progress stall + hysteresis) and a
 * cost-proxy repulsion term are implemented and unit-tested. A true
 * distance-field APF, free-space gap search, and coordination with the CBF
 * safety filter are tracked as M1.x / M3 in the design doc.
 * See docs/architecture/2026-06_safe-escape-mppi-design.md §3.1.
 */
class EscapeCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

protected:
  // Own detector: used as a fallback when not driven by a SafeEscapeController
  // (e.g. with the stock MPPI controller). When the controller drives the shared
  // state, the critic follows it so escape and the CBF coordinator agree.
  nav2_se_controller::EntrapmentDetector detector_;
  std::shared_ptr<nav2_se_controller::SharedEntrapment> shared_;

  // --- parameters ---
  // always_on_: inject escape costs every cycle (no gating) -- ablation baseline.
  bool always_on_{false};
  bool use_apf_{true};            // true: true distance-field APF (M1.x); false: cost-proxy (M1)
  float repulsion_weight_{2.0f};  // cost-proxy gain
  int repulsion_power_{1};
  float apf_influence_dist_{0.6f};  // APF d0 (m)
  float apf_eta_{1.0f};             // APF gain

  // free-space gap search (temporary escape subgoal toward an opening)
  bool use_gap_search_{true};
  float gap_weight_{4.0f};
  int gap_power_{1};
  int gap_num_rays_{36};
  float gap_max_range_{2.0f};
  float gap_min_clearance_{0.6f};

  bool prev_injecting_{false};  // for BEGIN/END injection transition logs
};

}  // namespace critics
}  // namespace mppi

#endif  // NAV2_SE_CONTROLLER__ESCAPE_CRITIC_HPP_
