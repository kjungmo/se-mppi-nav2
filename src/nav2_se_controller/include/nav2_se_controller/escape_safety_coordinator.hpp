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

#ifndef NAV2_SE_CONTROLLER__ESCAPE_SAFETY_COORDINATOR_HPP_
#define NAV2_SE_CONTROLLER__ESCAPE_SAFETY_COORDINATOR_HPP_

#include <limits>
#include <vector>

#include "nav2_se_controller/cbf_types.hpp"

namespace nav2_se_controller
{

/// Tunables for the escape-safety coordinator.
struct CoordinationConfig
{
  double alpha_base{2.0};            ///< conservative CBF gain (normal driving).
  double alpha_escape{6.0};          ///< relaxed CBF gain while escaping (tighter manoeuvres).
  double ttc_override_threshold{1.5};  ///< [s] below this dynamic TTC, safety overrides escape.
  double q_trust_threshold{0.25};    ///< [m] max conformal bound under which an
                                     ///< aggressive escape is still sanctioned:
                                     ///< "escape boldly only when the prediction
                                     ///< is trustworthy" (design §5). Above it,
                                     ///< alpha stays base while the q-inflated
                                     ///< radius already widens the margin — the
                                     ///< two-variable coordination.
};

/**
 * @brief Minimum time-to-collision between the robot and any obstacle.
 *
 * Constant-velocity closing-speed model. The robot translates at @p v along its
 * heading; each obstacle moves at its own constant velocity. Returns +inf when
 * nothing is approaching. Used by the coordinator to gate aggressive escape.
 *
 * @param state Robot pose.
 * @param v Robot forward speed (m/s).
 * @param obstacles Tracked obstacles.
 * @param robot_radius Robot inscribed radius (m).
 */
double minTimeToCollision(
  const RobotState & state,
  double v,
  const std::vector<TrackedObstacle> & obstacles,
  double robot_radius);

/**
 * @class EscapeSafetyCoordinator
 * @brief Reconciles local-minima escape with formal CBF safety (SE-MPPI core).
 *
 * The escape layer (EscapeCritic) and the safety layer (CbfSafetyFilter) pull in
 * opposite directions: repulsive escape pushes the robot toward the safe-set
 * boundary, while the CBF filter clamps exactly those aggressive samples. This
 * coordinator resolves the conflict by modulating the CBF class-K gain alpha:
 *
 *   - Not entrapped            -> alpha_base   (stay well clear of obstacles).
 *   - Entrapped, TTC safe      -> alpha_escape (permit tighter detours to escape).
 *   - Entrapped, TTC imminent  -> alpha_base   (dynamic safety overrides escape).
 *
 * Crucially, forward invariance (h >= 0, i.e. no collision) holds for ANY
 * alpha > 0; raising alpha only permits the robot to approach the boundary more
 * closely. Thus escape manoeuvres remain certified-safe — this coordination, not
 * the sum of the two layers, is the contribution. See design §3.2.
 */
class EscapeSafetyCoordinator
{
public:
  void configure(const CoordinationConfig & cfg) {cfg_ = cfg;}
  const CoordinationConfig & config() const {return cfg_;}

  /// CBF gain alpha to use this cycle, given entrapment + dynamic risk and
  /// (SE-Predict N3) the prediction trust: max_q is the largest conformal
  /// bound among the obstacles the CBF will act on (0 = perfectly trusted /
  /// uncalibrated-legacy). An untrustworthy prediction keeps alpha at base —
  /// the escape proceeds cautiously while the q-inflated radius covers the
  /// model error.
  double alpha(bool entrapped, double min_ttc, double max_q = 0.0) const
  {
    if (!entrapped) {
      return cfg_.alpha_base;
    }
    if (min_ttc < cfg_.ttc_override_threshold) {
      return cfg_.alpha_base;  // safety overrides escape
    }
    if (max_q > cfg_.q_trust_threshold) {
      return cfg_.alpha_base;  // prediction not trusted: escape cautiously
    }
    return cfg_.alpha_escape;
  }

  /// Whether an aggressive escape manoeuvre is currently sanctioned.
  bool escapeSanctioned(bool entrapped, double min_ttc) const
  {
    return entrapped && min_ttc >= cfg_.ttc_override_threshold;
  }

  /// Convenience: compute TTC + prediction trust from obstacles then resolve
  /// alpha in one call.
  double resolveAlpha(
    bool entrapped, const RobotState & state, double v,
    const std::vector<TrackedObstacle> & obstacles, double robot_radius) const
  {
    double max_q = 0.0;
    for (const auto & o : obstacles) {
      if (!o.q.empty() && o.q.front() > max_q) {
        max_q = o.q.front();
      }
    }
    return alpha(
      entrapped, minTimeToCollision(state, v, obstacles, robot_radius), max_q);
  }

private:
  CoordinationConfig cfg_{};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__ESCAPE_SAFETY_COORDINATOR_HPP_
