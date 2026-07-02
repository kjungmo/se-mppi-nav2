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

#ifndef NAV2_SE_CONTROLLER__CBF_SAFETY_FILTER_HPP_
#define NAV2_SE_CONTROLLER__CBF_SAFETY_FILTER_HPP_

#include <vector>

#include "nav2_se_controller/cbf_types.hpp"

namespace nav2_se_controller
{

/**
 * @class CbfSafetyFilter
 * @brief Control-Barrier-Function safety filter for a differential-drive robot.
 *
 * Projects a nominal control u_nom = (v, w) onto the closest control that keeps
 * the robot's forward-invariant safe set, w.r.t. moving obstacles. To make the
 * barrier relative-degree one in (v, w) for a unicycle, safety is enforced on a
 * look-ahead point P offset by L ahead of the base — which is fully actuated by
 * (v, w) — a standard differential-drive feedback-linearisation trick.
 *
 * For each obstacle j (radius R_j, constant-velocity), with d = P - p_j and
 * effective radius R = robot_radius + R_j + margin:
 *   h_j      = d.d - R^2                                  (>= 0 is safe)
 *   d/dt h_j = 2 d^T (G u - v_j),  G = [[cos, -L sin],[sin, L cos]]
 * The continuous CBF condition  d/dt h_j + alpha * h_j >= 0  is a linear
 * inequality in u. A small QP minimises ||u - u_nom||^2 + rho*delta^2 subject to
 * these constraints (relaxed by slack delta >= 0 to preserve feasibility) and the
 * input box limits. Solved with OSQP.
 *
 * Pure (no ROS/costmap dependency) and unit-tested. See design §3.2; the
 * escape-safety coordination (alpha modulation when entrapped) is M3 — exposed
 * here via the alpha_override argument.
 */
class CbfSafetyFilter
{
public:
  struct Result
  {
    double v{0.0};
    double w{0.0};
    bool feasible{true};    ///< QP solved (a solution was returned).
    double slack{0.0};      ///< constraint slack used.
    bool hard_safe{true};   ///< slack ~ 0: the barrier holds without relaxation.
    bool modified{false};   ///< control differs from nominal beyond tolerance.
  };

  void configure(const CbfConfig & cfg) {cfg_ = cfg;}
  const CbfConfig & config() const {return cfg_;}

  /**
   * @brief Filter a nominal control to be CBF-safe.
   * @param state Current robot pose.
   * @param v_nom Nominal linear velocity.
   * @param w_nom Nominal angular velocity.
   * @param obstacles Tracked obstacles (constant-velocity).
   * @param alpha_override If > 0, replaces cfg.alpha for this call (M3 hook).
   * @return Filtered control + diagnostics. Falls back to the (box-clamped)
   *         nominal control if the QP fails.
   */
  Result filter(
    const RobotState & state,
    double v_nom,
    double w_nom,
    const std::vector<TrackedObstacle> & obstacles,
    double alpha_override = -1.0) const;

  /**
   * @brief Barrier-condition residual  d/dt h_j + alpha * h_j  for a given
   *        control and obstacle. >= 0 means the CBF constraint is satisfied.
   *        Exposed for testing and for the M3 coordinator.
   */
  double barrierResidual(
    const RobotState & state,
    double v,
    double w,
    const TrackedObstacle & obstacle,
    double alpha) const;

private:
  CbfConfig cfg_{};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__CBF_SAFETY_FILTER_HPP_
