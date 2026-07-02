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

#ifndef NAV2_SE_CONTROLLER__CBF_TYPES_HPP_
#define NAV2_SE_CONTROLLER__CBF_TYPES_HPP_

#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

namespace nav2_se_controller
{

/// Planar robot pose (world frame).
struct RobotState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A tracked obstacle with a constant-velocity model (world frame).
struct TrackedObstacle
{
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  double radius{0.0};

  /// False when the tracker classified this as persistent static structure
  /// (SE-Predict N1, occupancy persistence) — walls/furniture belong to the
  /// MPPI/costmap path, not the CBF. Defaults true: an unproven obstacle
  /// stays eligible for the CBF (the conservative direction).
  bool is_dynamic{true};

  /// Predicted future positions at fixed spacing (SE-Predict N2; design §2.1
  /// TrackedObstaclePred). EMPTY means no prediction available — consumers
  /// must fall back to the constant-velocity predict() (the legacy path),
  /// which keeps the migration incremental.
  std::vector<Eigen::Vector2d> horizon;

  /// Conformal per-step prediction-error bounds q_k (SE-Predict N3), aligned
  /// with `horizon`. EMPTY = uncalibrated; consumers fall back to their
  /// static margins. The CBF inflates the effective radius by q (time-varying
  /// radius): bounded prediction error then implies the true barrier holds.
  std::vector<double> q;

  /// Reciprocal barrier-budget share lambda (Multi-SE-MPPI N2).
  /// 1.0 (default) = the single-robot constraint: hdot >= -alpha h with the
  /// reactive velocity term — behaviour unchanged. < 1.0 marks a reciprocal
  /// NEIGHBOR ROBOT running the same protocol: this robot enforces
  /// a_i u_i >= -lambda alpha h (its share of the barrier-decay budget; no
  /// velocity term — the neighbor's motion is covered by the neighbor's own
  /// share). With lambda_ij + lambda_ji = 1 the pair of QPs restores the
  /// joint hdot + alpha h >= 0. Larger lambda = more allowance (a passer);
  /// smaller = more deference (a yielder). Crucially, neither side carries
  /// the other's closing velocity, which is what removes the symmetric
  /// double-braking standoff of two reactive single-robot filters.
  double responsibility{1.0};

  /// Predicted position after dt seconds under the constant-velocity model.
  Eigen::Vector2d predict(double dt) const {return position + velocity * dt;}
};

/// Tunables for the CBF safety filter.
struct CbfConfig
{
  double alpha{2.0};             ///< class-K gain on the barrier (continuous CBF).
  double lookahead{0.2};         ///< L: look-ahead point distance ahead of base (m), > 0.
  double robot_radius{0.22};     ///< robot inscribed radius (m).
  double safety_margin{0.05};    ///< extra clearance added to each obstacle (m).
  double slack_weight{1.0e3};    ///< rho: penalty on constraint slack (feasibility).
  double emergency_dist{0.15};   ///< surface gap below which a reciprocal
                                 ///< (responsibility < 1) constraint reverts to
                                 ///< full responsibility + the velocity term —
                                 ///< the multi-robot analogue of the TTC
                                 ///< override (safety beats protocol).
  double w_v{1.0};               ///< tracking weight on linear velocity.
  double w_w{1.0};               ///< tracking weight on angular velocity.
  double v_min{-0.35};
  double v_max{0.5};
  double w_max{1.9};
  int max_obstacles{20};         ///< cap on nearest obstacles fed to the QP.
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__CBF_TYPES_HPP_
