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

#include "nav2_se_controller/cbf_safety_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Sparse>  // NOLINT(build/include_order)
#include <OsqpEigen/OsqpEigen.h>  // NOLINT(build/include_order)

namespace nav2_se_controller
{

namespace
{
constexpr double kInf = 1.0e30;

/// Look-ahead-point Jacobian G = [[cos, -L sin], [sin, L cos]].
inline Eigen::Matrix2d lookaheadJacobian(double yaw, double lookahead)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix2d g;
  g << c, -lookahead * s,
    s, lookahead * c;
  return g;
}

inline Eigen::Vector2d lookaheadPoint(const RobotState & state, double lookahead)
{
  return Eigen::Vector2d(
    state.x + lookahead * std::cos(state.yaw),
    state.y + lookahead * std::sin(state.yaw));
}
}  // namespace

double CbfSafetyFilter::barrierResidual(
  const RobotState & state, double v, double w,
  const TrackedObstacle & obstacle, double alpha) const
{
  const Eigen::Vector2d p_l = lookaheadPoint(state, cfg_.lookahead);
  const Eigen::Vector2d d = p_l - obstacle.position;
  const double q0 = obstacle.q.empty() ? 0.0 : obstacle.q.front();
  const double eff_r =
    cfg_.robot_radius + obstacle.radius + cfg_.safety_margin + q0;
  const double h = d.squaredNorm() - eff_r * eff_r;

  const Eigen::Matrix2d g = lookaheadJacobian(state.yaw, cfg_.lookahead);
  const Eigen::Vector2d pdot = g * Eigen::Vector2d(v, w);
  const double hdot = 2.0 * d.dot(pdot - obstacle.velocity);
  return hdot + alpha * h;
}

CbfSafetyFilter::Result CbfSafetyFilter::filter(
  const RobotState & state, double v_nom, double w_nom,
  const std::vector<TrackedObstacle> & obstacles, double alpha_override) const
{
  const double alpha = alpha_override > 0.0 ? alpha_override : cfg_.alpha;

  // Box-clamped nominal control: the fallback and the "modified" reference.
  const double v_clamped = std::clamp(v_nom, cfg_.v_min, cfg_.v_max);
  const double w_clamped = std::clamp(w_nom, -cfg_.w_max, cfg_.w_max);

  Result fallback;
  fallback.v = v_clamped;
  fallback.w = w_clamped;
  fallback.feasible = true;
  fallback.modified = false;

  if (obstacles.empty()) {
    return fallback;
  }

  // Keep only the nearest max_obstacles (by look-ahead-point distance).
  const Eigen::Vector2d p_l = lookaheadPoint(state, cfg_.lookahead);
  std::vector<const TrackedObstacle *> obs;
  obs.reserve(obstacles.size());
  for (const auto & o : obstacles) {
    obs.push_back(&o);
  }
  const std::size_t n_obs =
    std::min<std::size_t>(obs.size(), static_cast<std::size_t>(std::max(cfg_.max_obstacles, 1)));
  // Keep the nearest by CLEARANCE (gap to the robot footprint), not centre
  // distance, so a large obstacle is not pruned in favour of a smaller one whose
  // centre is closer but whose surface is farther.
  const double base_clearance = cfg_.robot_radius + cfg_.safety_margin;
  std::partial_sort(
    obs.begin(), obs.begin() + n_obs, obs.end(),
    [&p_l, base_clearance](const TrackedObstacle * a, const TrackedObstacle * b) {
      const double ca = (a->position - p_l).norm() - a->radius - base_clearance;
      const double cb = (b->position - p_l).norm() - b->radius - base_clearance;
      return ca < cb;
    });
  obs.resize(n_obs);

  // Decision variables z = [v, w, delta]. Constraints: n_obs CBF rows + 3 boxes.
  const int n_vars = 3;
  const int n_cons = static_cast<int>(n_obs) + 3;

  const Eigen::Matrix2d g = lookaheadJacobian(state.yaw, cfg_.lookahead);

  Eigen::SparseMatrix<double> hessian(n_vars, n_vars);
  hessian.insert(0, 0) = cfg_.w_v;
  hessian.insert(1, 1) = cfg_.w_w;
  hessian.insert(2, 2) = cfg_.slack_weight;
  hessian.makeCompressed();

  Eigen::VectorXd gradient(n_vars);
  gradient << -cfg_.w_v * v_clamped, -cfg_.w_w * w_clamped, 0.0;

  Eigen::SparseMatrix<double> constraints(n_cons, n_vars);
  Eigen::VectorXd lower(n_cons);
  Eigen::VectorXd upper(n_cons);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(n_obs) * 3 + 3);

  // CBF rows:  A z >= b   with   A = [2 d^T G, 1].
  // b depends on the responsibility share (Multi-SE-MPPI N2):
  //   lambda = 1 (default):  b = -alpha h + 2 d^T v_obs   (single-robot, reactive)
  //   lambda < 1 (neighbor): b = -lambda alpha h          (reciprocal share; the
  //                          neighbor enforces the complement, no velocity term)
  // and reverts to the full reactive form inside the emergency band.
  for (std::size_t i = 0; i < n_obs; ++i) {
    const TrackedObstacle & o = *obs[i];
    const Eigen::Vector2d d = p_l - o.position;
    // SE-Predict N3: the conformal bound q inflates the radius (time-varying
    // radius). If the true position stays within q of the prediction, the
    // inflated barrier certifies the true one (triangle inequality). q[0] is
    // the next-step bound — the scale the instantaneous QP acts on.
    const double q0 = o.q.empty() ? 0.0 : o.q.front();
    const double eff_r =
      cfg_.robot_radius + o.radius + cfg_.safety_margin + q0;
    const double h = d.squaredNorm() - eff_r * eff_r;
    const Eigen::RowVector2d a = 2.0 * d.transpose() * g;

    double lambda = std::clamp(o.responsibility, 0.0, 1.0);
    const double gap = d.norm() - eff_r;
    if (gap < cfg_.emergency_dist) {
      lambda = 1.0;  // safety beats protocol when the pair gets critical
    }
    const double b = lambda >= 1.0 - 1.0e-9 ?
      -alpha * h + 2.0 * d.dot(o.velocity) :
      -lambda * alpha * h;

    const int row = static_cast<int>(i);
    triplets.emplace_back(row, 0, a(0));
    triplets.emplace_back(row, 1, a(1));
    triplets.emplace_back(row, 2, 1.0);  // +delta slack
    lower(row) = b;
    upper(row) = kInf;
  }

  // Box rows on v, w, delta.
  const int rv = static_cast<int>(n_obs);
  triplets.emplace_back(rv, 0, 1.0);
  lower(rv) = cfg_.v_min;
  upper(rv) = cfg_.v_max;
  triplets.emplace_back(rv + 1, 1, 1.0);
  lower(rv + 1) = -cfg_.w_max;
  upper(rv + 1) = cfg_.w_max;
  triplets.emplace_back(rv + 2, 2, 1.0);
  lower(rv + 2) = 0.0;     // delta >= 0
  upper(rv + 2) = kInf;

  constraints.setFromTriplets(triplets.begin(), triplets.end());
  constraints.makeCompressed();

  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(false);
  solver.settings()->setAbsoluteTolerance(1.0e-6);
  solver.settings()->setRelativeTolerance(1.0e-6);
  solver.settings()->setMaxIteration(4000);
  solver.data()->setNumberOfVariables(n_vars);
  solver.data()->setNumberOfConstraints(n_cons);

  if (!solver.data()->setHessianMatrix(hessian) ||
    !solver.data()->setGradient(gradient) ||
    !solver.data()->setLinearConstraintsMatrix(constraints) ||
    !solver.data()->setLowerBound(lower) ||
    !solver.data()->setUpperBound(upper) ||
    !solver.initSolver())
  {
    fallback.feasible = false;
    fallback.hard_safe = false;  // could not verify safety -> caller must be conservative
    return fallback;
  }

  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    fallback.feasible = false;
    fallback.hard_safe = false;
    return fallback;
  }

  const Eigen::VectorXd sol = solver.getSolution();

  Result result;
  result.v = std::clamp(sol(0), cfg_.v_min, cfg_.v_max);
  result.w = std::clamp(sol(1), -cfg_.w_max, cfg_.w_max);
  result.slack = std::max(0.0, sol(2));
  result.feasible = true;
  // slack ~ 0 means the barrier holds without relaxation; a non-trivial slack
  // means the QP had to violate a CBF constraint (imminent collision).
  result.hard_safe = result.slack <= 1.0e-3;
  result.modified =
    std::abs(result.v - v_clamped) > 1.0e-3 || std::abs(result.w - w_clamped) > 1.0e-3;
  return result;
}

}  // namespace nav2_se_controller
