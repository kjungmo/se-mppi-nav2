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

#include "nav2_se_controller/trajectory_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)

namespace nav2_se_controller
{

namespace
{

Eigen::Vector2d clampNorm(const Eigen::Vector2d & v, double max_norm)
{
  const double n = v.norm();
  if (n > max_norm && n > 1.0e-9) {
    return v * (max_norm / n);
  }
  return v;
}

/// Least-squares polynomial fit of the track (degree 1 = CV, 2 = CVCA),
/// time-centred on the newest point. Fitting over the WHOLE history (instead
/// of finite differences on the last 2-3 points) is what makes the estimate
/// usable under centroid jitter: a 2 cm noise on 0.1 s spacing turns a
/// finite-difference acceleration into pure noise (sigma ~ 5.6 m/s^2), while
/// the 10-point fit averages it down by an order of magnitude. Measured in
/// experiments/prediction/ade_fde_eval.py (math parity with this file).
struct MotionFit
{
  Eigen::Vector2d p0{0.0, 0.0};
  Eigen::Vector2d v{0.0, 0.0};
  Eigen::Vector2d a{0.0, 0.0};
};

MotionFit fitPolynomial(const std::deque<TrackPoint> & history, int degree)
{
  MotionFit fit;
  const auto n = static_cast<int>(history.size());
  fit.p0 = history.back().position;
  if (n < 2) {
    return fit;
  }
  degree = std::min(degree, n >= 5 ? 2 : 1);  // quadratic needs support

  const double t_last = history.back().stamp;
  Eigen::MatrixXd X(n, degree + 1);
  Eigen::MatrixXd Y(n, 2);
  for (int i = 0; i < n; ++i) {
    const double tr = history[i].stamp - t_last;
    double pow_t = 1.0;
    for (int d = 0; d <= degree; ++d) {
      X(i, d) = pow_t;        // columns 1, t, t^2
      pow_t *= tr;
    }
    Y.row(i) = history[i].position.transpose();
  }
  const Eigen::MatrixXd C =
    (X.transpose() * X).ldlt().solve(X.transpose() * Y);  // (deg+1) x 2

  fit.p0 = C.row(0).transpose();
  fit.v = C.row(1).transpose();
  if (degree >= 2) {
    fit.a = 2.0 * C.row(2).transpose();
  }
  return fit;
}

}  // namespace

std::vector<Eigen::Vector2d> TrajectoryPredictor::predict(
  const std::deque<TrackPoint> & history) const
{
  std::vector<Eigen::Vector2d> horizon;
  horizon.reserve(cfg_.horizon_steps);
  if (history.empty()) {
    return horizon;
  }

  const int degree =
    cfg_.model == PredictorConfig::Model::kConstantAcceleration ? 2 : 1;
  MotionFit fit = fitPolynomial(history, degree);
  fit.v = clampNorm(fit.v, cfg_.max_speed);
  fit.a = clampNorm(fit.a, cfg_.max_accel);

  // Roll the model forward with geometric damping on the acceleration: a raw
  // quadratic diverges, and the damped rollout reduces to CV as gamma^k -> 0.
  Eigen::Vector2d p = fit.p0;
  Eigen::Vector2d vel = fit.v;
  for (int k = 0; k < cfg_.horizon_steps; ++k) {
    const double damp = std::pow(cfg_.accel_damping, k);
    vel = clampNorm(vel + fit.a * damp * cfg_.horizon_dt, cfg_.max_speed);
    p += vel * cfg_.horizon_dt;
    horizon.push_back(p);
  }
  return horizon;
}

}  // namespace nav2_se_controller
