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

#ifndef NAV2_SE_CONTROLLER__CONFORMAL_CALIBRATOR_HPP_
#define NAV2_SE_CONTROLLER__CONFORMAL_CALIBRATOR_HPP_

#include <cstdint>
#include <vector>

namespace nav2_se_controller
{

/// Tunables for the online conformal calibrator.
struct ConformalConfig
{
  double coverage{0.9};        ///< target P(residual <= q) per horizon step.
  double learning_rate{0.02};  ///< quantile-tracking step (m per observation).
  double initial_q{0.05};      ///< starting bound (≈ the CBF safety margin).
  double max_q{0.40};          ///< cap — the over-conservatism guard (design §7):
                               ///< an unbounded q would freeze the robot in
                               ///< exactly the narrow spaces escape exists for.
};

/**
 * @class ConformalCalibrator
 * @brief Online per-step prediction-error bounds q_k (SE-Predict N3).
 *
 * For each prediction-horizon step k it tracks the `coverage`-quantile of the
 * observed residuals ||p_observed - p_predicted(k)|| with the standard online
 * quantile (pinball-gradient) update — the adaptive-conformal-style rule:
 *
 *     q_k <- clamp(q_k + lr * (1{res > q_k} - (1 - coverage)), 0, max_q)
 *
 * which is distribution-free: at stationarity the miss rate equals
 * 1 - coverage whatever the residual distribution, and under shift q_k tracks
 * the new quantile at the learning rate. Residuals are pooled across tracks
 * (more samples per step; per-agent calibration is future work).
 *
 * The bound is consumed by the CBF as a time-varying radius inflation
 * (eff_r + q): if the true position stays within q of the prediction, the
 * inflated barrier certifies the true one (triangle inequality — design §4).
 * The guarantee is asymptotic in the online setting; the early phase leans on
 * the existing static safety_margin and initial_q.
 */
class ConformalCalibrator
{
public:
  void configure(const ConformalConfig & cfg, int horizon_steps)
  {
    cfg_ = cfg;
    q_.assign(
      static_cast<std::size_t>(horizon_steps > 0 ? horizon_steps : 0),
      cfg.initial_q);
    n_.assign(q_.size(), 0);
    misses_.assign(q_.size(), 0);
  }
  const ConformalConfig & config() const {return cfg_;}

  /// Feed one observed residual (m) for horizon step k.
  void observe(int k, double residual)
  {
    if (k < 0 || static_cast<std::size_t>(k) >= q_.size()) {
      return;
    }
    const auto i = static_cast<std::size_t>(k);
    const bool miss = residual > q_[i];
    q_[i] += cfg_.learning_rate *
      ((miss ? 1.0 : 0.0) - (1.0 - cfg_.coverage));
    q_[i] = q_[i] < 0.0 ? 0.0 : (q_[i] > cfg_.max_q ? cfg_.max_q : q_[i]);
    ++n_[i];
    if (miss) {
      ++misses_[i];
    }
  }

  /// Current bound for step k (initial_q when k is out of range).
  double q(int k) const
  {
    if (k < 0 || static_cast<std::size_t>(k) >= q_.size()) {
      return cfg_.initial_q;
    }
    return q_[static_cast<std::size_t>(k)];
  }

  /// All per-step bounds (size = horizon_steps).
  const std::vector<double> & qAll() const {return q_;}

  /// Observations / empirical miss rate per step (diagnostics, tests).
  std::int64_t samples(int k) const
  {
    if (k < 0 || static_cast<std::size_t>(k) >= n_.size()) {
      return 0;
    }
    return n_[static_cast<std::size_t>(k)];
  }
  double missRate(int k) const
  {
    const auto s = samples(k);
    return s > 0 ?
           static_cast<double>(misses_[static_cast<std::size_t>(k)]) /
           static_cast<double>(s) : 0.0;
  }

  void reset()
  {
    for (auto & v : q_) {
      v = cfg_.initial_q;
    }
    n_.assign(n_.size(), 0);
    misses_.assign(misses_.size(), 0);
  }

private:
  ConformalConfig cfg_{};
  std::vector<double> q_;
  std::vector<std::int64_t> n_;
  std::vector<std::int64_t> misses_;
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__CONFORMAL_CALIBRATOR_HPP_
