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

#ifndef NAV2_SE_CONTROLLER__TRAJECTORY_PREDICTOR_HPP_
#define NAV2_SE_CONTROLLER__TRAJECTORY_PREDICTOR_HPP_

#include <deque>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

namespace nav2_se_controller
{

/// One timestamped position observation of a tracked obstacle.
struct TrackPoint
{
  double stamp{0.0};
  Eigen::Vector2d position{0.0, 0.0};
};

/// Tunables for the short-horizon trajectory predictor.
struct PredictorConfig
{
  enum class Model { kConstantVelocity, kConstantAcceleration };

  /// CV is the conservative default: CVCA measurably wins on accelerating /
  /// turning agents but LOSES on oscillatory ones (weave), and until the N3
  /// conformal layer absorbs model misfit into the CBF radius, the safer
  /// model is the one without an extrapolated acceleration. Quantified in
  /// experiments/prediction/ade_fde_eval.py.
  Model model{Model::kConstantVelocity};
  int horizon_steps{15};        ///< K predicted positions...
  double horizon_dt{0.1};       ///< ...spaced this far apart (s). 15*0.1 = 1.5 s.
  double max_speed{2.0};        ///< clamp the fitted speed (m/s).
  double max_accel{2.0};        ///< clamp the fitted acceleration (m/s^2).
  double accel_damping{0.7};    ///< per-step decay of the accel term; a raw
                                ///< quadratic extrapolation diverges fast, and
                                ///< real agents do not accelerate forever.
};

/**
 * @class TrajectoryPredictor
 * @brief Short-horizon obstacle prediction from a track history (SE-Predict N2).
 *
 * Fits a motion model to the track history by LEAST SQUARES over the whole
 * history (not finite differences on the last points — under realistic
 * centroid jitter a 2-3-point difference turns the velocity, and especially
 * the acceleration, into amplified noise) and rolls it forward
 * `horizon_steps` x `horizon_dt`:
 *
 *  - kConstantVelocity:      linear fit; with exactly 2 points this equals
 *                            the legacy TrackedObstacle::predict path.
 *  - kConstantAcceleration:  quadratic fit (needs >= 5 points, else falls
 *                            back to linear), acceleration damped
 *                            geometrically per step. Measurably better on
 *                            accelerating/turning agents, WORSE on
 *                            oscillatory ones — see PredictorConfig::model.
 *
 * Degrades gracefully: < 2 points holds the current position. Math parity
 * with experiments/prediction/ade_fde_eval.py. The conformal error bound on
 * these horizons is milestone N3.
 */
class TrajectoryPredictor
{
public:
  void configure(const PredictorConfig & cfg) {cfg_ = cfg;}
  const PredictorConfig & config() const {return cfg_;}

  /**
   * @brief Predict future positions from a track history.
   * @param history Oldest-first timestamped positions of one obstacle.
   * @return horizon_steps positions at t = horizon_dt, 2*horizon_dt, ...
   */
  std::vector<Eigen::Vector2d> predict(
    const std::deque<TrackPoint> & history) const;

private:
  PredictorConfig cfg_{};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__TRAJECTORY_PREDICTOR_HPP_
