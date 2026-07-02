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

#ifndef NAV2_SE_CONTROLLER__DYNAMIC_OBSTACLE_TRACKER_HPP_
#define NAV2_SE_CONTROLLER__DYNAMIC_OBSTACLE_TRACKER_HPP_

#include <deque>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/cbf_types.hpp"
#include "nav2_se_controller/conformal_calibrator.hpp"
#include "nav2_se_controller/obstacle_clustering.hpp"
#include "nav2_se_controller/static_occupancy_filter.hpp"
#include "nav2_se_controller/trajectory_predictor.hpp"

namespace nav2_se_controller
{

/// Tunables for the dynamic obstacle tracker.
struct TrackerConfig
{
  unsigned char cost_threshold{253};  ///< occupied threshold (INSCRIBED_INFLATED_OBSTACLE).
  int min_cells{2};                   ///< drop clusters smaller than this.
  double association_gate{0.6};       ///< max centroid jump to associate across frames (m).
  double max_speed{2.0};              ///< clamp estimated speed (m/s); rejects bad matches.

  // SE-Predict N1: occupancy-persistence static/dynamic classification.
  // static_min_frames trades off how fast walls are recognized vs how slow an
  // obstacle may move before its cell dwell time builds static evidence
  // (~1 s at 10 Hz: an obstacle slower than ~radius/1s can misclassify; the
  // MPPI/costmap path still avoids it). fraction 0.5 keeps a wall static even
  // when a sensor reveal doubles its visible extent in one frame.
  bool classify_static{true};           ///< enable the static-evidence filter.
  int static_min_frames{10};            ///< frames of occupancy before a cell is static.
  double static_fraction_threshold{0.5};  ///< cluster static-cell fraction => static.

  // SE-Predict N2: persistent tracks + short-horizon prediction.
  bool predict_horizon{true};           ///< fill TrackedObstacle::horizon for dynamic tracks.
  int history_length{10};               ///< per-track position history (frames).
  int max_missed_frames{3};             ///< drop a track unmatched this long.
  PredictorConfig predictor{};          ///< CV / CVCA horizon settings.

  // SE-Predict N3: conformal calibration of the prediction error.
  bool conformal{true};                 ///< fill TrackedObstacle::q from residuals.
  ConformalConfig conformal_cfg{};      ///< coverage / learning rate / cap.
};

/**
 * @class DynamicObstacleTracker
 * @brief Costmap -> tracked obstacles with a constant-velocity estimate.
 *
 * Each update() clusters the occupied costmap cells (connected components),
 * associates clusters to the previous frame by nearest centroid within a gate,
 * and estimates each obstacle's velocity by finite difference. Unmatched
 * clusters are reported with zero velocity (newly seen). Feeds the CBF safety
 * filter and the escape-safety coordinator. See design §3.3.
 *
 * SE-Predict N1: in addition, an occupancy-persistence grid
 * (StaticOccupancyFilter) classifies each cluster static/dynamic. A cluster
 * whose cells have persistently been occupied is STATIC: is_dynamic = false
 * and its velocity is forced to zero, so an association-jitter "phantom
 * velocity" on a wall can never reach the CBF (the wall-freeze failure seen
 * in live runs). Genuinely moving obstacles sweep fresh cells, never build
 * per-cell evidence, and stay dynamic.
 *
 * SE-Predict N2: clusters are associated to PERSISTENT TRACKS (id + position
 * history surviving brief misses), and a TrajectoryPredictor fills each
 * dynamic obstacle's `horizon` from its track history.
 *
 * SE-Predict N3: every matched observation scores the track's PAST horizons
 * (residual at step k = distance between the observation and what was
 * predicted k steps ago), feeding a shared ConformalCalibrator whose
 * per-step bounds q_k are published on TrackedObstacle::q. The CBF inflates
 * its effective radius by q[0] (time-varying radius) and the coordinator
 * gates aggressive escape on prediction trust (max q).
 */
class DynamicObstacleTracker
{
public:
  void configure(const TrackerConfig & cfg)
  {
    cfg_ = cfg;
    StaticFilterConfig sf;
    sf.cost_threshold = cfg.cost_threshold;
    sf.static_min_frames = cfg.static_min_frames;
    static_filter_.configure(sf);
    predictor_.configure(cfg.predictor);
    calibrator_.configure(cfg.conformal_cfg, cfg.predictor.horizon_steps);
  }
  const TrackerConfig & config() const {return cfg_;}

  /**
   * @brief Process one costmap frame.
   * @param costmap Local costmap.
   * @param stamp Monotonic timestamp (s).
   * @return Tracked obstacles (position, CV velocity, radius).
   */
  std::vector<TrackedObstacle> update(
    const nav2_costmap_2d::Costmap2D & costmap, double stamp);

  void reset()
  {
    tracks_.clear();
    next_track_id_ = 0;
    static_filter_.reset();
  }

  /// Read access for tests/diagnostics.
  const StaticOccupancyFilter & staticFilter() const {return static_filter_;}
  const ConformalCalibrator & calibrator() const {return calibrator_;}
  std::size_t trackCount() const {return tracks_.size();}

private:
  /// One horizon a track predicted earlier, kept until its steps are scored.
  struct PastPrediction
  {
    double stamp{0.0};                       // when the prediction was made
    std::vector<Eigen::Vector2d> horizon;    // predicted positions
  };

  /// A persistent obstacle track (survives brief association misses).
  struct Track
  {
    int id{0};
    std::deque<TrackPoint> history;   // oldest-first, capped at history_length
    int missed{0};
    std::deque<PastPrediction> past;  // for conformal residual scoring
  };

  void scoreResiduals(
    Track & track, double stamp,
    const Eigen::Vector2d & observed);

  TrackerConfig cfg_{};
  std::vector<Track> tracks_;
  int next_track_id_{0};
  StaticOccupancyFilter static_filter_;
  TrajectoryPredictor predictor_;
  ConformalCalibrator calibrator_;
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__DYNAMIC_OBSTACLE_TRACKER_HPP_
