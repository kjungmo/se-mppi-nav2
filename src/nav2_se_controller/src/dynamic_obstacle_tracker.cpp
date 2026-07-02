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

#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace nav2_se_controller
{

std::vector<TrackedObstacle> DynamicObstacleTracker::update(
  const nav2_costmap_2d::Costmap2D & costmap, double stamp)
{
  const std::vector<ObstacleCluster> clusters =
    clusterObstacles(costmap, cfg_.cost_threshold, cfg_.min_cells);

  // Fold this frame into the static-evidence grid BEFORE classifying, so a
  // wall observed for static_min_frames is static from that frame onward.
  if (cfg_.classify_static) {
    static_filter_.update(costmap);
  }

  // Associate clusters to persistent tracks: greedy nearest-track within the
  // gate, each track consumable once — a split/merge cannot give two current
  // clusters the same (phantom) track velocity.
  std::vector<char> consumed(tracks_.size(), 0);
  std::vector<int> match_of(clusters.size(), -1);
  const double gate2 = cfg_.association_gate * cfg_.association_gate;

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    double best_dist2 = gate2;
    int match = -1;
    for (std::size_t j = 0; j < tracks_.size(); ++j) {
      if (consumed[j]) {
        continue;
      }
      const double d2 =
        (clusters[i].centroid - tracks_[j].history.back().position)
        .squaredNorm();
      if (d2 <= best_dist2) {
        best_dist2 = d2;
        match = static_cast<int>(j);
      }
    }
    if (match >= 0) {
      consumed[match] = 1;
      match_of[i] = match;
    }
  }

  std::vector<TrackedObstacle> obstacles;
  obstacles.reserve(clusters.size());

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const auto & c = clusters[i];
    TrackedObstacle obs;
    obs.position = c.centroid;
    obs.radius = c.radius;
    obs.velocity = Eigen::Vector2d::Zero();

    Track * track = nullptr;
    if (match_of[i] >= 0) {
      track = &tracks_[match_of[i]];
      track->missed = 0;
      // Guard a non-advancing clock (clock-lock failure upstream): keep the
      // history sane rather than dividing by ~0 later.
      if (stamp - track->history.back().stamp > 1.0e-6) {
        track->history.push_back({stamp, c.centroid});
        while (track->history.size() >
          static_cast<std::size_t>(std::max(2, cfg_.history_length)))
        {
          track->history.pop_front();
        }
      }
    } else {
      tracks_.push_back(Track{next_track_id_++, {{stamp, c.centroid}}, 0});
      track = &tracks_.back();
      // Re-fetch consumed bookkeeping is unnecessary: new tracks can't be
      // claimed by later clusters this frame (they are appended after the
      // association pass).
    }

    // Velocity from the last two history points (the legacy finite-difference
    // estimate, now track-based so it survives a brief association miss).
    const auto & h = track->history;
    if (h.size() >= 2) {
      const double dt = h.back().stamp - h[h.size() - 2].stamp;
      if (dt > 1.0e-6) {
        Eigen::Vector2d vel =
          (h.back().position - h[h.size() - 2].position) / dt;
        const double speed = vel.norm();
        if (speed > cfg_.max_speed) {
          vel *= cfg_.max_speed / speed;  // clamp implausible velocities
        }
        obs.velocity = vel;
      }
    }

    // SE-Predict N1: a cluster sitting (mostly) on persistently-occupied cells
    // is static structure. Zero its velocity so association jitter cannot hand
    // the CBF a "moving wall" (the live-run wall-freeze failure).
    if (cfg_.classify_static &&
      static_filter_.staticFraction(c.cells) >= cfg_.static_fraction_threshold)
    {
      obs.is_dynamic = false;
      obs.velocity = Eigen::Vector2d::Zero();
    }

    // SE-Predict N3: score past predictions against this observation (online
    // conformal residuals), BEFORE issuing the new prediction.
    if (cfg_.conformal && match_of[i] >= 0) {
      scoreResiduals(*track, stamp, c.centroid);
    }

    // SE-Predict N2: short-horizon prediction from the track history, for
    // dynamic obstacles only (static structure needs no forecast).
    if (cfg_.predict_horizon && obs.is_dynamic) {
      obs.horizon = predictor_.predict(track->history);
      if (cfg_.conformal && !obs.horizon.empty()) {
        track->past.push_back({stamp, obs.horizon});
        // Keep only predictions whose horizon can still be scored.
        const double window =
          cfg_.predictor.horizon_steps * cfg_.predictor.horizon_dt;
        while (!track->past.empty() &&
          stamp - track->past.front().stamp > window + 1.0e-6)
        {
          track->past.pop_front();
        }
        obs.q = calibrator_.qAll();   // shared, pooled-across-tracks bounds
      }
    }

    obstacles.push_back(std::move(obs));
  }

  // Age out tracks that went unmatched too long (left the window / merged).
  // Tracks beyond consumed.size() were created this frame — never aged here.
  for (std::size_t j = 0; j < tracks_.size(); ) {
    if (j < consumed.size() && !consumed[j]) {
      if (++tracks_[j].missed > cfg_.max_missed_frames) {
        tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(j));
        consumed.erase(consumed.begin() + static_cast<std::ptrdiff_t>(j));
        continue;
      }
    }
    ++j;
  }

  return obstacles;
}

void DynamicObstacleTracker::scoreResiduals(
  Track & track, double stamp, const Eigen::Vector2d & observed)
{
  // Each stored horizon predicted positions at past.stamp + (k+1)*dt. When
  // `stamp` lands on one of those instants (within half a step), the distance
  // between the observation and that prediction is one conformal residual for
  // step k. As frames advance, every horizon is scored once per step index.
  const double dt = cfg_.predictor.horizon_dt;
  if (dt <= 1.0e-9) {
    return;
  }
  for (const auto & past : track.past) {
    const double k_float = (stamp - past.stamp) / dt - 1.0;
    const int k = static_cast<int>(std::lround(k_float));
    if (k < 0 || k >= static_cast<int>(past.horizon.size()) ||
      std::abs(k_float - k) > 0.25)
    {
      continue;
    }
    calibrator_.observe(k, (observed - past.horizon[k]).norm());
  }
}

}  // namespace nav2_se_controller
