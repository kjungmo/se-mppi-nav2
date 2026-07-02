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

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"
#include "nav2_se_controller/trajectory_predictor.hpp"

using nav2_se_controller::DynamicObstacleTracker;
using nav2_se_controller::PredictorConfig;
using nav2_se_controller::TrackPoint;
using nav2_se_controller::TrajectoryPredictor;
using nav2_se_controller::TrackerConfig;

namespace
{
constexpr unsigned char kLethal = 254;

std::deque<TrackPoint> track(std::initializer_list<TrackPoint> pts)
{
  return std::deque<TrackPoint>(pts);
}

void fillBlock(
  nav2_costmap_2d::Costmap2D & map,
  unsigned int x0, unsigned int x1, unsigned int y0, unsigned int y1,
  unsigned char cost)
{
  for (unsigned int y = y0; y <= y1; ++y) {
    for (unsigned int x = x0; x <= x1; ++x) {
      map.setCost(x, y, cost);
    }
  }
}

double ade(
  const std::vector<Eigen::Vector2d> & pred,
  const std::vector<Eigen::Vector2d> & truth)
{
  double sum = 0.0;
  for (std::size_t k = 0; k < pred.size(); ++k) {
    sum += (pred[k] - truth[k]).norm();
  }
  return sum / static_cast<double>(pred.size());
}
}  // namespace

TEST(TrajectoryPredictor, CvHorizonMatchesLegacyPredict)
{
  TrajectoryPredictor p;
  PredictorConfig cfg;
  cfg.model = PredictorConfig::Model::kConstantVelocity;
  cfg.horizon_steps = 5;
  cfg.horizon_dt = 0.2;
  p.configure(cfg);

  // Constant velocity (1.0, 0.5) m/s.
  const auto h = p.predict(track({{0.0, {0.0, 0.0}}, {0.5, {0.5, 0.25}}}));
  ASSERT_EQ(h.size(), 5u);
  for (int k = 0; k < 5; ++k) {
    const double t = 0.2 * (k + 1);
    EXPECT_NEAR(h[k].x(), 0.5 + 1.0 * t, 1e-9);
    EXPECT_NEAR(h[k].y(), 0.25 + 0.5 * t, 1e-9);
  }
}

TEST(TrajectoryPredictor, EmptyAndSinglePointDegrade)
{
  TrajectoryPredictor p;
  p.configure(PredictorConfig{});
  EXPECT_TRUE(p.predict({}).empty());

  // One point: position held (zero velocity).
  const auto h = p.predict(track({{0.0, {1.0, 2.0}}}));
  ASSERT_FALSE(h.empty());
  for (const auto & q : h) {
    EXPECT_NEAR((q - Eigen::Vector2d(1.0, 2.0)).norm(), 0.0, 1e-9);
  }
}

TEST(TrajectoryPredictor, CvcaBeatsCvOnAcceleratingAgent)
{
  // Ground truth: constant acceleration a = (0.5, 0) from rest-ish. Six
  // history points: the quadratic fit requires >= 5 (noise-robustness guard).
  const double dt = 0.1;
  auto pos = [](double t) {
    return Eigen::Vector2d(0.2 * t + 0.25 * t * t, 0.0);
  };
  std::deque<TrackPoint> hist;
  for (int i = 0; i < 6; ++i) {
    hist.push_back({i * dt, pos(i * dt)});
  }
  const double t_last = 5 * dt;

  PredictorConfig base;
  base.horizon_steps = 10;
  base.horizon_dt = dt;
  base.accel_damping = 1.0;        // exact CA world: no damping needed

  TrajectoryPredictor cv;
  base.model = PredictorConfig::Model::kConstantVelocity;
  cv.configure(base);
  TrajectoryPredictor cvca;
  base.model = PredictorConfig::Model::kConstantAcceleration;
  cvca.configure(base);

  std::vector<Eigen::Vector2d> truth;
  for (int k = 1; k <= 10; ++k) {
    truth.push_back(pos(t_last + k * dt));
  }
  const double ade_cv = ade(cv.predict(hist), truth);
  const double ade_cvca = ade(cvca.predict(hist), truth);
  EXPECT_LT(ade_cvca, ade_cv * 0.5);   // decisively better, not marginally
}

TEST(TrajectoryPredictor, CvcaEqualsCvOnStraightLine)
{
  const double dt = 0.1;
  std::deque<TrackPoint> hist;
  for (int i = 0; i < 6; ++i) {
    hist.push_back({i * dt, {0.1 * i, 0.0}});
  }

  PredictorConfig base;
  base.horizon_steps = 8;
  base.horizon_dt = dt;
  TrajectoryPredictor cv;
  base.model = PredictorConfig::Model::kConstantVelocity;
  cv.configure(base);
  TrajectoryPredictor cvca;
  base.model = PredictorConfig::Model::kConstantAcceleration;
  cvca.configure(base);

  const auto hcv = cv.predict(hist);
  const auto hca = cvca.predict(hist);
  for (std::size_t k = 0; k < hcv.size(); ++k) {
    EXPECT_NEAR((hcv[k] - hca[k]).norm(), 0.0, 1e-9);
  }
}

TEST(TrajectoryPredictor, SpeedAndAccelClamped)
{
  TrajectoryPredictor p;
  PredictorConfig cfg;
  cfg.max_speed = 1.0;
  cfg.horizon_steps = 5;
  cfg.horizon_dt = 0.1;
  p.configure(cfg);

  // Implausible 10 m/s displacement: per-step motion must respect max_speed.
  const auto h = p.predict(track({{0.0, {0.0, 0.0}}, {0.1, {1.0, 0.0}}}));
  for (std::size_t k = 1; k < h.size(); ++k) {
    EXPECT_LE((h[k] - h[k - 1]).norm(), 1.0 * 0.1 + 1e-9);
  }
}

// ---------------------------------------------------------------------------
// Tracker integration (persistent tracks feeding the predictor).
// ---------------------------------------------------------------------------

TEST(TrackerN2, HorizonFilledForDynamicObstacle)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.predictor.horizon_steps = 10;
  cfg.predictor.horizon_dt = 0.1;
  tracker.configure(cfg);

  std::vector<nav2_se_controller::TrackedObstacle> obs;
  for (int f = 0; f < 4; ++f) {
    nav2_costmap_2d::Costmap2D map(40, 40, 0.1, 0.0, 0.0, 0);
    const auto x0 = static_cast<unsigned int>(5 + 2 * f);   // 0.2 m / 0.5 s
    fillBlock(map, x0, x0 + 1, 10, 11, kLethal);
    obs = tracker.update(map, 0.5 * f);
  }
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_TRUE(obs[0].is_dynamic);
  ASSERT_EQ(obs[0].horizon.size(), 10u);
  // The horizon must continue the rightward motion.
  EXPECT_GT(obs[0].horizon.back().x(), obs[0].position.x());
  EXPECT_EQ(tracker.trackCount(), 1u);
}

TEST(TrackerN2, StaticObstacleHasNoHorizon)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.static_min_frames = 3;
  tracker.configure(cfg);

  nav2_costmap_2d::Costmap2D wall(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(wall, 10, 11, 5, 24, kLethal);
  std::vector<nav2_se_controller::TrackedObstacle> obs;
  for (int f = 0; f < 5; ++f) {
    obs = tracker.update(wall, 0.1 * f);
  }
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_FALSE(obs[0].is_dynamic);
  EXPECT_TRUE(obs[0].horizon.empty());
}

TEST(TrackerN2, TrackSurvivesBriefMiss)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.max_missed_frames = 2;
  tracker.configure(cfg);

  nav2_costmap_2d::Costmap2D with(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(with, 10, 11, 10, 11, kLethal);
  nav2_costmap_2d::Costmap2D without(30, 30, 0.1, 0.0, 0.0, 0);

  tracker.update(with, 0.0);
  EXPECT_EQ(tracker.trackCount(), 1u);
  tracker.update(without, 0.1);            // miss 1 -> retained
  EXPECT_EQ(tracker.trackCount(), 1u);
  tracker.update(with, 0.2);               // re-acquired, same track
  EXPECT_EQ(tracker.trackCount(), 1u);

  // Three consecutive misses exceed max_missed_frames -> dropped.
  tracker.update(without, 0.3);
  tracker.update(without, 0.4);
  tracker.update(without, 0.5);
  EXPECT_EQ(tracker.trackCount(), 0u);
}

TEST(TrackerN2, HorizonDisabledLeavesLegacyOutput)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.predict_horizon = false;
  tracker.configure(cfg);

  nav2_costmap_2d::Costmap2D map(30, 30, 0.1, 0.0, 0.0, 0);
  fillBlock(map, 10, 11, 10, 11, kLethal);
  auto obs = tracker.update(map, 0.0);
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_TRUE(obs[0].horizon.empty());
}
