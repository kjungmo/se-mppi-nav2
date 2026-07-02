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
#include <random>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/cbf_safety_filter.hpp"
#include "nav2_se_controller/conformal_calibrator.hpp"
#include "nav2_se_controller/dynamic_obstacle_tracker.hpp"
#include "nav2_se_controller/escape_safety_coordinator.hpp"

using nav2_se_controller::CbfConfig;
using nav2_se_controller::CbfSafetyFilter;
using nav2_se_controller::ConformalCalibrator;
using nav2_se_controller::ConformalConfig;
using nav2_se_controller::CoordinationConfig;
using nav2_se_controller::DynamicObstacleTracker;
using nav2_se_controller::EscapeSafetyCoordinator;
using nav2_se_controller::RobotState;
using nav2_se_controller::TrackedObstacle;
using nav2_se_controller::TrackerConfig;

namespace
{
constexpr unsigned char kLethal = 254;

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
}  // namespace

TEST(ConformalCalibrator, ConvergesToTargetQuantile)
{
  ConformalCalibrator cal;
  ConformalConfig cfg;
  cfg.coverage = 0.9;
  cfg.learning_rate = 0.01;
  cfg.initial_q = 0.05;
  cfg.max_q = 1.0;
  cal.configure(cfg, 1);

  // |N(0, 0.1)| residuals: the 0.9-quantile is ~ 1.645 * 0.1 = 0.1645.
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.1);
  for (int i = 0; i < 6000; ++i) {
    cal.observe(0, std::abs(noise(rng)));
  }
  EXPECT_NEAR(cal.q(0), 0.1645, 0.03);

  // Empirical coverage on a fresh stream at the converged bound ~ 90 %.
  int hits = 0;
  const int n = 4000;
  for (int i = 0; i < n; ++i) {
    if (std::abs(noise(rng)) <= cal.q(0)) {
      ++hits;
    }
  }
  EXPECT_NEAR(static_cast<double>(hits) / n, 0.9, 0.04);
}

TEST(ConformalCalibrator, AdaptsToDistributionShift)
{
  ConformalCalibrator cal;
  ConformalConfig cfg;
  cfg.coverage = 0.9;
  cfg.learning_rate = 0.01;
  cfg.max_q = 2.0;
  cal.configure(cfg, 1);

  std::mt19937 rng(3);
  std::normal_distribution<double> small(0.0, 0.05);
  std::normal_distribution<double> large(0.0, 0.20);
  for (int i = 0; i < 4000; ++i) {
    cal.observe(0, std::abs(small(rng)));
  }
  const double q_small = cal.q(0);
  for (int i = 0; i < 6000; ++i) {
    cal.observe(0, std::abs(large(rng)));
  }
  const double q_large = cal.q(0);
  EXPECT_GT(q_large, 2.0 * q_small);  // tracked the 4x noise increase
}

TEST(ConformalCalibrator, CapAndPerStepIndependence)
{
  ConformalCalibrator cal;
  ConformalConfig cfg;
  cfg.max_q = 0.30;
  cfg.learning_rate = 0.05;
  cal.configure(cfg, 3);

  for (int i = 0; i < 2000; ++i) {
    cal.observe(2, 10.0);   // absurd residuals only on step 2
  }
  EXPECT_DOUBLE_EQ(cal.q(2), 0.30);          // capped (over-conservatism guard)
  EXPECT_DOUBLE_EQ(cal.q(0), cfg.initial_q);  // untouched steps stay at init
  EXPECT_EQ(cal.samples(0), 0);
  EXPECT_EQ(cal.samples(2), 2000);
}

// ---------------------------------------------------------------------------
// CBF consumption: q inflates the effective radius (time-varying radius).
// ---------------------------------------------------------------------------

TEST(ConformalCbf, BoundInflatesRadiusAndBrakesEarlier)
{
  CbfSafetyFilter f;
  CbfConfig cfg;
  cfg.alpha = 2.0;
  cfg.lookahead = 0.2;
  cfg.robot_radius = 0.22;
  cfg.safety_margin = 0.05;
  f.configure(cfg);
  const RobotState s{0.0, 0.0, 0.0};

  TrackedObstacle bare;
  bare.position = Eigen::Vector2d(1.0, 0.0);
  bare.velocity = Eigen::Vector2d(-0.3, 0.0);
  bare.radius = 0.2;

  TrackedObstacle bounded = bare;
  bounded.q = {0.25};                      // calibrated next-step bound

  const auto r_bare = f.filter(s, 0.5, 0.0, {bare});
  const auto r_bounded = f.filter(s, 0.5, 0.0, {bounded});
  ASSERT_TRUE(r_bare.feasible);
  ASSERT_TRUE(r_bounded.feasible);
  EXPECT_LT(r_bounded.v, r_bare.v - 0.02);  // q widens the certified margin
}

// ---------------------------------------------------------------------------
// Coordinator: two-variable coordination ("bold only when trusted").
// ---------------------------------------------------------------------------

TEST(ConformalCoordinator, EscapeAlphaGatedByPredictionTrust)
{
  EscapeSafetyCoordinator c;
  CoordinationConfig cfg;
  cfg.alpha_base = 2.0;
  cfg.alpha_escape = 6.0;
  cfg.ttc_override_threshold = 1.5;
  cfg.q_trust_threshold = 0.25;
  c.configure(cfg);

  // Entrapped, TTC safe, prediction trusted -> bold escape.
  EXPECT_DOUBLE_EQ(c.alpha(true, 10.0, 0.10), 6.0);
  // Entrapped, TTC safe, prediction NOT trusted -> cautious escape (the
  // q-inflated radius is already widening the margin).
  EXPECT_DOUBLE_EQ(c.alpha(true, 10.0, 0.40), 2.0);
  // TTC override still beats everything.
  EXPECT_DOUBLE_EQ(c.alpha(true, 0.5, 0.10), 2.0);
  // Legacy callers (no q argument) behave as before.
  EXPECT_DOUBLE_EQ(c.alpha(true, 10.0), 6.0);
  EXPECT_DOUBLE_EQ(c.alpha(false, 10.0, 0.40), 2.0);
}

// ---------------------------------------------------------------------------
// Tracker end-to-end: residual scoring fills q on dynamic obstacles.
// ---------------------------------------------------------------------------

TEST(ConformalTracker, QFilledAndSmallForConstantVelocityAgent)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.predictor.horizon_steps = 5;
  cfg.predictor.horizon_dt = 0.1;
  cfg.conformal_cfg.initial_q = 0.05;
  cfg.conformal_cfg.learning_rate = 0.01;
  tracker.configure(cfg);

  // A blob moving exactly one cell (0.1 m) per 0.1 s frame: CV predictions
  // are near-perfect (centroid quantization only), so q must stay small.
  std::vector<TrackedObstacle> obs;
  for (int f = 0; f < 30; ++f) {
    nav2_costmap_2d::Costmap2D map(60, 60, 0.1, 0.0, 0.0, 0);
    const auto x0 = static_cast<unsigned int>(5 + f);
    fillBlock(map, x0, x0 + 1, 10, 11, kLethal);
    obs = tracker.update(map, 0.1 * f);
  }
  ASSERT_EQ(obs.size(), 1u);
  ASSERT_EQ(obs[0].q.size(), 5u);
  EXPECT_GT(tracker.calibrator().samples(0), 10);
  // Near-exact predictions: the bound shrinks from (or stays near) its init.
  EXPECT_LE(obs[0].q.front(), cfg.conformal_cfg.initial_q + 1e-9);
}

TEST(ConformalTracker, DisabledLeavesQEmpty)
{
  DynamicObstacleTracker tracker;
  TrackerConfig cfg;
  cfg.conformal = false;
  tracker.configure(cfg);

  std::vector<TrackedObstacle> obs;
  for (int f = 0; f < 6; ++f) {
    nav2_costmap_2d::Costmap2D map(40, 40, 0.1, 0.0, 0.0, 0);
    const auto x0 = static_cast<unsigned int>(5 + f);
    fillBlock(map, x0, x0 + 1, 10, 11, kLethal);
    obs = tracker.update(map, 0.1 * f);
  }
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_TRUE(obs[0].q.empty());
  EXPECT_EQ(tracker.calibrator().samples(0), 0);
}
