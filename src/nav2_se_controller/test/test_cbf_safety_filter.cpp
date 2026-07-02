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

#include <vector>

#include "nav2_se_controller/cbf_safety_filter.hpp"

using nav2_se_controller::CbfConfig;
using nav2_se_controller::CbfSafetyFilter;
using nav2_se_controller::RobotState;
using nav2_se_controller::TrackedObstacle;

namespace
{
CbfConfig makeConfig()
{
  CbfConfig c;
  c.alpha = 2.0;
  c.lookahead = 0.2;
  c.robot_radius = 0.22;
  c.safety_margin = 0.05;
  c.v_min = -0.35;
  c.v_max = 0.5;
  c.w_max = 1.9;
  c.slack_weight = 1.0e3;
  return c;
}

TrackedObstacle makeObstacle(double x, double y, double r)
{
  TrackedObstacle o;
  o.position = Eigen::Vector2d(x, y);
  o.velocity = Eigen::Vector2d(0.0, 0.0);
  o.radius = r;
  return o;
}
}  // namespace

TEST(CbfSafetyFilter, NoObstacleReturnsNominal)
{
  CbfSafetyFilter f;
  f.configure(makeConfig());
  auto r = f.filter(RobotState{0.0, 0.0, 0.0}, 0.4, 0.1, {});
  EXPECT_TRUE(r.feasible);
  EXPECT_FALSE(r.modified);
  EXPECT_NEAR(r.v, 0.4, 1e-6);
  EXPECT_NEAR(r.w, 0.1, 1e-6);
}

TEST(CbfSafetyFilter, NominalClampedToBox)
{
  CbfSafetyFilter f;
  f.configure(makeConfig());
  auto r = f.filter(RobotState{0.0, 0.0, 0.0}, 5.0, 0.0, {});  // v_nom > v_max
  EXPECT_NEAR(r.v, 0.5, 1e-6);
}

TEST(CbfSafetyFilter, FarObstacleLeavesControlNearNominal)
{
  CbfSafetyFilter f;
  f.configure(makeConfig());
  std::vector<TrackedObstacle> obs = {makeObstacle(10.0, 0.0, 0.1)};
  auto r = f.filter(RobotState{0.0, 0.0, 0.0}, 0.4, 0.0, obs);
  EXPECT_TRUE(r.feasible);
  EXPECT_NEAR(r.v, 0.4, 1e-2);
}

TEST(CbfSafetyFilter, ObstacleAheadIsMadeSafe)
{
  CbfSafetyFilter f;
  const CbfConfig cfg = makeConfig();
  f.configure(cfg);
  std::vector<TrackedObstacle> obs = {makeObstacle(0.8, 0.0, 0.1)};
  const RobotState s{0.0, 0.0, 0.0};

  // Nominal drives straight into the obstacle.
  const double nominal_residual = f.barrierResidual(s, 0.5, 0.0, obs[0], cfg.alpha);
  ASSERT_LT(nominal_residual, 0.0);  // nominal violates the CBF condition

  auto r = f.filter(s, 0.5, 0.0, obs);
  EXPECT_TRUE(r.feasible);
  EXPECT_TRUE(r.modified);
  EXPECT_LT(r.v, 0.5);  // forward motion reduced

  // Filtered control must satisfy the barrier condition.
  const double res = f.barrierResidual(s, r.v, r.w, obs[0], cfg.alpha);
  EXPECT_GE(res, -1e-3);
}

TEST(CbfSafetyFilter, ObstacleBehindDoesNotBlockForwardMotion)
{
  CbfSafetyFilter f;
  f.configure(makeConfig());
  std::vector<TrackedObstacle> obs = {makeObstacle(-0.8, 0.0, 0.1)};
  auto r = f.filter(RobotState{0.0, 0.0, 0.0}, 0.5, 0.0, obs);
  EXPECT_TRUE(r.feasible);
  EXPECT_NEAR(r.v, 0.5, 1e-2);
}

// ---------------------------------------------------------------------------
// Responsibility allocation (Multi-SE-MPPI N2): reciprocal robot constraints.
// ---------------------------------------------------------------------------

TEST(CbfResponsibility, ReciprocalShareIsLessConservativeThanFull)
{
  // A neighbor robot closing head-on. The single-robot constraint (lambda=1,
  // reactive velocity term) must brake harder than the reciprocal half-share
  // (the neighbor's own filter handles its share).
  CbfSafetyFilter f;
  CbfConfig cfg = makeConfig();
  cfg.emergency_dist = 0.0;   // isolate the share semantics from the override
  f.configure(cfg);
  const RobotState s{0.0, 0.0, 0.0};

  TrackedObstacle full = makeObstacle(1.1, 0.0, 0.22);
  full.velocity = Eigen::Vector2d(-0.4, 0.0);   // closing
  full.responsibility = 1.0;

  TrackedObstacle recip = full;
  recip.responsibility = 0.5;

  const auto r_full = f.filter(s, 0.5, 0.0, {full});
  const auto r_recip = f.filter(s, 0.5, 0.0, {recip});
  ASSERT_TRUE(r_full.feasible);
  ASSERT_TRUE(r_recip.feasible);
  EXPECT_GT(r_recip.v, r_full.v + 0.02);  // measurably less double-braking
}

TEST(CbfResponsibility, EmergencyBandRestoresFullConstraint)
{
  // Inside the emergency band a reciprocal obstacle is treated with the full
  // reactive constraint (safety beats protocol). A large budget share
  // (lambda = 0.9, a "passer") is permissive on its own; the band must
  // override it.
  CbfSafetyFilter f;
  CbfConfig cfg = makeConfig();
  cfg.emergency_dist = 0.40;
  f.configure(cfg);
  const RobotState s{0.0, 0.0, 0.0};

  // Surface gap to the look-ahead point ~ 0.31 m < 0.40 -> emergency.
  TrackedObstacle o = makeObstacle(1.0, 0.0, 0.22);
  o.velocity = Eigen::Vector2d(-0.4, 0.0);
  o.responsibility = 0.9;     // permissive budget without the band
  const auto r_band = f.filter(s, 0.5, 0.0, {o});

  CbfConfig no_band = cfg;
  no_band.emergency_dist = 0.0;
  CbfSafetyFilter f2;
  f2.configure(no_band);
  const auto r_free = f2.filter(s, 0.5, 0.0, {o});

  ASSERT_TRUE(r_band.feasible);
  ASSERT_TRUE(r_free.feasible);
  EXPECT_LT(r_band.v, r_free.v - 0.02);  // band is strictly more conservative
}

TEST(CbfResponsibility, ReciprocalPairKeepsJointBarrier)
{
  // Two robots running the SAME protocol head-on, each filtering with the
  // other as a lambda=0.5 reciprocal obstacle and nominal commands that drive
  // straight at each other. Rolling both forward, the pair must stay
  // separated: each enforces its share, the sum restores hdot + alpha h >= 0.
  CbfSafetyFilter f;
  CbfConfig cfg = makeConfig();
  f.configure(cfg);

  RobotState a{0.0, 0.0, 0.0};
  RobotState b{2.4, 0.0, M_PI};
  double va = 0.0;
  double vb = 0.0;
  const double dt = 0.05;
  const double eff_r = 2 * 0.22 + cfg.safety_margin;
  double min_gap = 1.0e9;

  for (int k = 0; k < 240; ++k) {
    TrackedObstacle oa = makeObstacle(b.x, b.y, 0.22);   // B as seen by A
    oa.velocity = Eigen::Vector2d(vb * std::cos(b.yaw), vb * std::sin(b.yaw));
    oa.responsibility = 0.5;
    TrackedObstacle ob = makeObstacle(a.x, a.y, 0.22);   // A as seen by B
    ob.velocity = Eigen::Vector2d(va * std::cos(a.yaw), va * std::sin(a.yaw));
    ob.responsibility = 0.5;

    const auto ra = f.filter(a, 0.5, 0.0, {oa});
    const auto rb = f.filter(b, 0.5, 0.0, {ob});
    va = ra.hard_safe ? ra.v : 0.0;
    vb = rb.hard_safe ? rb.v : 0.0;

    a.x += va * std::cos(a.yaw) * dt;
    a.y += va * std::sin(a.yaw) * dt;
    a.yaw += ra.w * dt;
    b.x += vb * std::cos(b.yaw) * dt;
    b.y += vb * std::sin(b.yaw) * dt;
    b.yaw += rb.w * dt;

    const double gap =
      std::hypot(a.x - b.x, a.y - b.y) - eff_r;
    min_gap = std::min(min_gap, gap);
  }
  // The pair never violates the joint barrier (centre distance >= eff_r).
  EXPECT_GE(min_gap, -1e-3);
}

TEST(CbfResponsibility, DefaultResponsibilityKeepsLegacyBehavior)
{
  // responsibility defaults to 1.0: identical to the pre-N2 constraint.
  CbfSafetyFilter f;
  f.configure(makeConfig());
  const RobotState s{0.0, 0.0, 0.0};
  TrackedObstacle o = makeObstacle(0.9, 0.0, 0.15);
  o.velocity = Eigen::Vector2d(-0.3, 0.0);

  TrackedObstacle o_explicit = o;
  o_explicit.responsibility = 1.0;

  const auto r1 = f.filter(s, 0.5, 0.0, {o});
  const auto r2 = f.filter(s, 0.5, 0.0, {o_explicit});
  EXPECT_NEAR(r1.v, r2.v, 1e-9);
  EXPECT_NEAR(r1.w, r2.w, 1e-9);
}
