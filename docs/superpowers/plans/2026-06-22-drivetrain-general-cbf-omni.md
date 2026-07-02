# Drivetrain-General CBF (J,U) Abstraction + Omnidirectional — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generalize the `CbfSafetyFilter` from differential-drive-only to a drivetrain-parameterized `(J, U)` form, and add the omnidirectional (holonomic) instantiation — without changing differential-drive behavior.

**Architecture:** Introduce a `Drivetrain` enum and a small internal *plant* (`(safety_point, B-matrix, control count, box limits, tracking weights)`) that the QP is assembled from generically. The differential path becomes one instantiation of that plant (behavior-identical, regression-locked by the existing tests); omnidirectional is a second instantiation where the base position is directly relative-degree-1, the translational pair `(vx, vy)` enters the barrier through `R(θ)`, and `ω` decouples from the positional barrier. This is contribution **D1** (generalization) of the Drivetrain-General SE-MPPI spec, plus the omni half of **D3**.

**Tech Stack:** C++17, Eigen3, OSQP via `osqp-eigen`, GoogleTest (`ament_cmake_gtest`), ROS 2 Jazzy + Nav2 (RoboStack), colcon.

**Spec:** `docs/superpowers/specs/2026-06-22-drivetrain-general-se-mppi-design.md` (§3.3 the `(J,U)` abstraction; §3.4 D1/D3; §3.7 implementation).

## Global Constraints

- ROS 2 **Jazzy** + Nav2; build against the **installed Jazzy headers** (do not guess APIs).
- The **differential-drive path must stay behavior-identical** — every existing test in `test/test_cbf_safety_filter.cpp` must keep passing **unedited** (regression lock).
- Pure-core only: `se_mppi_core` library has **no ROS/costmap dependency** in `cbf_safety_filter.*`; keep it that way.
- New files carry the **Apache-2.0 license header** (copy the 13-line header from `src/cbf_safety_filter.cpp`). No "Generated with Claude Code" / Co-Authored-By footers.
- Build/test command (env per `CLAUDE.md`):
  ```bash
  export MAMBA_ROOT_PREFIX=/opt/micromamba
  micromamba run -n ros2 bash -lc 'cd /home/cona/kangj/se-mppi-nav2 && \
    colcon build --packages-select nav2_se_controller && \
    colcon test --packages-select nav2_se_controller --event-handlers console_direct+'
  ```
  (Local non-root prefix may be `$HOME/micromamba` — use whatever `scripts/setup_ros2_env.sh` printed.)
- Run a single gtest case during TDD:
  ```bash
  micromamba run -n ros2 bash -lc 'cd /home/cona/kangj/se-mppi-nav2 && \
    ./build/nav2_se_controller/test_nav2_se_controller --gtest_filter=<Suite.Case>'
  ```
- Work on branch `claude/fervent-newton-lbo96`; **do not push** unless asked.
- Git commit trailer (per repo policy): end each commit message with
  `Claude-Session: https://claude.ai/code/session_01QK1AeEbWiAfk1EKezRWnqu`.

## Out of scope (separate plans)

Ackermann / D2 (curvature cone + reverse escape + switching proof); omni/Ackermann Gazebo models; cross-drivetrain eval campaign; the paper write-up. This plan delivers the omni-capable core library + its controller param, unit-tested.

## File Structure

- `include/nav2_se_controller/cbf_types.hpp` — **modify**: add `enum class Drivetrain`; add `drivetrain`, `vy_max`, `w_vy` to `CbfConfig`.
- `include/nav2_se_controller/cbf_safety_filter.hpp` — **modify**: add `vy` to `Result`; add omni overloads of `filter` and `barrierResidual`; update class doc.
- `src/cbf_safety_filter.cpp` — **modify**: replace the differential-specific internals with the generic `(J,U)` plant + `solveQp`/`residualImpl`; re-express the public differential methods on top; add the omni public methods.
- `test/test_cbf_safety_filter.cpp` — **modify**: append config/Result default tests (Task 1) and the omni test suite (Task 3). Existing tests stay unedited.
- `src/safe_escape_controller.cpp` — **modify** (Task 4): read `se_drivetrain` + `se_cbf_vy_max` params into `CbfConfig`.
- `config/nav2_se_controller_params.yaml` — **modify** (Task 4): document the two new keys.

---

### Task 1: Drivetrain enum + config/result fields

**Files:**
- Modify: `include/nav2_se_controller/cbf_types.hpp`
- Modify: `include/nav2_se_controller/cbf_safety_filter.hpp:51-59` (`Result` struct)
- Test: `test/test_cbf_safety_filter.cpp`

**Interfaces:**
- Produces: `enum class Drivetrain { Differential, Omnidirectional };`;
  `CbfConfig.drivetrain` (default `Differential`), `CbfConfig.vy_max` (default `0.0`),
  `CbfConfig.w_vy` (default `1.0`); `CbfSafetyFilter::Result.vy` (default `0.0`).

- [ ] **Step 1: Write the failing tests**

Append to `test/test_cbf_safety_filter.cpp` (and add `using nav2_se_controller::Drivetrain;` next to the other `using` lines):

```cpp
TEST(CbfConfigDefaults, DrivetrainDefaultsToDifferential)
{
  CbfConfig c;
  EXPECT_EQ(c.drivetrain, Drivetrain::Differential);
  EXPECT_DOUBLE_EQ(c.vy_max, 0.0);
  EXPECT_DOUBLE_EQ(c.w_vy, 1.0);
}

TEST(CbfResultDefaults, VyDefaultsToZero)
{
  CbfSafetyFilter::Result r;
  EXPECT_DOUBLE_EQ(r.vy, 0.0);
}
```

- [ ] **Step 2: Run tests to verify they fail (compile error)**

Run: `micromamba run -n ros2 bash -lc 'cd /home/cona/kangj/se-mppi-nav2 && colcon build --packages-select nav2_se_controller'`
Expected: FAIL — `Drivetrain` not declared / no member `vy`/`vy_max`/`w_vy`.

- [ ] **Step 3: Add the enum and fields**

In `cbf_types.hpp`, immediately inside `namespace nav2_se_controller {` (before `struct RobotState`):

```cpp
/// Wheeled-mobile-robot mobility class the CBF/escape is instantiated for.
/// Differential: control (v, w), barrier on a look-ahead point.
/// Omnidirectional: control (vx, vy, w), barrier on the base point; w decoupled.
enum class Drivetrain
{
  Differential,
  Omnidirectional
};
```

In `cbf_types.hpp`, add to `struct CbfConfig` (after `int max_obstacles{20};`):

```cpp
  Drivetrain drivetrain{Drivetrain::Differential};  ///< mobility class.
  double vy_max{0.0};            ///< |vy| limit (m/s) for omni; 0 disables lateral motion.
  double w_vy{1.0};              ///< tracking weight on lateral velocity (omni).
```

In `cbf_safety_filter.hpp`, add to `struct Result` (after `double w{0.0};`):

```cpp
    double vy{0.0};         ///< lateral velocity (omnidirectional; 0 for differential).
```

- [ ] **Step 4: Run tests to verify they pass**

Run: build, then `./build/nav2_se_controller/test_nav2_se_controller --gtest_filter='CbfConfigDefaults.*:CbfResultDefaults.*'`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add include/nav2_se_controller/cbf_types.hpp include/nav2_se_controller/cbf_safety_filter.hpp test/test_cbf_safety_filter.cpp
git commit -m "feat(cbf): add Drivetrain enum + omni config/result fields

Claude-Session: https://claude.ai/code/session_01QK1AeEbWiAfk1EKezRWnqu"
```

---

### Task 2: `(J,U)` plant abstraction; refactor differential path (regression-only)

**Files:**
- Modify: `src/cbf_safety_filter.cpp` (replace the anonymous-namespace helpers + both public methods)
- Test: `test/test_cbf_safety_filter.cpp` (no new tests — the existing suite is the regression gate)

**Interfaces:**
- Consumes: `CbfConfig` (Task 1), `RobotState`, `TrackedObstacle`.
- Produces (internal, file-local): `struct CbfPlant`; `CbfPlant buildPlant(const RobotState&, const CbfConfig&)`; `double residualImpl(const CbfConfig&, const RobotState&, const std::array<double,3>&, const TrackedObstacle&, double)`; `struct QpOut`; `QpOut solveQp(const CbfConfig&, const RobotState&, const std::array<double,3>&, const std::vector<TrackedObstacle>&, double)`. Public signatures of `filter(state,v,w,obs,alpha)` and `barrierResidual(state,v,w,o,alpha)` unchanged.

- [ ] **Step 1: Confirm the regression gate is green before refactor**

Run: `./build/nav2_se_controller/test_nav2_se_controller --gtest_filter='CbfSafetyFilter.*:CbfResponsibility.*'`
Expected: PASS (all existing CBF tests). This is the behavior we must preserve.

- [ ] **Step 2: Replace the file body with the generic plant implementation**

Replace everything in `src/cbf_safety_filter.cpp` **from the `#include` block's end through the closing `}  // namespace`** with the following (keep the 13-line Apache header at top). Add `#include <array>` to the includes.

```cpp
#include "nav2_se_controller/cbf_safety_filter.hpp"

#include <algorithm>
#include <array>
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

/// Drivetrain-parameterized plant the CBF-QP is assembled from.
///   pdot(safety_point) = bmat.leftCols(n_ctrl) * u
/// Differential: safety_point = look-ahead point, u = [v, w] (2 ctrl).
/// Omnidirectional: safety_point = base, u = [vx, vy, w] (3 ctrl); w column = 0.
struct CbfPlant
{
  Eigen::Vector2d safety_point;
  Eigen::Matrix<double, 2, 3> bmat;
  int n_ctrl{2};
  std::array<double, 3> u_lo{{0.0, 0.0, 0.0}};
  std::array<double, 3> u_hi{{0.0, 0.0, 0.0}};
  std::array<double, 3> w_track{{0.0, 0.0, 0.0}};
};

CbfPlant buildPlant(const RobotState & state, const CbfConfig & cfg)
{
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  CbfPlant p;
  if (cfg.drivetrain == Drivetrain::Omnidirectional) {
    // Holonomic: the base point is fully actuated by (vx, vy) in body frame;
    // pdot = R(yaw) [vx, vy]. w is decoupled from the positional barrier.
    p.safety_point = Eigen::Vector2d(state.x, state.y);
    p.bmat << c, -s, 0.0,
      s, c, 0.0;
    p.n_ctrl = 3;
    p.u_lo = {cfg.v_min, -cfg.vy_max, -cfg.w_max};
    p.u_hi = {cfg.v_max, cfg.vy_max, cfg.w_max};
    p.w_track = {cfg.w_v, cfg.w_vy, cfg.w_w};
  } else {
    // Differential: look-ahead point makes the barrier relative-degree one in
    // (v, w); pdot = G [v, w], G = [[c, -L s], [s, L c]].
    p.safety_point = Eigen::Vector2d(
      state.x + cfg.lookahead * c, state.y + cfg.lookahead * s);
    p.bmat << c, -cfg.lookahead * s, 0.0,
      s, cfg.lookahead * c, 0.0;
    p.n_ctrl = 2;
    p.u_lo = {cfg.v_min, -cfg.w_max, 0.0};
    p.u_hi = {cfg.v_max, cfg.w_max, 0.0};
    p.w_track = {cfg.w_v, cfg.w_w, 0.0};
  }
  return p;
}

/// Barrier residual  d/dt h + alpha h  for control u (drivetrain ordering).
double residualImpl(
  const CbfConfig & cfg, const RobotState & state,
  const std::array<double, 3> & u, const TrackedObstacle & o, double alpha)
{
  const CbfPlant plant = buildPlant(state, cfg);
  const Eigen::Vector2d d = plant.safety_point - o.position;
  const double q0 = o.q.empty() ? 0.0 : o.q.front();
  const double eff_r = cfg.robot_radius + o.radius + cfg.safety_margin + q0;
  const double h = d.squaredNorm() - eff_r * eff_r;
  Eigen::Vector2d pdot = Eigen::Vector2d::Zero();
  for (int j = 0; j < plant.n_ctrl; ++j) {
    pdot += plant.bmat.col(j) * u[j];
  }
  const double hdot = 2.0 * d.dot(pdot - o.velocity);
  return hdot + alpha * h;
}

struct QpOut
{
  std::array<double, 3> u{{0.0, 0.0, 0.0}};
  double slack{0.0};
  bool feasible{true};
  bool hard_safe{true};
  bool modified{false};
};

QpOut solveQp(
  const CbfConfig & cfg, const RobotState & state,
  const std::array<double, 3> & u_nom,
  const std::vector<TrackedObstacle> & obstacles, double alpha)
{
  const CbfPlant plant = buildPlant(state, cfg);
  const int n = plant.n_ctrl;

  std::array<double, 3> u_cl{{0.0, 0.0, 0.0}};
  for (int i = 0; i < n; ++i) {
    u_cl[i] = std::clamp(u_nom[i], plant.u_lo[i], plant.u_hi[i]);
  }

  QpOut fb;
  fb.u = u_cl;
  fb.feasible = true;
  fb.hard_safe = true;
  fb.modified = false;
  if (obstacles.empty()) {
    return fb;
  }

  // Keep the nearest max_obstacles by CLEARANCE to the safety point.
  const Eigen::Vector2d ps = plant.safety_point;
  std::vector<const TrackedObstacle *> obs;
  obs.reserve(obstacles.size());
  for (const auto & o : obstacles) {
    obs.push_back(&o);
  }
  const std::size_t n_obs =
    std::min<std::size_t>(obs.size(), static_cast<std::size_t>(std::max(cfg.max_obstacles, 1)));
  const double base_clearance = cfg.robot_radius + cfg.safety_margin;
  std::partial_sort(
    obs.begin(), obs.begin() + n_obs, obs.end(),
    [&ps, base_clearance](const TrackedObstacle * a, const TrackedObstacle * b) {
      const double ca = (a->position - ps).norm() - a->radius - base_clearance;
      const double cb = (b->position - ps).norm() - b->radius - base_clearance;
      return ca < cb;
    });
  obs.resize(n_obs);

  const int n_vars = n + 1;                       // controls + slack
  const int n_cons = static_cast<int>(n_obs) + n_vars;  // CBF rows + boxes

  Eigen::SparseMatrix<double> hessian(n_vars, n_vars);
  for (int i = 0; i < n; ++i) {
    hessian.insert(i, i) = plant.w_track[i];
  }
  hessian.insert(n, n) = cfg.slack_weight;
  hessian.makeCompressed();

  Eigen::VectorXd gradient(n_vars);
  gradient.setZero();
  for (int i = 0; i < n; ++i) {
    gradient(i) = -plant.w_track[i] * u_cl[i];
  }

  Eigen::SparseMatrix<double> constraints(n_cons, n_vars);
  Eigen::VectorXd lower(n_cons);
  Eigen::VectorXd upper(n_cons);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(n_obs) * (n + 1) + n_vars);

  for (std::size_t i = 0; i < n_obs; ++i) {
    const TrackedObstacle & o = *obs[i];
    const Eigen::Vector2d d = ps - o.position;
    const double q0 = o.q.empty() ? 0.0 : o.q.front();
    const double eff_r = cfg.robot_radius + o.radius + cfg.safety_margin + q0;
    const double h = d.squaredNorm() - eff_r * eff_r;

    double lambda = std::clamp(o.responsibility, 0.0, 1.0);
    const double gap = d.norm() - eff_r;
    if (gap < cfg.emergency_dist) {
      lambda = 1.0;  // safety beats protocol when the pair gets critical
    }
    const double b = lambda >= 1.0 - 1.0e-9 ?
      -alpha * h + 2.0 * d.dot(o.velocity) :
      -lambda * alpha * h;

    const int row = static_cast<int>(i);
    for (int j = 0; j < n; ++j) {
      const double coeff = 2.0 * d.dot(plant.bmat.col(j));  // (2 d^T B)_j
      triplets.emplace_back(row, j, coeff);
    }
    triplets.emplace_back(row, n, 1.0);  // +delta slack
    lower(row) = b;
    upper(row) = kInf;
  }

  for (int j = 0; j < n; ++j) {
    const int r = static_cast<int>(n_obs) + j;
    triplets.emplace_back(r, j, 1.0);
    lower(r) = plant.u_lo[j];
    upper(r) = plant.u_hi[j];
  }
  const int rs = static_cast<int>(n_obs) + n;  // slack box
  triplets.emplace_back(rs, n, 1.0);
  lower(rs) = 0.0;
  upper(rs) = kInf;

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
    fb.feasible = false;
    fb.hard_safe = false;
    return fb;
  }
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    fb.feasible = false;
    fb.hard_safe = false;
    return fb;
  }

  const Eigen::VectorXd sol = solver.getSolution();
  QpOut out;
  for (int i = 0; i < n; ++i) {
    out.u[i] = std::clamp(sol(i), plant.u_lo[i], plant.u_hi[i]);
  }
  out.slack = std::max(0.0, sol(n));
  out.feasible = true;
  out.hard_safe = out.slack <= 1.0e-3;
  out.modified = false;
  for (int i = 0; i < n; ++i) {
    if (std::abs(out.u[i] - u_cl[i]) > 1.0e-3) {
      out.modified = true;
    }
  }
  return out;
}
}  // namespace

double CbfSafetyFilter::barrierResidual(
  const RobotState & state, double v, double w,
  const TrackedObstacle & obstacle, double alpha) const
{
  return residualImpl(cfg_, state, {v, w, 0.0}, obstacle, alpha);
}

CbfSafetyFilter::Result CbfSafetyFilter::filter(
  const RobotState & state, double v_nom, double w_nom,
  const std::vector<TrackedObstacle> & obstacles, double alpha_override) const
{
  const double alpha = alpha_override > 0.0 ? alpha_override : cfg_.alpha;
  const QpOut o = solveQp(cfg_, state, {v_nom, w_nom, 0.0}, obstacles, alpha);
  Result r;
  r.v = o.u[0];
  r.w = o.u[1];
  r.vy = 0.0;
  r.slack = o.slack;
  r.feasible = o.feasible;
  r.hard_safe = o.hard_safe;
  r.modified = o.modified;
  return r;
}

}  // namespace nav2_se_controller
```

- [ ] **Step 3: Build**

Run: `micromamba run -n ros2 bash -lc 'cd /home/cona/kangj/se-mppi-nav2 && colcon build --packages-select nav2_se_controller'`
Expected: build succeeds.

- [ ] **Step 4: Run the full existing CBF suite (regression gate)**

Run: `./build/nav2_se_controller/test_nav2_se_controller --gtest_filter='CbfSafetyFilter.*:CbfResponsibility.*'`
Expected: PASS — identical set to Step 1 (differential behavior unchanged). If any differs, the refactor diverged; do not proceed.

- [ ] **Step 5: Commit**

```bash
git add src/cbf_safety_filter.cpp
git commit -m "refactor(cbf): generic (J,U) plant + QP; differential behavior unchanged

Claude-Session: https://claude.ai/code/session_01QK1AeEbWiAfk1EKezRWnqu"
```

---

### Task 3: Omnidirectional instantiation + omni public methods

**Files:**
- Modify: `include/nav2_se_controller/cbf_safety_filter.hpp` (add two overloads + doc)
- Modify: `src/cbf_safety_filter.cpp` (add the two overload bodies)
- Test: `test/test_cbf_safety_filter.cpp` (append omni suite)

**Interfaces:**
- Consumes: `solveQp`/`residualImpl` (Task 2); `CbfConfig.drivetrain == Omnidirectional`, `vy_max`, `w_vy`.
- Produces:
  `Result filter(const RobotState&, double vx_nom, double vy_nom, double w_nom, const std::vector<TrackedObstacle>&, double alpha_override = -1.0) const;`
  `double barrierResidual(const RobotState&, double vx, double vy, double w, const TrackedObstacle&, double alpha) const;`
  Result mapping for omni: `r.v = u[0] (vx)`, `r.vy = u[1]`, `r.w = u[2]`.

- [ ] **Step 1: Write the failing omni tests**

Append to `test/test_cbf_safety_filter.cpp`:

```cpp
namespace
{
CbfConfig makeOmniConfig()
{
  CbfConfig c = makeConfig();
  c.drivetrain = Drivetrain::Omnidirectional;
  c.vy_max = 0.5;
  c.w_vy = 1.0;
  return c;
}
}  // namespace

TEST(CbfOmni, NoObstacleReturnsNominalIncludingVy)
{
  CbfSafetyFilter f;
  f.configure(makeOmniConfig());
  auto r = f.filter(RobotState{0.0, 0.0, 0.0}, 0.3, 0.2, 0.1, {});  // vx, vy, w
  EXPECT_TRUE(r.feasible);
  EXPECT_FALSE(r.modified);
  EXPECT_NEAR(r.v, 0.3, 1e-6);
  EXPECT_NEAR(r.vy, 0.2, 1e-6);
  EXPECT_NEAR(r.w, 0.1, 1e-6);
}

TEST(CbfOmni, BarrierResidualMatchesAnalytic)
{
  CbfSafetyFilter f;
  f.configure(makeOmniConfig());
  const RobotState s{0.0, 0.0, 0.0};
  TrackedObstacle o = makeObstacle(0.6, 0.0, 0.1);  // ahead
  const double vx = 0.3, vy = 0.2, w = 0.1, alpha = 2.0;
  // base point (0,0); d = (-0.6, 0); eff_r = 0.22+0.1+0.05 = 0.37
  // pdot = (vx, vy) at yaw 0; hdot = 2 d.pdot = -1.2 vx (vy term 0)
  const double eff = 0.22 + 0.1 + 0.05;
  const double h = 0.6 * 0.6 - eff * eff;
  const double expected = -1.2 * vx + alpha * h;
  EXPECT_NEAR(f.barrierResidual(s, vx, vy, w, o, alpha), expected, 1e-6);
}

TEST(CbfOmni, LateralObstacleConstrainsLateralVelocity)
{
  // Obstacle to the LEFT; nominal lateral velocity drives toward it. The QP
  // must curb vy while leaving the (decoupled) forward axis near nominal.
  CbfSafetyFilter f;
  const CbfConfig cfg = makeOmniConfig();
  f.configure(cfg);
  const RobotState s{0.0, 0.0, 0.0};
  std::vector<TrackedObstacle> obs = {makeObstacle(0.0, 0.5, 0.1)};

  // Nominal lateral velocity violates the barrier (vy must be reduced).
  const double nominal_res = f.barrierResidual(s, 0.0, 0.4, 0.0, obs[0], cfg.alpha);
  ASSERT_LT(nominal_res, 0.0);

  auto r = f.filter(s, 0.0, 0.4, 0.0, obs);
  EXPECT_TRUE(r.feasible);
  EXPECT_TRUE(r.modified);
  EXPECT_LT(r.vy, 0.4);              // lateral motion toward obstacle curbed
  EXPECT_NEAR(r.v, 0.0, 1e-2);       // forward axis untouched (decoupled)
  const double res = f.barrierResidual(s, r.v, r.vy, r.w, obs[0], cfg.alpha);
  EXPECT_GE(res, -1e-3);
}

TEST(CbfOmni, HeadingRateDecoupledFromAvoidance)
{
  // Obstacle ahead; nominal carries a forward speed AND a turn rate. The QP
  // curbs forward speed for safety but leaves w untouched, because w does not
  // enter the positional barrier for a holonomic base.
  CbfSafetyFilter f;
  f.configure(makeOmniConfig());
  const RobotState s{0.0, 0.0, 0.0};
  std::vector<TrackedObstacle> obs = {makeObstacle(0.6, 0.0, 0.1)};
  auto r = f.filter(s, 0.5, 0.0, 0.5, obs);
  EXPECT_TRUE(r.feasible);
  EXPECT_TRUE(r.modified);
  EXPECT_LT(r.v, 0.5);              // forward curbed
  EXPECT_NEAR(r.w, 0.5, 1e-2);     // heading rate preserved (decoupled)
}
```

- [ ] **Step 2: Run to verify they fail (compile error)**

Run: `micromamba run -n ros2 bash -lc 'cd /home/cona/kangj/se-mppi-nav2 && colcon build --packages-select nav2_se_controller'`
Expected: FAIL — no 4-velocity-arg `filter` / 3-velocity-arg `barrierResidual` overload.

- [ ] **Step 3: Declare the overloads in the header**

In `cbf_safety_filter.hpp`, inside `class CbfSafetyFilter` after the existing `filter(...)` declaration (line ~79), add:

```cpp
  /**
   * @brief Omnidirectional overload: filter a nominal (vx, vy, w).
   *        Requires cfg.drivetrain == Drivetrain::Omnidirectional. The barrier
   *        acts on the base point through (vx, vy); w is unconstrained by it.
   *        Result carries vx in `v`, the lateral velocity in `vy`, and w in `w`.
   */
  Result filter(
    const RobotState & state,
    double vx_nom,
    double vy_nom,
    double w_nom,
    const std::vector<TrackedObstacle> & obstacles,
    double alpha_override = -1.0) const;
```

And after the existing `barrierResidual(...)` declaration (line ~91), add:

```cpp
  /**
   * @brief Omnidirectional barrier residual for control (vx, vy, w).
   *        Requires cfg.drivetrain == Drivetrain::Omnidirectional.
   */
  double barrierResidual(
    const RobotState & state,
    double vx,
    double vy,
    double w,
    const TrackedObstacle & obstacle,
    double alpha) const;
```

- [ ] **Step 4: Implement the overload bodies**

In `src/cbf_safety_filter.cpp`, before the final `}  // namespace nav2_se_controller`, add:

```cpp
double CbfSafetyFilter::barrierResidual(
  const RobotState & state, double vx, double vy, double w,
  const TrackedObstacle & obstacle, double alpha) const
{
  return residualImpl(cfg_, state, {vx, vy, w}, obstacle, alpha);
}

CbfSafetyFilter::Result CbfSafetyFilter::filter(
  const RobotState & state, double vx_nom, double vy_nom, double w_nom,
  const std::vector<TrackedObstacle> & obstacles, double alpha_override) const
{
  const double alpha = alpha_override > 0.0 ? alpha_override : cfg_.alpha;
  const QpOut o = solveQp(cfg_, state, {vx_nom, vy_nom, w_nom}, obstacles, alpha);
  Result r;
  r.v = o.u[0];
  r.vy = o.u[1];
  r.w = o.u[2];
  r.slack = o.slack;
  r.feasible = o.feasible;
  r.hard_safe = o.hard_safe;
  r.modified = o.modified;
  return r;
}
```

- [ ] **Step 5: Build and run the omni suite**

Run: build, then `./build/nav2_se_controller/test_nav2_se_controller --gtest_filter='CbfOmni.*'`
Expected: PASS (4 tests).

- [ ] **Step 6: Run the whole CBF file to confirm no regression**

Run: `./build/nav2_se_controller/test_nav2_se_controller --gtest_filter='Cbf*'`
Expected: PASS (existing differential + responsibility + new omni).

- [ ] **Step 7: Commit**

```bash
git add include/nav2_se_controller/cbf_safety_filter.hpp src/cbf_safety_filter.cpp test/test_cbf_safety_filter.cpp
git commit -m "feat(cbf): omnidirectional instantiation (body-frame barrier, decoupled w)

Claude-Session: https://claude.ai/code/session_01QK1AeEbWiAfk1EKezRWnqu"
```

---

### Task 4: Wire `se_drivetrain` + `se_cbf_vy_max` into the controller

**Files:**
- Modify: `src/safe_escape_controller.cpp:58-64` (the `CbfConfig fc` block)
- Modify: `config/nav2_se_controller_params.yaml` (document the keys)
- Test: covered by the existing `test_plugin_load` (the controller still loads); no new unit test (param parsing is config glue).

**Interfaces:**
- Consumes: `CbfConfig.drivetrain`, `CbfConfig.vy_max` (Task 1); the `getParam` helper already in scope at this point in `configure()`.
- Produces: runtime params `se_drivetrain` (string `"diff"`|`"omni"`, default `"diff"`) and `se_cbf_vy_max` (double, default `0.0`).

- [ ] **Step 1: Read the surrounding code**

Confirm the block at `src/safe_escape_controller.cpp:58-64` matches:

```cpp
  CbfConfig fc;
  fc.alpha = cc.alpha_base;
  getParam(fc.lookahead, "se_cbf_lookahead", 0.2);
  getParam(fc.safety_margin, "se_cbf_safety_margin", 0.05);
  getParam(fc.slack_weight, "se_cbf_slack_weight", 1.0e3);
  fc.robot_radius = robot_radius_;
  filter_.configure(fc);
```

- [ ] **Step 2: Add the drivetrain + vy_max parsing**

Replace that block with:

```cpp
  CbfConfig fc;
  fc.alpha = cc.alpha_base;
  getParam(fc.lookahead, "se_cbf_lookahead", 0.2);
  getParam(fc.safety_margin, "se_cbf_safety_margin", 0.05);
  getParam(fc.slack_weight, "se_cbf_slack_weight", 1.0e3);
  fc.robot_radius = robot_radius_;
  // Drivetrain selection (D1): "diff" (default) or "omni". For omni, vy_max > 0
  // enables holonomic lateral motion in the CBF-QP.
  std::string drivetrain = "diff";
  getParam(drivetrain, "se_drivetrain", std::string("diff"));
  fc.drivetrain =
    drivetrain == "omni" ? Drivetrain::Omnidirectional : Drivetrain::Differential;
  getParam(fc.vy_max, "se_cbf_vy_max", 0.0);
  filter_.configure(fc);
```

(`Drivetrain` and `CbfConfig` are already visible via the existing `cbf_safety_filter.hpp` include; if the build reports `Drivetrain` not found, add `#include "nav2_se_controller/cbf_types.hpp"` near the other includes — it is header-only.)

- [ ] **Step 3: Document the keys in the params yaml**

In `config/nav2_se_controller_params.yaml`, after the `se_cbf_slack_weight` line (line 32), add:

```yaml
      # Drivetrain (Drivetrain-General SE-MPPI, D1). "diff" (default) uses the
      # look-ahead-point unicycle CBF; "omni" uses a body-frame holonomic CBF
      # where vx/vy enter the barrier and w decouples from it. Set se_cbf_vy_max
      # > 0 (e.g. 0.5) for a mecanum/omni base; 0 keeps lateral motion disabled.
      se_drivetrain: "diff"             # "diff" | "omni"
      se_cbf_vy_max: 0.0                # [m/s] |vy| limit (omni only)
```

- [ ] **Step 4: Build and confirm the plugin still loads**

Run: `micromamba run -n ros2 bash -lc 'cd /home/cona/kangj/se-mppi-nav2 && colcon build --packages-select nav2_se_controller && ./build/nav2_se_controller/test_plugin_load'`
Expected: build succeeds; `test_plugin_load` PASS (controller + critic load via pluginlib).

- [ ] **Step 5: Full test sweep**

Run: `micromamba run -n ros2 bash -lc 'cd /home/cona/kangj/se-mppi-nav2 && colcon test --packages-select nav2_se_controller --event-handlers console_direct+ && colcon test-result --verbose'`
Expected: all tests PASS (gtest + linters); no regressions.

- [ ] **Step 6: Commit**

```bash
git add src/safe_escape_controller.cpp config/nav2_se_controller_params.yaml
git commit -m "feat(controller): se_drivetrain + se_cbf_vy_max params (diff|omni)

Claude-Session: https://claude.ai/code/session_01QK1AeEbWiAfk1EKezRWnqu"
```

---

## Self-Review

**1. Spec coverage.**
- §3.3 `(J,U)` abstraction → Task 2 (`CbfPlant`/`buildPlant`/generic `solveQp`). ✓
- §3.4 D1 (drivetrain-general formulation) → Tasks 1–3. ✓
- §3.4 D3 (holonomic exploitation: decoupled ω, lateral handling) → Task 3 omni tests (`HeadingRateDecoupledFromAvoidance`, `LateralObstacleConstrainsLateralVelocity`). Partial — escape-side holonomic gap-attraction is escape-critic work, correctly deferred (this plan is the CBF half). ✓ (noted)
- §3.7 implementation (drivetrain enum, diff path unchanged, config plumbing, no perception code) → Tasks 1, 2 (regression), 4. ✓
- Ackermann/D2, sim models, eval, paper → explicitly out of scope. ✓

**2. Placeholder scan.** No TBD/TODO/"add error handling"/"similar to Task N". Every code step has complete code; every run step has a command + expected result. ✓

**3. Type consistency.** `Drivetrain::{Differential,Omnidirectional}`, `CbfConfig.{drivetrain,vy_max,w_vy}`, `Result.vy`, `CbfPlant.{safety_point,bmat,n_ctrl,u_lo,u_hi,w_track}`, `QpOut.{u,slack,feasible,hard_safe,modified}`, and the overload signatures match across Tasks 1→4. Differential `filter`/`barrierResidual` signatures are byte-identical to the originals (regression-safe). The omni `filter` maps `u[1]→vy`, `u[2]→w` consistent with `buildPlant`'s omni control order `[vx,vy,w]`. ✓

---

## Execution Handoff (after plan approval)

Recommended: subagent-driven execution (fresh subagent per task, review between tasks). Task 2 is the riskiest (regression lock) and deserves the closest review gate.
