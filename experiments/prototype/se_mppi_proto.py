#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""SE-MPPI standalone 2D validation.

A faithful NumPy re-implementation of the SE-MPPI algorithm (the C++ Nav2
controller's core) on a 2D unicycle, so the contribution can be validated
end-to-end without ROS / Gazebo / a lidar. The math mirrors the C++:

  - MPPI            : sampling-based MPC over (v, w) for a unicycle.
  - EntrapmentDetector : monotonic progress stall, clears on progress
                         (entrapment_detector.hpp).
  - APF repulsion   : U = 0.5*eta*(1/d - 1/d0)^2, d<d0 (repulsion.cpp).
  - Gap search      : raycast for the goal-closest opening (gap_search.cpp).
  - CBF filter      : look-ahead-point DCBF-QP via OSQP (cbf_safety_filter.cpp).
  - Coordinator     : alpha base/escape with TTC override (escape_safety_*).

Obstacles are circles (matching TrackedObstacle: position, velocity, radius);
static walls are built from circle chains. Distance fields / raycasts are
computed analytically against the circles.
"""

import numpy as np
import osqp
from scipy import sparse


# --------------------------------------------------------------------------- #
# World
# --------------------------------------------------------------------------- #
class Obstacle:
    def __init__(self, x, y, r, vx=0.0, vy=0.0):
        self.p = np.array([x, y], float)
        self.v = np.array([vx, vy], float)
        self.r = float(r)

    def at(self, t):
        return self.p + self.v * t


class World:
    def __init__(self, obstacles, goal, robot_radius=0.22):
        self.obstacles = obstacles
        self.goal = np.array(goal, float)
        self.robot_radius = robot_radius

    def step_obstacles(self, dt):
        for o in self.obstacles:
            o.p = o.p + o.v * dt

    def min_clearance(self, pos):
        """Distance from pos to the nearest obstacle surface (negative = inside)."""
        if not self.obstacles:
            return np.inf
        return min(np.linalg.norm(pos - o.p) - o.r for o in self.obstacles)

    def in_collision(self, pos):
        return self.min_clearance(pos) < self.robot_radius


# --------------------------------------------------------------------------- #
# Escape: entrapment detector + APF + gap search  (mirror C++)
# --------------------------------------------------------------------------- #
class EntrapmentDetector:
    def __init__(self, stall_window=15):
        self.window = stall_window
        self.reset()

    def reset(self):
        self._seen = False
        self.entrapped = False
        self._stall = 0
        self._best = None  # most progress (smallest dist-to-goal) so far

    def update(self, dist_to_goal):
        # progress == new minimum distance-to-goal (monotonic furthest reached)
        if not self._seen:
            self._seen = True
            self._best = dist_to_goal
            return self.entrapped
        if dist_to_goal < self._best - 1e-3:
            self._best = dist_to_goal
            self._stall = 0
            self.entrapped = False
        elif not self.entrapped:
            self._stall += 1
            if self._stall >= self.window:
                self.entrapped = True
        return self.entrapped


def apf_repulsion(points, world, d0=0.8, eta=0.3):
    """Per-rollout APF cost from the true obstacle distance field.

    points: (K, T, 2). Returns (K,) mean APF potential over horizon.
    """
    k, t, _ = points.shape
    out = np.zeros(k)
    inv_d0 = 1.0 / d0
    for o in world.obstacles:
        d = np.linalg.norm(points - o.p, axis=2) - o.r  # (K,T) surface distance
        d = np.maximum(d, 0.02)
        u = np.where(d < d0, 0.5 * eta * (1.0 / d - inv_d0) ** 2, 0.0)
        out += u.mean(axis=1)
    return out


def find_escape_gap(pos, goal_bearing, world, num_rays=72, max_range=3.0,
                    min_clearance=0.7, prev_bearing=None, hysteresis=0.5):
    """Raycast for the viable opening closest to the goal bearing.

    A viable gap is a bearing whose free ray distance >= min_clearance. Among
    them the one minimizing alignment error to the goal is chosen; a hysteresis
    term biases toward `prev_bearing` so symmetric openings (e.g. a centred
    wall) don't make the choice flip every cycle and stall the robot.
    """
    best = None
    best_score = np.inf
    for k in range(num_rays):
        bearing = -np.pi + 2 * np.pi * k / num_rays
        d = ray_free_distance(pos, bearing, world, max_range)
        if d < min_clearance:
            continue
        score = abs(_wrap(bearing - goal_bearing))
        if prev_bearing is not None:
            score += hysteresis * abs(_wrap(bearing - prev_bearing))
        if score < best_score:
            best_score = score
            best = bearing
    return best


def ray_free_distance(pos, bearing, world, max_range, step=0.05):
    dvec = np.array([np.cos(bearing), np.sin(bearing)])
    r = step
    while r <= max_range:
        p = pos + r * dvec
        if world.min_clearance(p) < world.robot_radius:
            return r
        r += step
    return max_range


def gap_attraction(points, pos, gap_bearing, weight=14.0):
    """Reward (low cost) rollouts whose endpoint aligns with the gap bearing."""
    ep = points[:, -1, :] - pos
    ep_bearing = np.arctan2(ep[:, 1], ep[:, 0])
    delta = _wrap(ep_bearing - gap_bearing)
    return weight * (1.0 - np.cos(delta))


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


# --------------------------------------------------------------------------- #
# Escape-safety coordinator + TTC  (mirror C++)
# --------------------------------------------------------------------------- #
def min_time_to_collision(state, v, world):
    p = state[:2]
    vr = v * np.array([np.cos(state[2]), np.sin(state[2])])
    ttc = np.inf
    for o in world.obstacles:
        rel = o.p - p
        rng = np.linalg.norm(rel)
        if rng < 1e-6:
            return 0.0
        clear = rng - (world.robot_radius + o.r)
        if clear <= 0:
            return 0.0
        closing = -rel.dot(o.v - vr) / rng
        if closing <= 1e-6:
            continue
        ttc = min(ttc, clear / closing)
    return ttc


def coordinate_alpha(entrapped, ttc, alpha_base=2.0, alpha_escape=6.0,
                     ttc_threshold=1.5):
    if not entrapped:
        return alpha_base
    if ttc < ttc_threshold:
        return alpha_base       # dynamic safety overrides escape
    return alpha_escape


# --------------------------------------------------------------------------- #
# CBF safety filter (look-ahead-point DCBF-QP)  (mirror C++)
# --------------------------------------------------------------------------- #
def cbf_filter(state, u_nom, world, alpha, lookahead=0.2, margin=0.05,
               slack_weight=1e3, v_lim=(-0.35, 0.5), w_lim=1.9):
    x, y, th = state
    c, s = np.cos(th), np.sin(th)
    p_l = np.array([x + lookahead * c, y + lookahead * s])
    G = np.array([[c, -lookahead * s], [s, lookahead * c]])

    rows, lows = [], []
    for o in world.obstacles:
        d = p_l - o.p
        eff_r = world.robot_radius + o.r + margin
        h = d.dot(d) - eff_r ** 2
        a = 2.0 * d @ G                       # 1x2
        b = -alpha * h + 2.0 * d.dot(o.v)
        rows.append([a[0], a[1], 1.0])        # A z + delta >= b
        lows.append(b)

    # variables z = [v, w, delta]; objective ||u-u_nom||^2_W + rho*delta^2
    P = sparse.diags([1.0, 1.0, slack_weight]).tocsc()
    q = np.array([-u_nom[0], -u_nom[1], 0.0])

    A_rows = list(rows) + [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    lo = list(lows) + [v_lim[0], -w_lim, 0.0]
    hi = [np.inf] * len(rows) + [v_lim[1], w_lim, np.inf]
    A = sparse.csc_matrix(np.array(A_rows, float))
    lo = np.array(lo)
    hi = np.array(hi)

    prob = osqp.OSQP()
    prob.setup(P, q, A, lo, hi, verbose=False, eps_abs=1e-6, eps_rel=1e-6)
    res = prob.solve()
    if res.info.status_val not in (1, 2):     # solved / solved-inaccurate
        v = np.clip(u_nom[0], *v_lim)
        return v, np.clip(u_nom[1], -w_lim, w_lim), 0.0, False
    z = res.x
    slack = max(0.0, z[2])
    hard_safe = slack <= 1e-3
    return (np.clip(z[0], *v_lim), np.clip(z[1], -w_lim, w_lim), slack, hard_safe)


# --------------------------------------------------------------------------- #
# MPPI optimizer (unicycle)
# --------------------------------------------------------------------------- #
class MPPI:
    def __init__(self, horizon=25, dt=0.1, batch=600, temperature=0.3,
                 v_std=0.25, w_std=0.6, v_lim=(-0.35, 0.5), w_lim=1.9, seed=0,
                 prefer_forward=0.0):
        self.T, self.dt, self.K = horizon, dt, batch
        self.temp = temperature
        self.v_std, self.w_std = v_std, w_std
        self.v_lim, self.w_lim = v_lim, w_lim
        self.prefer_forward = prefer_forward
        self.u = np.zeros((horizon, 2))      # nominal control sequence
        self.rng = np.random.default_rng(seed)

    def rollout(self, state, noise):
        # noise: (K, T, 2). returns trajectories (K, T, 2) of (x,y) + controls
        K = noise.shape[0]
        u = np.clip(self.u[None] + noise, [self.v_lim[0], -self.w_lim],
                    [self.v_lim[1], self.w_lim])
        x = np.tile(state.astype(float), (K, 1))
        pts = np.empty((K, self.T, 2))
        for t in range(self.T):
            v, w = u[:, t, 0], u[:, t, 1]
            x[:, 0] += v * np.cos(x[:, 2]) * self.dt
            x[:, 1] += v * np.sin(x[:, 2]) * self.dt
            x[:, 2] += w * self.dt
            pts[:, t, :] = x[:, :2]
        return pts, u

    def step(self, state, world, escape_costs=None, goal_scale=1.0,
             eff_goal=None):
        # eff_goal lets the escape layer steer toward a temporary subgoal (an
        # opening) instead of the real goal while entrapped.
        goal = world.goal if eff_goal is None else np.asarray(eff_goal, float)
        noise = self.rng.normal(0, [self.v_std, self.w_std],
                                (self.K, self.T, 2))
        pts, u = self.rollout(state, noise)
        # --- base costs ---
        goal_d = np.linalg.norm(pts[:, -1, :] - goal, axis=1)
        cost = 8.0 * goal_scale * goal_d
        # path-progress shaping: reward getting closer along the horizon
        cost += 1.0 * goal_scale * np.linalg.norm(pts - goal, axis=2).mean(axis=1)
        # obstacle cost (collision = large; proximity = soft). An obstacle may
        # carry a reduced `soft` zone — the escape-coordination hook: while a
        # certified-safe close pass is intended, the CBF (not the sampling
        # cost) owns the safety margin, mirroring the alpha-raise principle.
        for o in world.obstacles:
            d = np.linalg.norm(pts - o.p, axis=2) - o.r - world.robot_radius
            zone = getattr(o, 'soft', 0.3)
            cost += np.where(d < 0, 1e4, 0.0).sum(axis=1)
            cost += np.where((d >= 0) & (d < zone), 30.0 * (zone - d), 0.0).sum(axis=1)
        # prefer-forward (mirrors PreferForwardCritic), OPT-IN: the multi-robot
        # face-off needs it (a blocked robot otherwise drifts into reversed
        # headings it cannot recover from); the validated single-robot
        # scenarios keep their original cost (prefer_forward=0).
        if self.prefer_forward > 0.0:
            cost += self.prefer_forward * \
                np.maximum(-u[:, :, 0], 0.0).mean(axis=1) * self.T
        if escape_costs is not None:
            cost += escape_costs(pts)
        # --- info-theoretic update ---
        cost -= cost.min()
        w_ = np.exp(-cost / self.temp)
        w_ /= w_.sum() + 1e-9
        self.u = np.einsum('k,ktc->tc', w_, u)
        self.u[:, 0] = np.clip(self.u[:, 0], *self.v_lim)
        self.u[:, 1] = np.clip(self.u[:, 1], -self.w_lim, self.w_lim)
        u0 = self.u[0].copy()
        self.u[:-1] = self.u[1:]            # shift
        return u0
