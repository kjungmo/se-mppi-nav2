# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Seed-deterministic randomized scenario generators for the 2D benchmark.

Four families, each a factory ``make_<family>(seed) -> Scenario``:

  * ``utrap``   — a finite U/C pocket (random width, depth, back-wall position,
                  goal-behind offset, whole-pocket rotation) with the opening
                  facing the robot and the goal *behind* the closed back wall,
                  so greedy goal-descent drives the robot into a local minimum.
  * ``clutter`` — a random field of circular obstacles between start and goal,
                  with a guaranteed free start→goal path (verified at generation
                  time by an occupancy-grid flood-fill; blocking obstacles are
                  dropped until a path exists).
  * ``dynamic`` — light static background + 1–2 constant-velocity movers, each
                  aimed to cross the robot's straight-line path (random speed,
                  crossing angle, and timing).
  * ``narrowdyn`` — a U-trap pocket plus 1–2 movers timed to sweep the escape
                  region: the coordination-stress family (escape and the
                  dynamic-obstacle CBF must act in the same tight window).

Obstacles are ``se_mppi_proto.Obstacle`` circles; a mover carries a nonzero
velocity (that is how the rollout tells dynamic from static). The generators are
pure functions of ``seed`` — same seed ⇒ byte-identical world — which is what the
paired McNemar design requires.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

import numpy as np

# The validated 2D primitives live in experiments/prototype.
_PROTO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'prototype')
if _PROTO not in sys.path:
    sys.path.insert(0, _PROTO)
import se_mppi_proto as se  # noqa: E402

# Generation region (metres) and default robot radius (matches se.World).
BOUNDS = (-1.5, 8.0, -4.0, 4.0)   # (x_lo, x_hi, y_lo, y_hi)
ROBOT_RADIUS = 0.22


@dataclass
class Scenario:
    family: str
    seed: int
    world: "se.World"
    start: tuple            # (x, y, theta)
    goal: tuple             # (x, y)
    meta: dict = field(default_factory=dict)


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def _wall(p0, p1, r=0.33, spacing=0.5):
    """A solid wall as an overlapping chain of circles from ``p0`` to ``p1``."""
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    n = max(2, int(np.hypot(*(p1 - p0)) / spacing) + 1)
    return [se.Obstacle(x, y, r) for x, y in
            zip(np.linspace(p0[0], p1[0], n), np.linspace(p0[1], p1[1], n))]


def _rotate_about(obstacles, points, pivot, theta):
    """Rotate obstacle centres and free points about ``pivot`` by ``theta``."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    pivot = np.asarray(pivot, float)
    for o in obstacles:
        o.p = pivot + R @ (o.p - pivot)
    rot_pts = [pivot + R @ (np.asarray(p, float) - pivot) for p in points]
    return obstacles, rot_pts


def _heading_to(start_xy, goal_xy):
    d = np.asarray(goal_xy, float) - np.asarray(start_xy, float)
    return float(np.arctan2(d[1], d[0]))


# --------------------------------------------------------------------------- #
# Feasibility: occupancy-grid flood-fill from start to goal
# --------------------------------------------------------------------------- #
def free_path_exists(obstacles, start_xy, goal_xy, robot_radius=ROBOT_RADIUS,
                     bounds=BOUNDS, res=0.15, margin=0.03):
    """True iff a footprint-clear 8-connected grid path start→goal exists.

    A cell is traversable when its centre is ≥ ``robot_radius + margin`` from
    every obstacle surface. Coarse (``res`` m) but sufficient to *reject*
    generated scenarios with no solution — it never fabricates a path.
    """
    x_lo, x_hi, y_lo, y_hi = bounds
    nx = int(round((x_hi - x_lo) / res)) + 1
    ny = int(round((y_hi - y_lo) / res)) + 1
    clear = robot_radius + margin

    def to_cell(p):
        return (int(round((p[0] - x_lo) / res)),
                int(round((p[1] - y_lo) / res)))

    def free(ix, iy):
        if not (0 <= ix < nx and 0 <= iy < ny):
            return False
        x = x_lo + ix * res
        y = y_lo + iy * res
        for o in obstacles:
            if (x - o.p[0]) ** 2 + (y - o.p[1]) ** 2 < (o.r + clear) ** 2:
                return False
        return True

    s = to_cell(start_xy)
    g = to_cell(goal_xy)
    if not free(*s) or not free(*g):
        return False
    from collections import deque
    seen = {s}
    dq = deque([s])
    nbr = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while dq:
        cx, cy = dq.popleft()
        if (cx, cy) == g:
            return True
        for dx, dy in nbr:
            nc = (cx + dx, cy + dy)
            if nc not in seen and free(*nc):
                seen.add(nc)
                dq.append(nc)
    return False


# --------------------------------------------------------------------------- #
# utrap: finite U/C pocket, goal behind the closed back wall
# --------------------------------------------------------------------------- #
def make_utrap(seed: int) -> Scenario:
    rng = np.random.default_rng(1_000_000 + seed)
    width = rng.uniform(1.2, 2.2)          # back-wall span
    depth_t = rng.uniform(1.0, 1.9)        # top side-wall depth
    depth_b = rng.uniform(1.0, 1.9)        # bottom side-wall depth (asymmetry)
    xb = rng.uniform(2.6, 3.4)             # back-wall x position
    behind = rng.uniform(1.4, 2.4)         # goal offset behind the back wall
    r, sp = 0.33, 0.5

    start = np.array([0.0, float(rng.uniform(-0.4, 0.4))])
    # Canonical frame: opening faces -x (toward the robot), goal at +x behind.
    obs = []
    obs += _wall((xb, -width / 2), (xb, width / 2), r, sp)          # back wall
    obs += _wall((xb, width / 2), (xb - depth_t, width / 2), r, sp)  # top side
    obs += _wall((xb, -width / 2), (xb - depth_b, -width / 2), r, sp)  # bottom
    goal = np.array([xb + behind, float(rng.uniform(-0.3, 0.3))])

    theta = float(rng.uniform(-0.7, 0.7))  # rotate whole pocket + goal
    obs, (start_r, goal_r) = _rotate_about(obs, [start, goal], start, theta)

    world = se.World(obs, goal=tuple(goal_r), robot_radius=ROBOT_RADIUS)
    heading = _heading_to(start_r, goal_r)
    meta = {'width': width, 'depth_t': depth_t, 'depth_b': depth_b, 'xb': xb,
            'behind': behind, 'theta': theta, 'n_obstacles': len(obs),
            'n_dynamic': 0,
            'feasible': free_path_exists(obs, start_r, goal_r)}
    return Scenario('utrap', seed, world,
                    (float(start_r[0]), float(start_r[1]), heading),
                    tuple(float(v) for v in goal_r), meta)


# --------------------------------------------------------------------------- #
# clutter: random circle field with a guaranteed free corridor
# --------------------------------------------------------------------------- #
def make_clutter(seed: int) -> Scenario:
    rng = np.random.default_rng(2_000_000 + seed)
    start = np.array([0.0, float(rng.uniform(-0.5, 0.5))])
    goal = np.array([float(rng.uniform(5.0, 6.5)), float(rng.uniform(-1.2, 1.2))])
    n_target = int(rng.integers(5, 10))
    keepout = 0.9  # keep obstacles clear of start/goal

    obs = []
    tries = 0
    while len(obs) < n_target and tries < 400:
        tries += 1
        x = float(rng.uniform(0.9, goal[0] - 0.4))
        y = float(rng.uniform(BOUNDS[2] + 0.6, BOUNDS[3] - 0.6))
        r = float(rng.uniform(0.30, 0.60))
        p = np.array([x, y])
        if np.hypot(*(p - start)) < keepout + r or np.hypot(*(p - goal)) < keepout + r:
            continue
        if any(np.hypot(*(p - o.p)) < r + o.r + 0.15 for o in obs):
            continue  # avoid fully-merged blobs (keeps distinct obstacles)
        obs.append(se.Obstacle(x, y, r))

    # Guarantee passability: drop obstacles (nearest the straight line first)
    # until a free start→goal path exists.
    def line_dist(o):
        d = goal - start
        t = np.clip((o.p - start) @ d / (d @ d), 0.0, 1.0)
        return np.hypot(*(start + t * d - o.p))
    while obs and not free_path_exists(obs, start, goal):
        obs.pop(int(np.argmin([line_dist(o) for o in obs])))

    world = se.World(obs, goal=tuple(goal), robot_radius=ROBOT_RADIUS)
    heading = _heading_to(start, goal)
    meta = {'n_obstacles': len(obs), 'n_dynamic': 0, 'n_target': n_target,
            'feasible': free_path_exists(obs, start, goal)}
    return Scenario('clutter', seed, world,
                    (float(start[0]), float(start[1]), heading),
                    tuple(float(v) for v in goal), meta)


# --------------------------------------------------------------------------- #
# dynamic: static background + 1–2 crossing movers
# --------------------------------------------------------------------------- #
def make_dynamic(seed: int) -> Scenario:
    rng = np.random.default_rng(3_000_000 + seed)
    start = np.array([0.0, float(rng.uniform(-0.4, 0.4))])
    goal = np.array([float(rng.uniform(5.0, 6.5)), float(rng.uniform(-0.8, 0.8))])
    cruise = 0.4  # nominal robot speed used to time the crossings

    # Light static background off the direct corridor (may be empty).
    static = []
    for _ in range(int(rng.integers(0, 4))):
        x = float(rng.uniform(1.2, goal[0] - 0.6))
        y = float(rng.uniform(BOUNDS[2] + 0.8, BOUNDS[3] - 0.8))
        r = float(rng.uniform(0.30, 0.50))
        p = np.array([x, y])
        # keep clear of start, goal, and the straight corridor centre-line
        d = goal - start
        t = np.clip((p - start) @ d / (d @ d), 0.0, 1.0)
        corridor = np.hypot(*(start + t * d - p))
        if (np.hypot(*(p - start)) < 1.0 or np.hypot(*(p - goal)) < 1.0
                or corridor < 0.8):
            continue
        static.append(se.Obstacle(x, y, r))

    # Movers: each is placed so that it reaches the crossing point at (about)
    # the same time the robot does — the coincidence that makes a *reactive*
    # (current-position-only) planner collide and a *velocity-aware* CBF avoid.
    # Crossings are biased near-perpendicular (the threatening geometry), speeds
    # are fast enough to leave little reaction margin, and a small timing jitter
    # spreads outcomes across near-misses and hits (varied difficulty).
    n_mov = int(rng.integers(1, 3))                 # 1–2 (fewer, better-timed)
    movers = []
    path_vec = goal - start
    path_len = float(np.hypot(*path_vec))
    base = float(np.arctan2(path_vec[1], path_vec[0]))
    x_lo, x_hi, y_lo, y_hi = BOUNDS

    def _in_bounds(p):
        return (x_lo + 0.3 <= p[0] <= x_hi - 0.3 and
                y_lo + 0.3 <= p[1] <= y_hi - 0.3)

    for _ in range(n_mov):
        f = float(rng.uniform(0.35, 0.65))          # crossing fraction along path
        pc = start + f * path_vec                   # crossing point
        rel_ang = float(rng.uniform(1.05, 2.09))    # 60°–120° from the path
        if rng.random() < 0.5:
            rel_ang = -rel_ang
        cross_dir = np.array([np.cos(base + rel_ang), np.sin(base + rel_ang)])
        # Pedestrian-like speeds/sizes: threatening (little reaction margin for a
        # reactive planner) yet avoidable by velocity-aware anticipation.
        speed = float(rng.uniform(0.35, 0.58))
        mv = speed * cross_dir
        # time the robot ~reaches pc, plus jitter, then back the mover out along
        # its own heading so it arrives at pc at that time.
        t_cross = (f * path_len) / cruise + float(rng.uniform(-1.1, 1.1))
        t_cross = max(1.0, t_cross)
        # shrink t_cross if the implied start falls outside the arena
        while t_cross > 0.6 and not _in_bounds(pc - mv * t_cross):
            t_cross -= 0.2
        p0 = pc - mv * t_cross
        r = float(rng.uniform(0.24, 0.36))
        if np.hypot(*(p0 - start)) < 0.9:           # not spawned on the robot
            continue
        movers.append(se.Obstacle(float(p0[0]), float(p0[1]), r,
                                  vx=float(mv[0]), vy=float(mv[1])))

    obs = static + movers
    world = se.World(obs, goal=tuple(goal), robot_radius=ROBOT_RADIUS)
    heading = _heading_to(start, goal)
    meta = {'n_obstacles': len(obs), 'n_static': len(static),
            'n_dynamic': len(movers),
            # feasibility is judged on the static background only
            'feasible': free_path_exists(static, start, goal)}
    return Scenario('dynamic', seed, world,
                    (float(start[0]), float(start[1]), heading),
                    tuple(float(v) for v in goal), meta)


# --------------------------------------------------------------------------- #
# narrowdyn: local minimum WITH a mover on the escape route (coordination test)
# --------------------------------------------------------------------------- #
def make_narrowdyn(seed: int) -> Scenario:
    """A U-trap pocket plus 1–2 movers crossing the escape region.

    This is the *narrow-AND-dynamic* coincidence the coordination contribution
    (E vs F) targets: the robot must run the escape maneuver (α raised) while a
    dynamic obstacle threatens the route the escape uses. Static pocket geometry
    is reused verbatim from ``make_utrap`` (so the trap is identical to the
    static family), and movers are timed to sweep the pocket mouth during the
    escape window. The CBF/TTC (dynamic-only) therefore act exactly on the
    escape route — where α coordination decides whether the squeeze is admitted.
    """
    base = make_utrap(seed)
    rng = np.random.default_rng(4_000_000 + seed)
    world = base.world
    start = np.array(base.start[:2], float)
    goal = np.array(base.goal, float)
    path = goal - start
    path_len = float(np.hypot(*path))
    path_dir = path / path_len
    perp = np.array([-path_dir[1], path_dir[0]])
    cruise = 0.4
    mouth = start + 0.45 * path                    # in front of the pocket back wall
    t_reach = float(np.hypot(*(mouth - start))) / cruise

    n_mov = int(rng.integers(1, 3))
    movers = []
    for _ in range(n_mov):
        side = 1.0 if rng.random() < 0.5 else -1.0
        speed = float(rng.uniform(0.30, 0.50))
        mv = speed * perp * side                   # cross the mouth laterally
        pc = mouth + float(rng.uniform(-0.5, 0.5)) * path_dir
        # cross pc during the escape window (after the robot reaches the pocket
        # and stalls, ~+1..4 s), backing the mover out along its own heading.
        t_cross = t_reach + float(rng.uniform(1.0, 4.0))
        while t_cross > 0.6 and not (BOUNDS[0] + 0.3 <= (pc - mv * t_cross)[0]
                                     <= BOUNDS[1] - 0.3 and
                                     BOUNDS[2] + 0.3 <= (pc - mv * t_cross)[1]
                                     <= BOUNDS[3] - 0.3):
            t_cross -= 0.2
        p0 = pc - mv * t_cross
        r = float(rng.uniform(0.24, 0.34))
        if np.hypot(*(p0 - start)) < 0.9:
            continue
        movers.append(se.Obstacle(float(p0[0]), float(p0[1]), r,
                                  vx=float(mv[0]), vy=float(mv[1])))
    world.obstacles.extend(movers)

    static = [o for o in world.obstacles if np.hypot(o.v[0], o.v[1]) <= 1e-9]
    meta = {'n_obstacles': len(world.obstacles), 'n_static': len(static),
            'n_dynamic': len(movers),
            'feasible': free_path_exists(static, start, goal)}
    return Scenario('narrowdyn', seed, world, base.start, base.goal, meta)


FAMILIES = {
    'utrap': make_utrap,
    'clutter': make_clutter,
    'dynamic': make_dynamic,
    'narrowdyn': make_narrowdyn,
}


def make(family: str, seed: int) -> Scenario:
    return FAMILIES[family](seed)
