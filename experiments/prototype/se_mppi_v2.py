# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""SE-MPPI v2 escape logic for the 2D mirror (opt-in; paper-v1 is the default).

The v1 mirror differs from the shipped plugin at three places: the detector
input (distance to the goal), the opening selection (closest-to-goal among rays
that clear 2.4 m of 3.0 m), and the escape actuation (a rolling 2.5 m subgoal).
This module holds the v2 replacements. Every piece is behind a flag in
``V2_PARAMS`` and is only used when a config carries a ``'v2'`` dict
(``experiments/benchmark2d/configs.py``). Nothing here is imported by the v1
code path.

  1. ``AnchorProgress``     progress on a per-task anchor path with simulated
                            1 Hz straight-line replans from the robot pose.
  2. ``select_opening``     horizon-ray opening selection with 0.5 rad hysteresis.
  3. ``WaypointHopper``     frozen waypoint with the re-selection rules of
                            DESIGN_ADVICE section 2 (defect 4).
  4. all rollouts collide   a zero command; the switch lives in
                            ``se_mppi_proto.MPPI.step(zero_on_all_collide=...)``.

The numeric values follow the C++ parameter set of DESIGN_ADVICE 4.2 (36 rays,
2.0 m range, 3 m x 3 m rolling window, stall window 30 cycles).
"""

from __future__ import annotations

import numpy as np

V2_PARAMS = dict(
    # --- the four switches (default of the v2 set = all on) ---
    progress='anchor',            # 'goal_distance' = paper-v1 detector input
    gap_rule='horizon',           # 'closest_viable' = paper-v1 selection
    hop=True,                     # False = re-select every step (v1 actuation)
    zero_on_all_collide=True,     # False = v1: keep the best colliding rollout
    # --- detector / progress (DESIGN_ADVICE 2 defect 3, 4.2) ---
    stall_window=30,              # cycles (3 s at dt 0.1)
    progress_min_advance=0.10,    # m, one progress event
    progress_adopt_advance=0.5,   # m, a replan becomes the anchor
    replan_period_s=1.0,          # the BT's 1 Hz replan from the robot pose
    # --- opening selection (defect 4a) ---
    gap_num_rays=36,
    gap_max_range=2.0,            # m
    window_half=1.5,              # m, half of the 3 m rolling costmap
    hysteresis=0.5,               # rad
    # --- waypoint (defect 4b) ---
    waypoint_cap=2.5,             # m
    waypoint_margin=0.05,         # m, subtracted with the robot radius
    reselect_radius=0.4,          # m
    reselect_timeout_s=15.0,      # s
)


def v2_params(**overrides):
    p = dict(V2_PARAMS)
    unknown = set(overrides) - set(p)
    if unknown:
        raise KeyError(f'unknown v2 parameter(s): {sorted(unknown)}')
    p.update(overrides)
    return p


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


# --------------------------------------------------------------------------- #
# 1. Anchor-path progress
# --------------------------------------------------------------------------- #
class PolylinePath:
    """A planar polyline with cumulative arc length."""

    def __init__(self, pts):
        pts = np.asarray(pts, float).reshape(-1, 2)
        keep = [0]
        for i in range(1, len(pts)):
            if np.hypot(*(pts[i] - pts[keep[-1]])) > 1e-9:
                keep.append(i)
        self.pts = pts[keep]
        if len(self.pts) < 2:
            # degenerate (robot already at the goal): a zero-length path
            self.pts = np.vstack([self.pts[0], self.pts[0]])
        d = np.diff(self.pts, axis=0)
        self.seg_len = np.hypot(d[:, 0], d[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])

    @property
    def length(self):
        return float(self.cum[-1])

    def nearest_arc(self, p):
        """Arc length of the path point nearest to ``p``."""
        p = np.asarray(p, float)
        a = self.pts[:-1]
        d = self.pts[1:] - a
        sl2 = np.maximum(self.seg_len ** 2, 1e-18)
        t = np.clip(((p - a) * d).sum(axis=1) / sl2, 0.0, 1.0)
        q = a + t[:, None] * d
        i = int(np.argmin(((q - p) ** 2).sum(axis=1)))
        return float(self.cum[i] + t[i] * self.seg_len[i])


class AnchorProgress:
    """Progress along a per-task anchor path (DESIGN_ADVICE 2, defect 3).

    * The anchor is the first plan of the task. A progress event fires when the
      arc length at the nearest anchor point exceeds the best so far by
      ``min_advance``. A same-goal replan does not touch the anchor.
    * Each replan is a candidate. The tracker measures the advance of the robot
      along it from its arrival pose (its arc length at the nearest point, so a
      robot that backs off and returns has advanced by its net displacement); at
      ``adopt_advance`` the candidate becomes the anchor and a progress event
      fires.
    * Mirror reading of "within seconds" for a detour: the BT replans every
      second and a robot cannot move 0.5 m in one second, so a replan inherits
      the advance already made along the previous candidate when that advance
      was at least ``min_advance`` (the robot is following the previous replan).
      Otherwise the advance restarts at zero. A robot creeping less than
      ``min_advance`` per replan, or rocking back and forth, never accumulates.

    ``events`` is a monotone counter; the (unchanged) ``EntrapmentDetector``
    receives ``-events`` so that each event is a new minimum.
    """

    def __init__(self, start_xy, goal_xy, min_advance=0.10, adopt_advance=0.5):
        self.min_advance = float(min_advance)
        self.adopt_advance = float(adopt_advance)
        self.anchor = PolylinePath([start_xy, goal_xy])
        self.best = self.anchor.nearest_arc(start_xy)
        self.events = 0
        self.adoptions = 0
        self.replans = 0
        self._cand = None
        self._carried = 0.0
        self._cand_adv = 0.0

    def replan(self, pose_xy, goal_xy):
        """A fresh straight path from the robot pose (the BT's 1 Hz replan)."""
        if self._cand is not None and self._cand_adv >= self.min_advance:
            self._carried += self._cand_adv
        else:
            self._carried = 0.0
        self._cand = PolylinePath([pose_xy, goal_xy])
        self._cand_adv = 0.0
        self.replans += 1

    def update(self, pose_xy):
        arc = self.anchor.nearest_arc(pose_xy)
        if arc > self.best + self.min_advance:
            self.best = arc
            self.events += 1
        if self._cand is not None:
            # displacement along the candidate since its arrival (current, not
            # the best so far: backing off and returning earns nothing)
            self._cand_adv = self._cand.nearest_arc(pose_xy)
            if self._carried + self._cand_adv >= self.adopt_advance:
                self.anchor = self._cand
                self.best = self.anchor.nearest_arc(pose_xy)
                self.events += 1
                self.adoptions += 1
                self._cand = None
                self._carried = 0.0
                self._cand_adv = 0.0
        return self.events


# --------------------------------------------------------------------------- #
# 2. Horizon-ray opening selection
# --------------------------------------------------------------------------- #
def scan_rays(pos, world, num_rays, max_range, window_half, step=0.05):
    """Free length of ``num_rays`` rays and whether each reached its horizon.

    The horizon of a ray is ``min(max_range, distance to the edge of the square
    window of half-size window_half centred on the robot)``: the rolling
    costmap edge of the C++ controller. A ray stops at the first sample where
    the robot centre would collide (clearance < robot radius), as
    ``se_mppi_proto.ray_free_distance``. Returns ``(bearings, free, reached)``.
    """
    pos = np.asarray(pos, float)
    k = np.arange(num_rays)
    bearings = -np.pi + 2 * np.pi * k / num_rays
    c, s = np.cos(bearings), np.sin(bearings)
    limit = np.minimum(max_range, window_half / np.maximum(np.abs(c), np.abs(s)))
    n_max = int(np.floor(limit.max() / step + 1e-9))
    r = step * np.arange(1, n_max + 1)                      # (S,)
    pts = pos[None, None, :] + r[None, :, None] * np.stack([c, s], 1)[:, None, :]
    if world.obstacles:
        cen = np.array([o.p for o in world.obstacles])      # (O,2)
        rad = np.array([o.r for o in world.obstacles])
        dist = np.linalg.norm(pts[:, :, None, :] - cen[None, None, :, :], axis=3)
        clr = (dist - rad[None, None, :]).min(axis=2)       # (R,S)
    else:
        clr = np.full(pts.shape[:2], np.inf)
    hit = (clr < world.robot_radius) & (r[None, :] <= limit[:, None] + 1e-9)
    any_hit = hit.any(axis=1)
    first = np.argmax(hit, axis=1)
    free = np.where(any_hit, r[first], limit)
    return bearings, free, ~any_hit


def select_opening(pos, goal_bearing, world, p, prev_bearing=None):
    """Horizon rule (DESIGN_ADVICE 2, defect 4a).

    Among the rays whose free length reaches the sensing horizon, take the one
    closest to the goal bearing; keep the previous bearing unless the new one is
    better by more than ``hysteresis`` rad. If no ray reaches the horizon, take
    the longest ray. Returns ``(bearing, free_length, reached_horizon)``.
    """
    bearings, free, reached = scan_rays(pos, world, p['gap_num_rays'],
                                        p['gap_max_range'], p['window_half'])
    n = len(bearings)
    kp = None
    if prev_bearing is not None:
        kp = int(np.round((wrap(prev_bearing) + np.pi) / (2 * np.pi) * n)) % n
    if reached.any():
        cand = np.flatnonzero(reached)
        align = np.abs(wrap(bearings[cand] - goal_bearing))
        best = int(cand[int(np.argmin(align))])             # first of equals
        if kp is not None and reached[kp]:
            a_prev = abs(float(wrap(bearings[kp] - goal_bearing)))
            a_best = abs(float(wrap(bearings[best] - goal_bearing)))
            if a_prev - a_best <= p['hysteresis']:
                best = kp
    else:
        best = int(np.argmax(free))
        if kp is not None and free[kp] >= free[best] - 1e-12:
            best = kp
    return float(bearings[best]), float(free[best]), bool(reached[best])


def straight_blocked(pos, target, world, step=0.05):
    """True if the straight segment pos->target leaves free space for the robot."""
    pos = np.asarray(pos, float)
    target = np.asarray(target, float)
    dist = float(np.hypot(*(target - pos)))
    if dist < 1e-9 or not world.obstacles:
        return False
    n = int(np.floor(dist / step + 1e-9))
    r = np.append(step * np.arange(1, n + 1), dist)
    pts = pos[None, :] + (target - pos)[None, :] * (r / dist)[:, None]
    cen = np.array([o.p for o in world.obstacles])
    rad = np.array([o.r for o in world.obstacles])
    clr = (np.linalg.norm(pts[:, None, :] - cen[None, :, :], axis=2)
           - rad[None, :]).min(axis=1)
    return bool((clr < world.robot_radius).any())


# --------------------------------------------------------------------------- #
# 3. Frozen-waypoint hopping
# --------------------------------------------------------------------------- #
class WaypointHopper:
    """Frozen escape waypoint (DESIGN_ADVICE 2, defect 4b).

    The waypoint lies on the selected bearing at
    ``min(free - robot_radius - waypoint_margin, waypoint_cap)``. It is frozen
    at selection and re-selected when the robot is within ``reselect_radius`` of
    it, when the straight segment to it is blocked, or after
    ``reselect_timeout_s``. Each selection is a hop. With ``hop=False`` the
    waypoint is re-selected every step (the v1 actuation, v2 rule).
    """

    def __init__(self, p, dt):
        self.p = p
        self.dt = dt
        self.timeout_steps = int(round(p['reselect_timeout_s'] / dt))
        self.hops = 0
        self.log = []          # (step, bearing, free, reached, reason)
        self.release()

    def release(self):
        self.wp = None
        self.prev_bearing = None
        self._t_sel = 0

    def _select(self, step, pos, goal_bearing, world, reason):
        p = self.p
        if p['gap_rule'] == 'horizon':
            b, free, reached = select_opening(pos, goal_bearing, world, p,
                                              self.prev_bearing)
            d = float(np.clip(free - world.robot_radius - p['waypoint_margin'],
                              0.0, p['waypoint_cap']))
        else:                                   # paper-v1 selection (flag off)
            import se_mppi_proto as se
            b = se.find_escape_gap(pos, goal_bearing, world,
                                   min_clearance=2.4,
                                   prev_bearing=self.prev_bearing)
            free, reached = float('nan'), False
            d = 2.5
            if b is None:
                self.prev_bearing = None
                self.wp = None
                return None
        self.prev_bearing = b
        self.wp = np.asarray(pos, float) + d * np.array([np.cos(b), np.sin(b)])
        self._t_sel = step
        self.hops += 1
        self.log.append((step, b, free, reached, reason))
        return self.wp

    def update(self, step, pos, goal_bearing, world):
        """Return the current escape subgoal (re-selecting if a rule fires)."""
        p = self.p
        pos = np.asarray(pos, float)
        if self.wp is None:
            reason = 'first'
        elif not p['hop']:
            reason = 'every_step'
        elif np.hypot(*(self.wp - pos)) < p['reselect_radius']:
            reason = 'reached'
        elif straight_blocked(pos, self.wp, world):
            reason = 'blocked'
        elif step - self._t_sel >= self.timeout_steps:
            reason = 'timeout'
        else:
            return self.wp
        wp = self._select(step, pos, goal_bearing, world, reason)
        return wp
