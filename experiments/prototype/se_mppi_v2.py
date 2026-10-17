# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""SE-MPPI v2 escape logic for the 2D mirror (opt-in; paper-v1 is the default).

The v1 mirror differs from the shipped plugin at three places: the detector
input (distance to the goal), the opening selection (closest-to-goal among rays
that clear 2.4 m of 3.0 m), and the escape actuation (a rolling 2.5 m subgoal).
This module holds the v2 replacements. It is a transcription of the plugin's
C++ (``src/nav2_se_controller``), not of the design document: where the plugin
adds a rule that DESIGN_ADVICE does not have, the mirror has it too, behind one
flag (``escape_extras``) so that both algorithms can be run.

  1. ``PathProgressTracker`` / ``AnchorProgress``  progress on a per-task anchor path with
                            simulated 1 Hz straight-line replans from the robot pose.
                            Line for line ``path_progress.cpp`` (signed advance along the chain of
                            candidates, a chain restart at every anchor event, an arc that
                            continues backwards behind the first pose).
  2. ``select_horizon_gap``  horizon-ray opening selection: ``opening_search.cpp``
                            ``selectHorizonGap`` (horizon rays, minimum clearance, hysteresis,
                            avoid list with a retry without it).
  3. ``WaypointHopper``     frozen waypoint: ``safe_escape_controller.cpp`` ``manageEscapePath``
                            (re-select at the waypoint radius, at a timeout, on a movement
                            watchdog, when the straight line is blocked; stall pose, failed
                            waypoints and recent hop origins are avoided; a waypoint closer than
                            0.2 m is not taken).
  4. all rollouts collide   a zero command; the switch lives in
                            ``se_mppi_proto.MPPI.step(zero_on_all_collide=...)``.
  5. ``act_on_previous_cycle``  the plugin decides the escape at the start of a cycle from the
                            detector state at the end of the previous one (one cycle of lag).

Extras of the plugin beyond DESIGN_ADVICE section 2 (docs/architecture/2026-10_escape-v2.md
section 3), all behind ``escape_extras``:

  (a) rays that lead back to the stall pose, to recent hop origins or to failed waypoints are
      excluded (``avoid_radius``, ``avoid_hop_radius``, ``avoid_hops``, ``failed_max``);
  (b) a movement watchdog (``stall_time_s`` / ``stall_distance``);
  (c) a floor on the waypoint distance (``min_waypoint``);
  (d) a minimum clearance of horizon candidates (``gap_min_clearance``).

``escape_extras=False`` is the design as advised. The Python code is checked against the C++
by ``experiments/benchmark2d/tests/test_v2_cxx_parity.py`` (through ``parity_shim``).
"""

from __future__ import annotations

import numpy as np

V2_PARAMS = dict(
    # --- the switches (default of the v2 set = all on) ---
    progress='anchor',            # 'goal_distance' = paper-v1 detector input
    gap_rule='horizon',           # 'closest_viable' = paper-v1 selection
    hop=True,                     # False = re-select every step (v1 actuation)
    zero_on_all_collide=True,     # False = v1: keep the best colliding rollout
    act_on_previous_cycle=True,   # the plugin's one cycle of lag between detection and escape
    escape_extras=True,           # False = the design as advised (no rule (a) to (d) below)
    # --- detector / progress (DESIGN_ADVICE 2 defect 3, 4.2) ---
    stall_window=30,              # cycles (3 s at dt 0.1)
    progress_min_advance=0.10,    # m, one progress event
    progress_adopt_advance=0.5,   # m, a replan becomes the anchor
    replan_period_s=1.0,          # the BT's 1 Hz replan from the robot pose
    goal_reached_tolerance=0.5,   # m, no entrapment this close to the goal (se_goal_reached_tolerance)
    # --- opening selection (defect 4a) ---
    gap_num_rays=36,
    gap_max_range=2.0,            # m
    window_half=1.5,              # m, half of the 3 m rolling costmap
    hysteresis=0.5,               # rad
    gap_min_clearance=0.6,        # m, extra (d): horizon candidates need it (se_gap_min_clearance)
    # --- waypoint (defect 4b) ---
    waypoint_cap=2.5,             # m
    waypoint_margin=0.05,         # m, subtracted with the robot radius
    reselect_radius=0.4,          # m
    reselect_timeout_s=15.0,      # s
    # --- extras of the plugin (escape_extras) ---
    min_waypoint=0.2,             # m, extra (c)
    stall_time_s=3.0,             # s, extra (b) (se_escape_stall_time)
    stall_distance=0.15,          # m, extra (b) (se_escape_stall_distance)
    avoid_radius=1.0,             # m, extra (a): the stall pose (se_escape_avoid_radius)
    avoid_hop_radius=0.6,         # m, extra (a): failed waypoints, hop origins
    avoid_hops=3,                 # extra (a): how many hop origins are remembered
    failed_max=6,                 # extra (a): how many failed waypoints are remembered
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
# 1. Anchor-path progress (path_progress.cpp)
# --------------------------------------------------------------------------- #
class PolylinePath:
    """A planar polyline with cumulative arc length (``PathProgressTracker::Poly``)."""

    def __init__(self, pts):
        pts = np.asarray(pts, float).reshape(-1, 2)
        pts = pts[np.isfinite(pts).all(axis=1)]
        self.pts = pts
        if len(pts) >= 2:
            d = np.diff(pts, axis=0)
            self.seg_len = np.hypot(d[:, 0], d[:, 1])
            self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        else:
            self.seg_len = np.zeros(0)
            self.cum = np.zeros(len(pts))

    def __len__(self):
        return len(self.pts)

    @property
    def length(self):
        return float(self.cum[-1]) if len(self.cum) else 0.0

    def nearest_arc(self, p):
        """Arc length at the path point nearest to ``p`` (``arcAtNearest``).

        Behind the first pose the arc continues backwards (negative), so that motion away from
        the start of a path counts as negative advance. After the last pose it is clamped.
        A path with fewer than two poses has arc 0.
        """
        if len(self.pts) < 2:
            return 0.0
        p = np.asarray(p, float)
        a = self.pts[:-1]
        d = self.pts[1:] - a
        len2 = self.seg_len ** 2
        safe = np.where(len2 > 0.0, len2, 1.0)
        t_raw = np.where(len2 > 0.0, ((p - a) * d).sum(axis=1) / safe, 0.0)
        t = np.clip(t_raw, 0.0, 1.0)
        e = p - (a + t[:, None] * d)
        d2 = (e ** 2).sum(axis=1)
        i = int(np.argmin(d2))                       # first of equals, as the strict '<'
        t_arc = t_raw[i] if (i == 0 and t_raw[i] < 0.0) else t[i]
        return float(self.cum[i] + t_arc * self.seg_len[i])


class PathProgressTracker:
    """``PathProgressTracker`` of ``path_progress.cpp``, statement for statement.

    An ANCHOR path per task (the first plan, with cumulative arc length); a progress event fires
    when the arc length at the robot's nearest anchor point exceeds the best so far by
    ``min_advance``. A same-goal replan is a CANDIDATE: the signed advance of the robot along the
    successive candidates (a chain, carried over replans) is measured, and at ``adopt_advance``
    the latest candidate becomes the anchor and an event fires.
    """

    def __init__(self, min_advance=0.10, adopt_advance=0.5):
        self.min_advance = float(min_advance)
        self.adopt_advance = float(adopt_advance)
        self.reset()

    def reset(self):
        self.anchor = PolylinePath([])
        self.candidate = PolylinePath([])
        self.candidate_pending_base = False
        self.candidate_base = 0.0
        self.carried = 0.0
        self.chain = 0.0
        self.best_arc = 0.0
        self.has_best = False
        self.has_last = False
        self.last = (0.0, 0.0)
        self.events = 0
        self.adoptions = 0

    def has_anchor(self):
        return len(self.anchor.cum) > 0

    def start_task(self, pts):
        self.reset()
        self.anchor = PolylinePath(pts)

    def offer_replan(self, pts):
        poly = PolylinePath(pts)
        if len(poly) == 0:
            return
        # Close the old candidate: its signed advance up to the last known position joins the chain.
        if len(self.candidate.cum) and not self.candidate_pending_base and self.has_last:
            self.carried += self.candidate.nearest_arc(self.last) - self.candidate_base
        self.candidate = poly
        self.candidate_pending_base = True      # the new path starts where the robot is

    def update(self, xy):
        x, y = float(xy[0]), float(xy[1])
        if not (np.isfinite(x) and np.isfinite(y)) or not self.has_anchor():
            return self.events
        self.last = (x, y)
        self.has_last = True
        arc = self.anchor.nearest_arc(self.last)
        if not self.has_best:
            self.best_arc = arc                 # baseline: the first position is not progress
            self.has_best = True
        if len(self.candidate.cum):
            cand_arc = self.candidate.nearest_arc(self.last)
            if self.candidate_pending_base:
                self.candidate_base = cand_arc
                self.candidate_pending_base = False
            self.chain = self.carried + (cand_arc - self.candidate_base)
            if self.chain >= self.adopt_advance:
                self.anchor = self.candidate
                self.candidate = PolylinePath([])
                self.candidate_pending_base = False
                self.carried = 0.0
                self.chain = 0.0
                self.best_arc = self.anchor.nearest_arc(self.last)
                self.events += 1
                self.adoptions += 1
                return self.events
        else:
            self.chain = self.carried
        if arc - self.best_arc >= self.min_advance:
            self.best_arc = arc
            self.events += 1
            self.carried = 0.0                  # the chain restarts from here
            self.chain = 0.0
            if len(self.candidate.cum):
                self.candidate_base = self.candidate.nearest_arc(self.last)
        return self.events


class AnchorProgress:
    """The tracker driven the way the benchmark drives it: the first plan is the straight path
    from the start to the goal, the BT replans a straight path from the robot every second.

    ``events`` is a monotone counter; the (unchanged) ``EntrapmentDetector`` receives
    ``-events`` so that each event is a new minimum.
    """

    def __init__(self, start_xy, goal_xy, min_advance=0.10, adopt_advance=0.5):
        self.tracker = PathProgressTracker(min_advance, adopt_advance)
        self.tracker.start_task([start_xy, goal_xy])
        self.replans = 0

    def replan(self, pose_xy, goal_xy):
        """A fresh straight path from the robot pose (the BT's 1 Hz replan)."""
        self.tracker.offer_replan([pose_xy, goal_xy])
        self.replans += 1

    def update(self, pose_xy):
        return self.tracker.update(pose_xy)

    @property
    def events(self):
        return self.tracker.events

    @property
    def adoptions(self):
        return self.tracker.adoptions

    @property
    def best(self):
        return self.tracker.best_arc


# --------------------------------------------------------------------------- #
# 2. Horizon-ray opening selection (opening_search.cpp)
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


def leads_to_avoided(bearing, clearance, avoid):
    """``leadsToAvoided``: the ray passes within ``radius`` of an avoided place ahead of the robot.

    ``avoid``: (bearing, distance, radius) tuples relative to the robot. A place is ignored while
    the robot is within its radius, when it is 90 degrees or more off the ray, or when it lies
    beyond the end of the ray by more than the radius.
    """
    for a_bearing, a_distance, a_radius in avoid:
        if a_distance <= a_radius:
            continue
        delta = abs(float(wrap(bearing - a_bearing)))
        if delta >= np.pi / 2.0:
            continue
        if a_distance > clearance + a_radius:
            continue
        if a_distance * np.sin(delta) < a_radius:
            return True
    return False


class Gap:
    """Result of ``select_horizon_gap`` (``HorizonGap``)."""

    __slots__ = ('found', 'bearing', 'clearance', 'horizon', 'kept_previous', 'candidates')

    def __init__(self, found=False, bearing=0.0, clearance=0.0, horizon=False,
                 kept_previous=False, candidates=0):
        self.found = found
        self.bearing = bearing
        self.clearance = clearance
        self.horizon = horizon
        self.kept_previous = kept_previous
        self.candidates = candidates

    def as_tuple(self):
        return (self.found, self.bearing, self.clearance, self.horizon, self.kept_previous,
                self.candidates)


def _select_once(bearings, free, reached, goal_bearing, min_clearance, prev_bearing,
                 hysteresis, avoid):
    n = len(bearings)
    if n == 0:
        return Gap()
    avoided = [leads_to_avoided(bearings[k], free[k], avoid) for k in range(n)]
    align = np.abs(wrap(bearings - goal_bearing))
    best, best_align, cands = None, np.inf, 0
    for k in range(n):
        if not reached[k] or free[k] < min_clearance or avoided[k]:
            continue
        cands += 1
        if align[k] < best_align:                  # strict: first of equals
            best, best_align = k, align[k]
    if best is None:
        # None reaches the horizon: the longest ray (ties: closest to the goal bearing).
        longest, longest_align = None, np.inf
        for k in range(n):
            if avoided[k]:
                continue
            if (longest is None or free[k] > free[longest] + 1e-9 or
                    (abs(free[k] - free[longest]) <= 1e-9 and align[k] < longest_align)):
                longest, longest_align = k, align[k]
        if longest is None or free[longest] < min_clearance:
            return Gap(candidates=cands)
        return Gap(True, float(bearings[longest]), float(free[longest]), False, False, cands)
    if prev_bearing is not None:
        dprev = np.abs(wrap(bearings - prev_bearing))
        kp = int(np.argmin(dprev))                 # first of equals
        if reached[kp] and free[kp] >= min_clearance and not avoided[kp]:
            if not (best_align < align[kp] - hysteresis):
                return Gap(True, float(bearings[kp]), float(free[kp]), True, True, cands)
    return Gap(True, float(bearings[best]), float(free[best]), True, False, cands)


def select_horizon_gap(bearings, free, reached, goal_bearing, min_clearance, prev_bearing,
                       hysteresis, avoid=()):
    """``selectHorizonGap``: the horizon ray closest to the goal bearing (hysteresis keeps the
    previous one unless the best is closer by more than ``hysteresis`` rad); the longest ray if
    none reaches the horizon; rays that lead to an avoided place are not candidates, and if that
    leaves nothing the selection is repeated without the avoid list."""
    out = _select_once(bearings, free, reached, goal_bearing, min_clearance, prev_bearing,
                       hysteresis, avoid)
    if not out.found and len(avoid):
        out = _select_once(bearings, free, reached, goal_bearing, min_clearance, prev_bearing,
                           hysteresis, ())
    return out


def select_opening(pos, goal_bearing, world, p, prev_bearing=None, avoid=()):
    """Cast the rays of the analytic world and select (horizon rule, DESIGN_ADVICE 2 defect 4a).

    ``p['gap_min_clearance']`` and ``avoid`` apply only with ``escape_extras`` (extras (d), (a)).
    Returns a ``Gap``.
    """
    bearings, free, reached = scan_rays(pos, world, p['gap_num_rays'], p['gap_max_range'],
                                        p['window_half'])
    extras = p['escape_extras']
    return select_horizon_gap(bearings, free, reached, goal_bearing,
                              p['gap_min_clearance'] if extras else 0.0, prev_bearing,
                              p['hysteresis'], avoid if extras else ())


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
# 3. Frozen-waypoint hopping (SafeEscapeController::manageEscapePath)
# --------------------------------------------------------------------------- #
class WaypointHopper:
    """Frozen escape waypoint, as ``manageEscapePath`` keeps it.

    The waypoint lies on the selected bearing at ``min(free - robot_radius - waypoint_margin,
    waypoint_cap)``. It is frozen at selection and re-selected, in this order, when the robot is
    within ``reselect_radius`` of it (inclusive), after ``reselect_timeout_s``, when the robot has
    not moved ``stall_distance`` in ``stall_time_s`` (extra (b)), or when the straight segment to
    it is blocked. A waypoint that did not work is remembered (``failed_max``), as are the stall
    pose and the last ``avoid_hops`` hop origins; no later hop leads toward them (extra (a)). A
    candidate waypoint closer than ``min_waypoint`` is not taken (extra (c)): the state stays and
    the next step tries again. With ``hop=False`` the waypoint is re-selected every step (the v1
    actuation).

    ``mark_entry(pos)`` is called at the step where the detector declares entrapment (the plugin
    records the stall pose then); ``release()`` when the escape ends.
    """

    def __init__(self, p, dt):
        self.p = p
        self.dt = dt
        freq = 1.0 / dt
        self.cycle_limit = max(1, int(round(p['reselect_timeout_s'] * freq)))
        self.stall_limit = max(1, int(round(p['stall_time_s'] * freq)))
        self.hops = 0
        self.log = []          # (step, bearing, free, reached, reason)
        self.release()

    def release(self):
        """``escape_ = EscapeState{}``: the episode is over (the stall pose goes too)."""
        self.wp = None
        self.active = False
        self.prev_bearing = None
        self.cycles = 0
        self.hop = 0
        self.origins = []
        self.failed = []
        self.ref = (0.0, 0.0)
        self.ref_cycles = 0
        self.has_stall = False
        self.stall = (0.0, 0.0)

    def mark_entry(self, pos):
        self.has_stall = True
        self.stall = (float(pos[0]), float(pos[1]))

    def _avoid(self, pos):
        p = self.p
        out = []

        def add(xy, radius):
            dx, dy = xy[0] - pos[0], xy[1] - pos[1]
            out.append((float(np.arctan2(dy, dx)), float(np.hypot(dx, dy)), radius))
        if self.has_stall:
            add(self.stall, p['avoid_radius'])
        for f in self.failed:
            add(f, p['avoid_hop_radius'])
        for o in self.origins:
            add(o, p['avoid_hop_radius'])
        return out

    def _select(self, step, pos, goal_bearing, world, reason):
        p = self.p
        extras = p['escape_extras']
        pos = np.asarray(pos, float)
        if p['gap_rule'] == 'horizon':
            gap = select_opening(pos, goal_bearing, world, p, self.prev_bearing,
                                 self._avoid(pos) if extras else ())
            if not gap.found:
                return self.wp
            d = min(gap.clearance - world.robot_radius - p['waypoint_margin'], p['waypoint_cap'])
            if extras:
                if d < p['min_waypoint']:
                    return self.wp             # not taken: nothing changes, the next step retries
            else:
                d = max(d, 0.0)
            b, free, reached = gap.bearing, gap.clearance, gap.horizon
        else:                                   # paper-v1 selection (flag off)
            import se_mppi_proto as se
            b = se.find_escape_gap(pos, goal_bearing, world, min_clearance=2.4,
                                   prev_bearing=self.prev_bearing)
            free, reached = float('nan'), False
            d = 2.5
            if b is None:
                self.prev_bearing = None
                self.wp = None
                self.active = False
                return None
        self.prev_bearing = b
        self.wp = pos + d * np.array([np.cos(b), np.sin(b)])
        self.active = True
        self.cycles = 0
        self.ref = (float(pos[0]), float(pos[1]))
        self.ref_cycles = 0
        self.hop += 1
        self.origins.append((float(pos[0]), float(pos[1])))
        while len(self.origins) > max(int(p['avoid_hops']), 0):
            self.origins.pop(0)
        self.hops += 1
        self.log.append((step, b, free, reached, reason))
        return self.wp

    def update(self, step, pos, goal_bearing, world):
        """Return the current escape subgoal (re-selecting if a rule fires)."""
        p = self.p
        pos = np.asarray(pos, float)
        need, why = not self.active, 'first'
        if self.active:
            self.cycles += 1
            if not p['hop']:
                need, why = True, 'every_step'
            elif np.hypot(*(self.wp - pos)) <= p['reselect_radius']:
                need, why = True, 'reached'
            elif self.cycles >= self.cycle_limit:
                need, why = True, 'timeout'
            else:
                stalled = False
                if p['escape_extras']:
                    if np.hypot(pos[0] - self.ref[0], pos[1] - self.ref[1]) >= p['stall_distance']:
                        self.ref = (float(pos[0]), float(pos[1]))
                        self.ref_cycles = 0
                    else:
                        self.ref_cycles += 1
                    stalled = self.ref_cycles >= self.stall_limit
                if stalled:
                    need, why = True, 'no_movement'
                elif straight_blocked(pos, self.wp, world):
                    need, why = True, 'blocked'
            if need and why not in ('reached', 'every_step'):
                self.failed.append((float(self.wp[0]), float(self.wp[1])))
                while len(self.failed) > int(p['failed_max']):
                    self.failed.pop(0)
        if not need:
            return self.wp
        return self._select(step, pos, goal_bearing, world, why)
