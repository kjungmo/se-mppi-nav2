# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for the SE-MPPI v2 escape logic in the 2D mirror (opt-in flags).

Covers: anchor-path progress (the Python analogue of the root-cause scratch
test), horizon-ray opening selection, frozen-waypoint hopping, the "all
rollouts collide -> zero command" mode, and that every flag off reproduces the
paper-v1 loop exactly. The mirror transcribes the plugin's C++ (extras included);
``test_v2_cxx_parity.py`` checks it against the C++ itself.
"""

import os
import sys

import numpy as np
import pytest

from experiments.benchmark2d import scenarios as sc
from experiments.benchmark2d.configs import CONFIGS, make_v2_configs
from experiments.benchmark2d.rollout import rollout

_PROTO = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'prototype')
if _PROTO not in sys.path:
    sys.path.insert(0, _PROTO)
import se_mppi_proto as se  # noqa: E402
import se_mppi_v2 as v2m  # noqa: E402

P = v2m.v2_params()
P_DESIGN = v2m.v2_params(escape_extras=False)       # the design as advised, without the extras
STEP = 2 * np.pi / P['gap_num_rays']


def _open(*args, **kw):
    """select_opening as the older (bearing, clearance, horizon) triple."""
    g = v2m.select_opening(*args, **kw)
    return g.bearing, g.clearance, g.horizon


# --------------------------------------------------------------------------- #
# Anchor-path progress
# --------------------------------------------------------------------------- #
def _drive(tr, det, xs, replan_every=None, y=0.0, goal=(2.0, 0.0)):
    """Feed robot x positions; optionally replan from the pose every N cycles."""
    for i, x in enumerate(xs):
        if replan_every and i > 0 and i % replan_every == 0:
            tr.replan((x, y), goal)
        det.update(-float(tr.update((x, y))))


def _stalled_tracker(window=5):
    """Robot drives x = 0 -> 1.0 on a path to (2, 0), backs to 0.6 and stalls."""
    tr = v2m.AnchorProgress((0.0, 0.0), (2.0, 0.0), 0.10, 0.5)
    det = se.EntrapmentDetector(stall_window=window)
    _drive(tr, det, [0.05 * k for k in range(21)])           # reaches x = 1.0
    _drive(tr, det, [0.6] * 10)                              # backed off, stalled
    return tr, det


def test_replan_from_the_robot_pose_does_not_clear_entrapment():
    # ROOT_CAUSE scratch test, mirrored: v1-style re-anchoring let one cell of
    # motion along a fresh path clear the stall. The anchor does not.
    tr, det = _stalled_tracker()
    assert det.entrapped
    best = tr.best
    tr.replan((0.6, 0.0), (2.0, 0.0))                        # 1 Hz replan
    det.update(-float(tr.update((0.6, 0.0))))
    assert det.entrapped
    det.update(-float(tr.update((0.65, 0.0))))               # one 0.05 m cell
    assert det.entrapped
    assert tr.best == pytest.approx(best)                    # anchor untouched


def test_progress_needs_min_advance_beyond_the_furthest_point():
    tr, det = _stalled_tracker()
    assert det.entrapped
    det.update(-float(tr.update((1.03, 0.0))))               # 0.03 m past 1.0
    assert det.entrapped
    det.update(-float(tr.update((1.15, 0.0))))               # 0.15 m past 1.0
    assert not det.entrapped


def test_exactly_at_the_min_advance_threshold():
    # path_progress.cpp fires at `arc - best >= min_advance` (inclusive). Dyadic numbers: exact.
    tr = v2m.AnchorProgress((0.0, 0.0), (2.0, 0.0), 0.125, 0.5)
    tr.update((1.0, 0.0))                                    # baseline, not progress
    assert tr.events == 0
    tr.update((1.125 - 2.0 ** -20, 0.0))                     # just under
    assert tr.events == 0
    tr.update((1.125, 0.0))                                  # advance == min_advance: event
    assert tr.events == 1


def test_creeping_with_replans_never_stalls():
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    det = se.EntrapmentDetector(stall_window=30)
    _drive(tr, det, [0.025 * k for k in range(1, 230)], replan_every=10,
           goal=(6.0, 0.0))
    assert not det.entrapped


def test_following_replans_adopts_the_candidate():
    # A detour followed at 0.3 m/s with a replan every second adopts: no single replan reaches
    # 0.5 m, the chain of the signed advances does. The same case as the C++ test
    # PathProgressTracker.TheChainCarriesAcrossReplansAtOneHertz (the detour leaves the anchor,
    # so no anchor event restarts the chain).
    tr = v2m.PathProgressTracker(0.10, 0.5)
    tr.start_task([(0, 0), (5, 0)])
    tr.update((1.0, 0.0))
    y = 0.0
    for _ in range(4):
        tr.offer_replan([(1.0, y), (1.0, 2.0), (5.0, 2.0), (5.0, 0.0)])
        for _ in range(10):
            y += 0.03
            tr.update((1.0, y))
    assert tr.adoptions == 1


def test_motion_along_the_anchor_never_adopts_because_events_restart_the_chain():
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    x = 0.0
    for _ in range(4):
        tr.replan((x, 0.0), (6.0, 0.0))
        for _ in range(10):
            x += 0.03
            tr.update((x, 0.0))
    assert tr.adoptions == 0 and tr.events >= 3


def test_rocking_in_place_never_adopts():
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    tr.update((1.0, 0.0))
    for k in range(40):                                      # +-0.3 m every second
        x = 0.7 + (0.3 if k % 2 else 0.0)
        tr.replan((x, 0.0), (6.0, 0.0))
        for j in range(10):
            tr.update((x + (0.03 * j if k % 2 == 0 else -0.03 * j), 0.0))
    assert tr.adoptions == 0


def test_adoption_threshold_is_inclusive_and_the_chain_carries_every_candidate():
    # dyadic numbers: every arc length below is exact in floating point. The anchor's best is held
    # at 1.0 by driving the robot to 1.0 first, so only the chain can adopt.
    def fresh():
        tr = v2m.AnchorProgress((0.0, 0.0), (8.0, 0.0), 0.5, 0.5)
        tr.update((1.0, 0.0))                                # baseline arc 1.0
        return tr
    # one candidate: chain == adopt_advance exactly -> adopted (inclusive)
    tr = fresh()
    tr.replan((1.0, 0.0), (8.0, 0.0))
    tr.update((1.0, 0.0))
    tr.update((1.25, 0.0))
    assert tr.adoptions == 0 and tr.tracker.chain == 0.25
    # a second candidate: the advance of the first (0.25) joins the chain; 0.25 + 0.25 adopts
    tr.replan((1.25, 0.0), (8.0, 0.0))
    tr.update((1.25, 0.0))
    tr.update((1.4375, 0.0))
    assert tr.adoptions == 0 and tr.tracker.chain == 0.4375
    tr.update((1.5, 0.0))
    assert tr.adoptions == 1                                  # chain == 0.25 + 0.25 == 0.5
    # the signed advance is carried: moving backwards along a candidate lowers the chain
    tr = fresh()
    tr.replan((1.0, 0.0), (8.0, 0.0))
    tr.update((1.0, 0.0))
    tr.update((0.5, 0.0))                                     # -0.5 along the candidate
    tr.replan((0.5, 0.0), (8.0, 0.0))
    tr.update((0.5, 0.0))
    tr.update((1.0, 0.0))                                     # +0.5 on the new one, -0.5 carried
    assert tr.adoptions == 0 and tr.tracker.chain == 0.0


def test_backing_off_and_returning_earns_no_advance():
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    tr.update((1.0, 0.0))
    tr.replan((1.0, 0.0), (6.0, 0.0))
    for x in (1.0, 1.45, 1.0):                               # forward 0.45, back to 0
        tr.update((x, 0.0))
    tr.replan((1.0, 0.0), (6.0, 0.0))
    tr.update((1.06, 0.0))
    assert tr.adoptions == 0


def test_an_anchor_event_restarts_the_chain():
    tr = v2m.AnchorProgress((0.0, 0.0), (8.0, 0.0), 0.25, 1.0)
    tr.update((0.0, 0.0))
    tr.replan((0.0, 0.0), (8.0, 0.0))
    tr.update((0.0, 0.0))
    tr.update((0.25, 0.0))                                   # anchor event: chain restarts at 0
    assert tr.events == 1 and tr.tracker.chain == 0.0
    tr.update((0.375, 0.0))
    assert tr.tracker.chain == 0.125


def test_polyline_nearest_arc_and_degenerate_path():
    path = v2m.PolylinePath([(0, 0), (1, 0), (1, 1)])
    assert path.length == pytest.approx(2.0)
    assert path.nearest_arc((0.5, 0.3)) == pytest.approx(0.5)
    assert path.nearest_arc((1.2, 0.5)) == pytest.approx(1.5)
    # behind the first pose the arc continues backwards (path_progress.cpp, arcAtNearest)
    assert path.nearest_arc((-3, 0)) == pytest.approx(-3.0)
    assert path.nearest_arc((9, 9)) == pytest.approx(2.0)     # clamped after the end
    deg = v2m.PolylinePath([(1, 1), (1, 1)])                  # robot on the goal
    assert deg.length == 0.0 and deg.nearest_arc((5, 5)) == 0.0
    assert v2m.PolylinePath([(1, 1)]).nearest_arc((5, 5)) == 0.0   # fewer than two poses
    assert v2m.PolylinePath([(0, 0), (np.nan, 1), (2, 0)]).length == pytest.approx(2.0)


# --------------------------------------------------------------------------- #
# Opening selection
# --------------------------------------------------------------------------- #
def _pocket(back_x=1.2, half_w=0.85, depth=1.8):
    """Canonical pocket: opening faces -x, goal behind the back wall (+x)."""
    obs = []
    obs += sc._wall((back_x, -half_w), (back_x, half_w))
    obs += sc._wall((back_x, half_w), (back_x - depth, half_w))
    obs += sc._wall((back_x, -half_w), (back_x - depth, -half_w))
    return se.World(obs, goal=(3.0, 0.0), robot_radius=0.22)


def _angdiff(a, b):
    return abs(float(v2m.wrap(a - b)))


def test_horizon_rule_picks_the_mouth_where_closest_to_goal_picks_a_side():
    world = _pocket()
    pos = np.array([0.35, 0.1])                              # 0.3 m short of the wall face
    gb = float(np.arctan2(0.0 - pos[1], 3.0 - pos[0]))
    # the diagnosed C++ behaviour: closest to the goal among rays >= 0.6 m
    old = se.find_escape_gap(pos, gb, world, num_rays=36, max_range=2.0,
                             min_clearance=0.6)
    assert old is not None and _angdiff(old, np.pi) > np.radians(20), old
    # the v2 rule: only rays that leave through the mouth reach the horizon
    b, free, reached = _open(pos, gb, world, P)
    assert reached
    assert _angdiff(b, np.pi) <= np.radians(20)


def test_open_world_takes_the_ray_closest_to_the_goal():
    world = se.World([], goal=(3.0, 0.0))
    b, free, reached = _open((0, 0), 0.3, world, P)
    assert reached and _angdiff(b, 0.3) <= STEP / 2 + 1e-9


def test_hysteresis_equal_to_the_improvement_keeps_the_previous_ray():
    world = se.World([], goal=(3.0, 0.0))
    kp = 21                                                   # 3 steps off the goal ray
    prev = -np.pi + 2 * np.pi * kp / P['gap_num_rays']
    gain = abs(float(v2m.wrap(prev - 0.0)))                   # a_prev - a_best exactly
    p = v2m.v2_params(hysteresis=gain)
    b, _, _ = _open((0, 0), 0.0, world, p, prev_bearing=prev)
    assert b == pytest.approx(v2m.wrap(prev))                 # equal: keep
    p = v2m.v2_params(hysteresis=np.nextafter(gain, -np.inf))
    b, _, _ = _open((0, 0), 0.0, world, p, prev_bearing=prev)
    assert b == pytest.approx(0.0)                            # just better: switch


def test_ray_stops_only_when_clearance_is_below_the_robot_radius():
    # clearance == robot radius at r = 0.5 is still free; blocked from r = 0.55
    world = se.World([se.Obstacle(1.0, 0.0, 0.25)], goal=(3.0, 0.0), robot_radius=0.25)
    _, free, reached = v2m.scan_rays((0.0, 0.0), world, 36, 2.0, 1.5)
    assert free[18] == pytest.approx(0.55) and not reached[18]


def test_hysteresis_boundary():
    world = se.World([], goal=(3.0, 0.0))
    ks = P['gap_num_rays'] / 2                                # ray k=18 is bearing 0
    assert ks == 18
    # previous ray 2 steps off (0.349 rad < 0.5 better): kept
    prev_keep = 2 * STEP
    b, _, _ = _open((0, 0), 0.0, world, P, prev_bearing=prev_keep)
    assert b == pytest.approx(v2m.wrap(prev_keep))
    # previous ray 3 steps off (0.524 rad > 0.5 better): switched to the goal ray
    b, _, _ = _open((0, 0), 0.0, world, P, prev_bearing=3 * STEP)
    assert b == pytest.approx(0.0)
    # sign and wrap-around: previous ray across the +-pi seam
    b, _, _ = _open((0, 0), np.pi, world, P, prev_bearing=-np.pi + STEP)
    assert abs(float(v2m.wrap(b - np.pi))) <= STEP + 1e-9


def test_previous_ray_that_no_longer_reaches_the_horizon_is_dropped():
    world = _pocket()
    pos = np.array([0.35, 0.1])
    gb = 0.0
    # bearing 0 hits the back wall: even a huge hysteresis cannot keep it
    p = v2m.v2_params(hysteresis=10.0)
    b, _, reached = _open(pos, gb, world, p, prev_bearing=0.0)
    assert reached and _angdiff(b, np.pi) <= np.radians(20)


def test_no_ray_reaches_the_horizon_takes_the_longest():
    # a closed ring of obstacles with one side pushed out: nothing reaches 2 m
    ring = [se.Obstacle(1.2 * np.cos(a), 1.2 * np.sin(a), 0.3)
            for a in np.linspace(0, 2 * np.pi, 40, endpoint=False)]
    ring = [o for o in ring if not (abs(o.p[1]) < 0.4 and o.p[0] > 0)]
    ring += [se.Obstacle(1.9, 0.0, 0.3)]                      # far plug on +x
    world = se.World(ring, goal=(3.0, 0.0))
    b, free, reached = _open((0, 0), np.pi, world, P)
    assert not reached
    bearings, lengths, rch = v2m.scan_rays((0, 0), world, 36, 2.0, 1.5)
    assert not rch.any() and free == pytest.approx(lengths.max())


def test_window_edge_counts_as_horizon():
    # a wall 1.8 m away on +x lies outside the 1.5 m half window: the +x ray is
    # not blocked (the C++ costmap would not see it either)
    world = se.World([se.Obstacle(2.1, 0.0, 0.3)], goal=(3.0, 0.0))
    _, free, reached = v2m.scan_rays((0, 0), world, 36, 2.0, 1.5)
    k = 18
    assert reached[k] and free[k] == pytest.approx(1.5)
    # on the diagonal the 2.0 m range binds (window corner is 2.12 m away)
    bearings, free2, reached2 = v2m.scan_rays((0, 0), se.World([], goal=(0, 0)),
                                              72, 2.0, 1.5)
    kd = int(np.argmin(np.abs(bearings - np.pi / 4)))
    assert free2[kd] == pytest.approx(2.0) and reached2[kd]


def test_scan_rays_agrees_with_the_v1_ray_cast_on_blocked_rays():
    world = _pocket()
    pos = np.array([0.35, 0.1])
    bearings, free, reached = v2m.scan_rays(pos, world, 72, 3.0, 99.0)
    for k in range(0, 72, 3):
        d = se.ray_free_distance(pos, bearings[k], world, 3.0)
        assert free[k] == pytest.approx(d, abs=0.051), k


# --------------------------------------------------------------------------- #
# Waypoint hopping
# --------------------------------------------------------------------------- #
def test_waypoint_is_frozen_until_a_rule_fires():
    world = _pocket()
    hop = v2m.WaypointHopper(P, 0.1)
    pos = np.array([0.35, 0.1])
    wp0 = hop.update(0, pos, 0.0, world)
    assert hop.hops == 1
    # distance rule: min(free - robot radius - 0.05, 2.5) along the bearing
    _, b0, free0, reached0, _ = hop.log[0]
    assert reached0 and _angdiff(b0, np.pi) <= np.radians(20)
    d = np.hypot(*(wp0 - pos))
    assert d == pytest.approx(min(free0 - 0.22 - 0.05, 2.5))
    assert np.arctan2(*(wp0 - pos)[::-1]) == pytest.approx(b0)
    for step in range(1, 29):                                # the robot barely moves (< 3 s)
        wp = hop.update(step, pos + [0.0, 0.0], 0.0, world)
        assert wp is wp0 and hop.hops == 1


def test_reselect_when_within_radius_of_the_waypoint():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    wp0 = hop.update(0, np.zeros(2), 0.0, world).copy()
    hop.update(1, wp0 + [-0.41, 0.0], 0.0, world)            # 0.41 m away: still frozen
    assert hop.hops == 1
    hop.update(2, wp0 + [-0.39, 0.0], 0.0, world)            # 0.39 m away: hop
    assert hop.hops == 2 and hop.log[-1][4] == 'reached'


def test_reselect_when_the_straight_segment_is_blocked():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    world.obstacles.append(se.Obstacle(0.9, 0.0, 0.3))        # appears on the line
    hop.update(1, np.zeros(2), 0.0, world)
    assert hop.hops == 2 and hop.log[-1][4] == 'blocked'


def test_reselect_after_the_timeout_boundary():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P_DESIGN, 0.1)                  # no watchdog: only the timeout
    hop.update(0, np.zeros(2), 0.0, world)
    for k in range(1, 150):                                  # one call per control cycle
        hop.update(k, np.zeros(2), 0.0, world)               # the 149th call: 14.9 s
    assert hop.hops == 1
    hop.update(150, np.zeros(2), 0.0, world)                 # 15.0 s
    assert hop.hops == 2 and hop.log[-1][4] == 'timeout'


def test_release_forgets_the_waypoint_and_the_previous_bearing():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    hop.release()
    assert hop.wp is None and hop.prev_bearing is None
    hop.update(1, np.zeros(2), 1.0, world)
    assert hop.hops == 2 and hop.log[-1][4] == 'first'


def test_reselect_radius_boundary_is_inclusive():
    # manageEscapePath: `hypot(w - x) <= escape_reselect_radius_`
    world = se.World([], goal=(5.0, 0.0))
    probe = v2m.WaypointHopper(P, 0.1)
    wp = probe.update(0, np.zeros(2), 0.0, world).copy()
    d = float(np.hypot(*(wp - np.zeros(2))))
    hop = v2m.WaypointHopper(v2m.v2_params(reselect_radius=d), 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    hop.update(1, np.zeros(2), 0.0, world)                    # distance == radius: reselect
    assert hop.hops == 2 and hop.log[-1][4] == 'reached'
    hop = v2m.WaypointHopper(v2m.v2_params(reselect_radius=np.nextafter(d, -np.inf)), 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    hop.update(1, np.zeros(2), 0.0, world)                    # distance > radius: frozen
    assert hop.hops == 1


def test_waypoint_distance_is_capped():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(v2m.v2_params(gap_max_range=6.0, window_half=6.0), 0.1)
    wp = hop.update(0, np.zeros(2), 0.0, world)
    assert np.hypot(*wp) == pytest.approx(2.5)                # min(6 - 0.27, 2.5)


def _ring_world():
    ring = [se.Obstacle(0.45 * np.cos(a), 0.45 * np.sin(a), 0.2)
            for a in np.linspace(0, 2 * np.pi, 30, endpoint=False)]
    return se.World(ring, goal=(5.0, 0.0))


def test_waypoint_distance_floor_is_zero_in_the_design():
    # a ray blocked at 0.1 m: free - 0.22 - 0.05 < 0 is clipped to 0, not negative
    hop = v2m.WaypointHopper(P_DESIGN, 0.1)
    wp = hop.update(0, np.zeros(2), 0.0, _ring_world())
    assert np.hypot(*wp) == pytest.approx(0.0)


def test_a_waypoint_closer_than_the_minimum_is_not_taken():
    # extra (c): safe_escape_controller.cpp `gap.found && dist >= 0.2`; nothing changes, no hop
    hop = v2m.WaypointHopper(v2m.v2_params(gap_min_clearance=0.0), 0.1)
    assert hop.update(0, np.zeros(2), 0.0, _ring_world()) is None
    assert hop.hops == 0 and not hop.active


def test_the_minimum_waypoint_boundary():
    # a free ray of clearance c gives d = c - 0.22 - 0.05; d == 0.2 is taken, just under is not
    world = se.World([], goal=(5.0, 0.0))
    for wh, ok in ((0.2 + 0.22 + 0.05, True), (0.2 + 0.22 + 0.05 - 1e-6, False)):
        p = v2m.v2_params(gap_max_range=wh, window_half=wh, gap_min_clearance=0.0)
        hop = v2m.WaypointHopper(p, 0.1)
        assert (hop.update(0, np.zeros(2), 0.0, world) is not None) is ok, wh


def test_hop_off_reselects_every_step():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(v2m.v2_params(hop=False), 0.1)
    for s in range(5):
        hop.update(s, np.zeros(2), 0.0, world)
    assert hop.hops == 5


def test_the_movement_watchdog_fires_after_stall_time_and_not_before():
    # extra (b): no movement of stall_distance in stall_time_s (30 cycles at 0.1 s) re-selects
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    for k in range(1, 30):
        hop.update(k, np.zeros(2), 0.0, world)
    assert hop.hops == 1                                      # 29 stalled cycles
    hop.update(30, np.zeros(2), 0.0, world)                   # the 30th
    assert hop.hops == 2 and hop.log[-1][4] == 'no_movement'
    assert len(hop.failed) == 1                               # the waypoint that did not work


def test_the_watchdog_counter_restarts_after_stall_distance_of_movement():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    for k in range(1, 25):
        hop.update(k, np.zeros(2), 0.0, world)
    hop.update(25, np.array([0.15, 0.0]), 0.0, world)         # exactly stall_distance: counts as moved
    for k in range(26, 55):
        hop.update(k, np.array([0.15, 0.0]), 0.0, world)      # 29 cycles since the move
    assert hop.hops == 1
    hop.update(55, np.array([0.15, 0.0]), 0.0, world)
    assert hop.hops == 2 and hop.log[-1][4] == 'no_movement'


def test_extras_off_has_no_watchdog_and_no_avoid_list():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P_DESIGN, 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    for k in range(1, 149):
        hop.update(k, np.zeros(2), 0.0, world)
    assert hop.hops == 1
    hop.mark_entry(np.zeros(2))
    assert hop._avoid(np.zeros(2)) != []                      # remembered, but not used:
    g1 = v2m.select_opening(np.zeros(2), 0.0, world, P_DESIGN, None, [(0.0, 1.5, 0.6)])
    g2 = v2m.select_opening(np.zeros(2), 0.0, world, P_DESIGN, None, ())
    assert g1.as_tuple() == g2.as_tuple()


def test_the_stall_pose_is_avoided_and_forgotten_on_release():
    # an open world: the goal bearing is 0, the stall pose is 1.5 m ahead on the +x ray
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    hop.mark_entry(np.array([1.5, 0.0]))
    hop.update(0, np.zeros(2), 0.0, world)
    b = hop.log[0][1]
    assert abs(float(v2m.wrap(b))) >= np.radians(30), b       # not the ray through the stall pose
    hop.release()
    hop.update(1, np.zeros(2), 0.0, world)
    assert abs(float(v2m.wrap(hop.log[1][1]))) <= STEP / 2 + 1e-9


def test_hop_origins_are_remembered_up_to_avoid_hops_oldest_dropped():
    world = se.World([], goal=(50.0, 0.0))
    hop = v2m.WaypointHopper(v2m.v2_params(reselect_radius=1e9), 0.1)
    for k in range(6):
        hop.update(k, np.array([float(k), 0.0]), 0.0, world)
    assert hop.origins == [(3.0, 0.0), (4.0, 0.0), (5.0, 0.0)]
    assert hop.hops == 6


def test_failed_waypoints_are_remembered_up_to_six():
    world = se.World([], goal=(50.0, 0.0))
    hop = v2m.WaypointHopper(v2m.v2_params(reselect_timeout_s=0.1), 0.1)   # timeout every call
    hop.update(0, np.zeros(2), 0.0, world)
    for k in range(1, 12):
        hop.update(k, np.array([0.0, 0.0]), 0.0, world)
    assert len(hop.failed) == 6


def test_a_waypoint_that_was_reached_is_not_a_failed_one():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    wp = hop.update(0, np.zeros(2), 0.0, world).copy()
    hop.update(1, wp, 0.0, world)
    assert hop.log[-1][4] == 'reached' and hop.failed == []


# --------------------------------------------------------------------------- #
# All rollouts collide -> zero command
# --------------------------------------------------------------------------- #
def test_all_rollouts_collide_returns_zero_and_resets_the_nominal_sequence():
    world = se.World([se.Obstacle(0.0, 0.0, 1.0)], goal=(3.0, 0.0))   # robot inside
    m = se.MPPI(seed=0)
    m.u[:] = 0.2
    u = m.step(np.array([0.0, 0.0, 0.0]), world, zero_on_all_collide=True)
    assert m.last_all_collide
    assert np.all(u == 0.0) and np.all(m.u == 0.0)


def test_flag_off_keeps_the_v1_behaviour_when_all_rollouts_collide():
    world = se.World([se.Obstacle(0.0, 0.0, 1.0)], goal=(3.0, 0.0))
    m = se.MPPI(seed=0)
    u = m.step(np.array([0.0, 0.0, 0.0]), world)
    assert not m.last_all_collide
    assert np.any(u != 0.0)


class _Crafted(se.MPPI):
    """MPPI whose sampled rollouts are fixed: K trajectories given by the caller."""

    def __init__(self, pts, **kw):
        super().__init__(batch=pts.shape[0], horizon=pts.shape[1], **kw)
        self._pts = pts

    def rollout(self, state, noise):
        return self._pts.copy(), np.zeros((self.K, self.T, 2))


def _crafted_pts(K=8, T=5, collide_last_only=True):
    pts = np.full((K, T, 2), 9.0)                             # far from the obstacle
    pts[:, -1, :] = 0.0                                       # last point on the obstacle
    return pts


def test_collision_at_any_time_counts_for_every_rollout():
    # every rollout collides, but only at its last time step
    world = se.World([se.Obstacle(0.0, 0.0, 0.3)], goal=(3.0, 0.0))
    m = _Crafted(_crafted_pts())
    u = m.step(np.array([5.0, 5.0, 0.0]), world, zero_on_all_collide=True)
    assert m.last_all_collide and np.all(u == 0.0)


def test_one_surviving_rollout_prevents_the_zero_command():
    world = se.World([se.Obstacle(0.0, 0.0, 0.3)], goal=(3.0, 0.0))
    pts = _crafted_pts()
    pts[3, -1, :] = 9.0                                       # rollout 3 never collides
    m = _Crafted(pts)
    m.step(np.array([5.0, 5.0, 0.0]), world, zero_on_all_collide=True)
    assert not m.last_all_collide


def test_one_free_rollout_is_not_all_colliding():
    world = se.World([], goal=(3.0, 0.0))
    m = se.MPPI(seed=0)
    m.step(np.array([0.0, 0.0, 0.0]), world, zero_on_all_collide=True)
    assert not m.last_all_collide


# --------------------------------------------------------------------------- #
# Flags off == paper-v1 loop; unknown parameter names fail loudly
# --------------------------------------------------------------------------- #
def test_unknown_v2_parameter_is_an_error():
    with pytest.raises(KeyError):
        v2m.v2_params(not_a_parameter=1)


def test_v1_configs_carry_no_v2_key():
    assert all('v2' not in cfg for cfg in CONFIGS.values())


@pytest.mark.parametrize('family,seed,name', [('utrap', 4, 'C_escape'),
                                              ('narrowdyn', 4, 'F_full')])
def test_every_flag_off_reproduces_the_v1_rollout(family, seed, name):
    off = make_v2_configs(progress='goal_distance', gap_rule='closest_viable',
                          hop=False, zero_on_all_collide=False, stall_window=15,
                          act_on_previous_cycle=False, escape_extras=False)
    a = sc.make(family, seed)
    b = sc.make(family, seed)
    r1 = rollout(a.world, CONFIGS[name], a.start, max_steps=400, seed=seed)
    r2 = rollout(b.world, off[name], b.start, max_steps=400, seed=seed)
    assert np.array_equal(r1['_traj'], r2['_traj'])
    for k in ('outcome', 'steps', 'path_length', 'min_clearance', 'entrapped_frac'):
        assert r1[k] == r2[k], k
    assert r2['v2_zero_cmd_steps'] == 0 and r2['v2_progress_events'] == 0


def test_v2_flags_change_the_u_trap_run():
    # sanity: the switches are live (v2 defaults differ from v1 on a U-trap)
    cfgs = make_v2_configs()
    a = sc.make('utrap', 4)
    b = sc.make('utrap', 4)
    r1 = rollout(a.world, CONFIGS['C_escape'], a.start, max_steps=400, seed=4)
    r2 = rollout(b.world, cfgs['C_escape'], b.start, max_steps=400, seed=4)
    assert r2['v2_hops'] >= 1 and r2['v2_progress_events'] > 0
    assert not np.array_equal(r1['_traj'], r2['_traj'])
