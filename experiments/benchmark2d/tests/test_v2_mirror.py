# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for the SE-MPPI v2 escape logic in the 2D mirror (opt-in flags).

Covers: anchor-path progress (the Python analogue of the root-cause scratch
test), horizon-ray opening selection, frozen-waypoint hopping, the "all
rollouts collide -> zero command" mode, and that every flag off reproduces the
paper-v1 loop exactly.
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
STEP = 2 * np.pi / P['gap_num_rays']


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
    tr = v2m.AnchorProgress((0.0, 0.0), (2.0, 0.0), 0.10, 0.5)
    tr.update((1.0, 0.0))
    n = tr.events
    tr.update((1.10, 0.0))                                   # advance == 0.10: no event
    assert tr.events == n
    tr.update((1.1001, 0.0))
    assert tr.events == n + 1


def test_creeping_with_replans_never_stalls():
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    det = se.EntrapmentDetector(stall_window=30)
    _drive(tr, det, [0.025 * k for k in range(1, 230)], replan_every=10,
           goal=(6.0, 0.0))
    assert not det.entrapped


def test_following_replans_adopts_the_candidate():
    # 0.3 m per second along successive 1 Hz replans: no single replan reaches
    # 0.5 m, the lineage does (mirror reading of "adopts within seconds").
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    x = 0.0
    for sec in range(4):
        tr.replan((x, 0.0), (6.0, 0.0))
        for _ in range(10):
            x += 0.03
            tr.update((x, 0.0))
    assert tr.adoptions >= 1


def test_rocking_in_place_never_adopts():
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    tr.update((1.0, 0.0))
    for k in range(40):                                      # +-0.3 m every second
        x = 0.7 + (0.3 if k % 2 else 0.0)
        tr.replan((x, 0.0), (6.0, 0.0))
        for j in range(10):
            tr.update((x + (0.03 * j if k % 2 == 0 else -0.03 * j), 0.0))
    assert tr.adoptions == 0


def test_adoption_and_carry_thresholds_are_inclusive():
    # dyadic numbers: every arc length below is exact in floating point
    tr = v2m.AnchorProgress((0.0, 0.0), (2.0, 0.0), 0.125, 0.5)
    tr.replan((0.0, 0.0), (2.0, 0.0))
    tr.update((0.125, 0.0))                                   # advance == min_advance
    tr.replan((0.125, 0.0), (2.125, 0.0))                     # carried: 0.125 >= 0.125
    tr.update((0.5, 0.0))                                     # 0.125 + 0.375 == adopt
    assert tr.adoptions == 1
    # just under the carry gate: nothing is carried, 0.375 alone does not adopt
    tr = v2m.AnchorProgress((0.0, 0.0), (2.0, 0.0), 0.125, 0.5)
    tr.replan((0.0, 0.0), (2.0, 0.0))
    tr.update((0.0625, 0.0))
    tr.replan((0.0625, 0.0), (2.0625, 0.0))
    tr.update((0.4375, 0.0))                                  # 0.375 + 0 carried
    assert tr.adoptions == 0


def test_backing_off_and_returning_earns_no_advance():
    tr = v2m.AnchorProgress((0.0, 0.0), (6.0, 0.0), 0.10, 0.5)
    tr.update((1.0, 0.0))
    tr.replan((1.0, 0.0), (6.0, 0.0))
    for x in (1.45, 1.0):                                     # forward 0.45, back to 0
        tr.update((x, 0.0))
    tr.replan((1.0, 0.0), (6.0, 0.0))
    tr.update((1.06, 0.0))
    assert tr.adoptions == 0


def test_polyline_nearest_arc_and_degenerate_path():
    path = v2m.PolylinePath([(0, 0), (1, 0), (1, 1)])
    assert path.length == pytest.approx(2.0)
    assert path.nearest_arc((0.5, 0.3)) == pytest.approx(0.5)
    assert path.nearest_arc((1.2, 0.5)) == pytest.approx(1.5)
    assert path.nearest_arc((-3, 0)) == pytest.approx(0.0)    # clamped before start
    assert path.nearest_arc((9, 9)) == pytest.approx(2.0)     # clamped after end
    deg = v2m.PolylinePath([(1, 1), (1, 1)])                  # robot on the goal
    assert deg.length == 0.0 and deg.nearest_arc((5, 5)) == 0.0


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
    b, free, reached = v2m.select_opening(pos, gb, world, P)
    assert reached
    assert _angdiff(b, np.pi) <= np.radians(20)


def test_open_world_takes_the_ray_closest_to_the_goal():
    world = se.World([], goal=(3.0, 0.0))
    b, free, reached = v2m.select_opening((0, 0), 0.3, world, P)
    assert reached and _angdiff(b, 0.3) <= STEP / 2 + 1e-9


def test_hysteresis_equal_to_the_improvement_keeps_the_previous_ray():
    world = se.World([], goal=(3.0, 0.0))
    kp = 21                                                   # 3 steps off the goal ray
    prev = -np.pi + 2 * np.pi * kp / P['gap_num_rays']
    gain = abs(float(v2m.wrap(prev - 0.0)))                   # a_prev - a_best exactly
    p = v2m.v2_params(hysteresis=gain)
    b, _, _ = v2m.select_opening((0, 0), 0.0, world, p, prev_bearing=prev)
    assert b == pytest.approx(v2m.wrap(prev))                 # equal: keep
    p = v2m.v2_params(hysteresis=np.nextafter(gain, -np.inf))
    b, _, _ = v2m.select_opening((0, 0), 0.0, world, p, prev_bearing=prev)
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
    b, _, _ = v2m.select_opening((0, 0), 0.0, world, P, prev_bearing=prev_keep)
    assert b == pytest.approx(v2m.wrap(prev_keep))
    # previous ray 3 steps off (0.524 rad > 0.5 better): switched to the goal ray
    b, _, _ = v2m.select_opening((0, 0), 0.0, world, P, prev_bearing=3 * STEP)
    assert b == pytest.approx(0.0)
    # sign and wrap-around: previous ray across the +-pi seam
    b, _, _ = v2m.select_opening((0, 0), np.pi, world, P, prev_bearing=-np.pi + STEP)
    assert abs(float(v2m.wrap(b - np.pi))) <= STEP + 1e-9


def test_previous_ray_that_no_longer_reaches_the_horizon_is_dropped():
    world = _pocket()
    pos = np.array([0.35, 0.1])
    gb = 0.0
    # bearing 0 hits the back wall: even a huge hysteresis cannot keep it
    p = v2m.v2_params(hysteresis=10.0)
    b, _, reached = v2m.select_opening(pos, gb, world, p, prev_bearing=0.0)
    assert reached and _angdiff(b, np.pi) <= np.radians(20)


def test_no_ray_reaches_the_horizon_takes_the_longest():
    # a closed ring of obstacles with one side pushed out: nothing reaches 2 m
    ring = [se.Obstacle(1.2 * np.cos(a), 1.2 * np.sin(a), 0.3)
            for a in np.linspace(0, 2 * np.pi, 40, endpoint=False)]
    ring = [o for o in ring if not (abs(o.p[1]) < 0.4 and o.p[0] > 0)]
    ring += [se.Obstacle(1.9, 0.0, 0.3)]                      # far plug on +x
    world = se.World(ring, goal=(3.0, 0.0))
    b, free, reached = v2m.select_opening((0, 0), np.pi, world, P)
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
    for step in range(1, 50):                                # the robot barely moves
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
    hop = v2m.WaypointHopper(P, 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    hop.update(149, np.zeros(2), 0.0, world)                 # 14.9 s
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


def test_reselect_radius_boundary_is_exclusive():
    world = se.World([], goal=(5.0, 0.0))
    probe = v2m.WaypointHopper(P, 0.1)
    wp = probe.update(0, np.zeros(2), 0.0, world).copy()
    d = float(np.hypot(*(wp - np.zeros(2))))
    hop = v2m.WaypointHopper(v2m.v2_params(reselect_radius=d), 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    hop.update(1, np.zeros(2), 0.0, world)                    # distance == radius
    assert hop.hops == 1
    hop = v2m.WaypointHopper(v2m.v2_params(reselect_radius=np.nextafter(d, np.inf)), 0.1)
    hop.update(0, np.zeros(2), 0.0, world)
    hop.update(1, np.zeros(2), 0.0, world)                    # distance < radius
    assert hop.hops == 2


def test_waypoint_distance_is_capped():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(v2m.v2_params(gap_max_range=6.0, window_half=6.0), 0.1)
    wp = hop.update(0, np.zeros(2), 0.0, world)
    assert np.hypot(*wp) == pytest.approx(2.5)                # min(6 - 0.27, 2.5)


def test_waypoint_distance_floor_is_zero():
    # a ray blocked at 0.1 m: free - 0.22 - 0.05 < 0 is clipped to 0, not negative
    ring = [se.Obstacle(0.45 * np.cos(a), 0.45 * np.sin(a), 0.2)
            for a in np.linspace(0, 2 * np.pi, 30, endpoint=False)]
    world = se.World(ring, goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(P, 0.1)
    wp = hop.update(0, np.zeros(2), 0.0, world)
    assert np.hypot(*wp) == pytest.approx(0.0)


def test_hop_off_reselects_every_step():
    world = se.World([], goal=(5.0, 0.0))
    hop = v2m.WaypointHopper(v2m.v2_params(hop=False), 0.1)
    for s in range(5):
        hop.update(s, np.zeros(2), 0.0, world)
    assert hop.hops == 5


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
                          hop=False, zero_on_all_collide=False, stall_window=15)
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
