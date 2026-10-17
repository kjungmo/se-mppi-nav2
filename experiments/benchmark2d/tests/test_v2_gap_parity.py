# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Parity of the mirror's opening selection with the C++ horizon rule.

The same synthetic pocket is given to (a) the mirror's ``select_opening`` on its
analytic circle world and (b) a Python re-implementation of the C++ rule on a
rasterised 60 x 60 costmap (0.05 m, rolling window centred on the robot). The
two must return the same bearing within one ray step (2*pi/36).

RE-POINT TO C++ LATER
---------------------
The C++ side (branch ``semppi-v2/escape-fix``, ``gap_search.cpp``) was not
available when this test was written. ``cxx_ray_free`` transcribes
``rayFreeDistance`` of the current ``gap_search.cpp`` (half-resolution step,
stop at cost >= 253, off-map returns the sample distance), and
``cxx_select_horizon`` encodes DESIGN_ADVICE section 2 defect 4a:

  * a ray "reaches the horizon" when it is not stopped by a cell (it runs to
    ``max_range`` or off the map edge);
  * among those rays take the one closest to the goal bearing (first of equals,
    as the strict ``<`` of ``findEscapeGap``); keep the previous bearing unless
    the new one is better by more than 0.5 rad;
  * if no ray reaches the horizon take the longest.

When the C++ rule lands, replace the two functions by a call into it (a small
pybind/ctypes shim or a gtest that prints bearings for the cases listed in
``_cases``) and delete this block; the test body and ``_cases`` stay.
"""

import math
import os
import sys

import numpy as np
import pytest

from experiments.benchmark2d import scenarios as sc

_PROTO = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'prototype')
if _PROTO not in sys.path:
    sys.path.insert(0, _PROTO)
import se_mppi_v2 as v2m  # noqa: E402

P = v2m.v2_params()
N_RAYS = P['gap_num_rays']
RAY_STEP = 2 * math.pi / N_RAYS
RES = 0.05
SIZE = 60                      # 3 m x 3 m rolling window
HALF = SIZE * RES / 2.0
COST_THRESHOLD = 253


# --------------------------------------------------------------------------- #
# Python stand-in for the C++ (to be re-pointed)
# --------------------------------------------------------------------------- #
def rasterise(world, centre):
    """Cost grid of the rolling window: 253 where the robot centre cannot be."""
    cx, cy = centre
    ox, oy = cx - HALF, cy - HALF
    ii = (np.arange(SIZE) + 0.5) * RES
    gx = ox + ii[None, :].repeat(SIZE, 0)
    gy = oy + ii[:, None].repeat(SIZE, 1)
    grid = np.zeros((SIZE, SIZE), np.uint8)
    for o in world.obstacles:
        d = np.hypot(gx - o.p[0], gy - o.p[1]) - o.r
        grid[d < world.robot_radius] = COST_THRESHOLD
    return grid, (ox, oy)


def cxx_ray_free(grid, origin, rx, ry, bearing, max_range):
    """``rayFreeDistance``: returns (distance, reached_horizon)."""
    step = max(0.5 * RES, 1e-3)
    c, s = math.cos(bearing), math.sin(bearing)
    r = step
    while r <= max_range:
        wx, wy = rx + r * c, ry + r * s
        if wx < origin[0] or wy < origin[1]:
            return r, True
        mx = int((wx - origin[0]) / RES)
        my = int((wy - origin[1]) / RES)
        if mx >= SIZE or my >= SIZE:
            return r, True                          # ran off the map: free to the edge
        if grid[my, mx] >= COST_THRESHOLD:
            return r, False
        r += step
    return max_range, True


def cxx_select_horizon(grid, origin, rx, ry, goal_bearing, num_rays, max_range,
                       prev_bearing=None, hysteresis=0.5):
    lens, reach, brgs = [], [], []
    for k in range(num_rays):
        b = -math.pi + 2 * math.pi * k / num_rays
        d, ok = cxx_ray_free(grid, origin, rx, ry, b, max_range)
        brgs.append(b)
        lens.append(d)
        reach.append(ok)
    align = [abs(math.atan2(math.sin(b - goal_bearing), math.cos(b - goal_bearing)))
             for b in brgs]
    kp = None
    if prev_bearing is not None:
        kp = int(round((v2m.wrap(prev_bearing) + math.pi) / (2 * math.pi) * num_rays)) \
            % num_rays
    if any(reach):
        best, best_a = None, math.inf
        for k in range(num_rays):
            if reach[k] and align[k] < best_a:
                best, best_a = k, align[k]
        if kp is not None and reach[kp] and align[kp] - best_a <= hysteresis:
            best = kp
    else:
        best = max(range(num_rays), key=lambda k: (lens[k], -k))
        if kp is not None and lens[kp] >= lens[best] - 1e-12:
            best = kp
    return brgs[best], lens[best], reach[best]


# --------------------------------------------------------------------------- #
# Cases: synthetic pockets (the benchmark's U-trap family), robot poses along
# the straight start->goal line and beside it, three goal bearings each.
# --------------------------------------------------------------------------- #
def _cases(seeds=range(8)):
    for seed in seeds:
        scn = sc.make('utrap', seed)
        world = scn.world
        start = np.array(scn.start[:2], float)
        goal = np.array(scn.goal, float)
        d = (goal - start) / np.hypot(*(goal - start))
        perp = np.array([-d[1], d[0]])
        xb = scn.meta['xb']
        for t in np.linspace(0.0, xb - 0.55, 9):
            for off in (-0.3, 0.0, 0.3):
                pos = start + t * d + off * perp
                if world.min_clearance(pos) < world.robot_radius + 0.02:
                    continue                              # inside the band
                gb = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
                for dg in (0.0, 0.8, -0.8):
                    yield seed, world, pos, gb + dg


def _angdiff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


# A 0.05 m cost cell cannot resolve a ray that grazes the bumpy circle-chain
# wall by a few millimetres (seed 1, pos (0.277, 0.034): the +90 deg ray passes a
# cusp with 7 mm to spare; the raster marks it blocked). Such a case is not a
# disagreement between the two rules: it is below the grid's resolution, and it
# can swap the choice between two rays on opposite sides when their alignments
# nearly tie (seed 5, pos (0.578, 0.513): -110 deg against -20 deg, 0.06 deg
# apart). A case is "well conditioned" when (1) the mirror's own selection stays
# within one ray step if the robot radius changes by half a cell diagonal, and
# (2) the best horizon ray beats the best ray more than one step away from it by
# at least MIN_MARGIN rad of alignment.
RADIUS_PERTURB = 0.5 * math.hypot(RES, RES)
MIN_MARGIN = 0.05


def _well_conditioned(world, pos, gb):
    r0 = world.robot_radius
    base = None
    try:
        for dr in (0.0, -RADIUS_PERTURB, RADIUS_PERTURB):
            world.robot_radius = r0 + dr
            b, _, _ = v2m.select_opening(pos, gb, world, P)
            if base is None:
                base = b
                brg, _, reached = v2m.scan_rays(pos, world, N_RAYS,
                                                P['gap_max_range'], P['window_half'])
            elif _angdiff(b, base) > RAY_STEP + 1e-9:
                return False
    finally:
        world.robot_radius = r0
    if reached.any():
        far = [abs(float(v2m.wrap(brg[k] - gb))) for k in range(N_RAYS)
               if reached[k] and _angdiff(brg[k], base) > 1.5 * RAY_STEP]
        near = abs(float(v2m.wrap(base - gb)))
        if far and min(far) - near < MIN_MARGIN:
            return False
    return True


def test_mirror_and_cxx_rule_agree_within_one_ray_step():
    n = worst = skipped = 0
    bad = []
    for seed, world, pos, gb in _cases():
        if not _well_conditioned(world, pos, gb):
            skipped += 1
            continue
        grid, origin = rasterise(world, pos)
        bc, _, _ = cxx_select_horizon(grid, origin, pos[0], pos[1], gb, N_RAYS,
                                      P['gap_max_range'])
        bm, _, _ = v2m.select_opening(pos, gb, world, P)
        diff = _angdiff(bm, bc)
        worst = max(worst, diff)
        n += 1
        if diff > RAY_STEP + 1e-9:
            bad.append((seed, pos.round(3).tolist(), round(gb, 3),
                        round(bm, 3), round(bc, 3)))
    print(f'\nparity: {int(n)} well-conditioned cases ({skipped} below the grid '
          f'resolution, skipped), worst difference {worst:.4f} rad '
          f'({worst / RAY_STEP:.2f} ray steps), mismatches {len(bad)}')
    assert n >= 300 and skipped <= 0.15 * (n + skipped), (n, skipped)
    assert not bad, bad[:5]


def test_parity_with_hysteresis_carried_between_positions():
    """Walk both selectors along the start->goal line carrying the bearing."""
    for seed in range(8):
        scn = sc.make('utrap', seed)
        world = scn.world
        start = np.array(scn.start[:2], float)
        goal = np.array(scn.goal, float)
        d = (goal - start) / np.hypot(*(goal - start))
        pm = pc = None
        for t in np.linspace(0.0, scn.meta['xb'] - 0.55, 25):
            pos = start + t * d
            if world.min_clearance(pos) < world.robot_radius + 0.02:
                continue
            gb = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
            if not _well_conditioned(world, pos, gb):
                continue
            grid, origin = rasterise(world, pos)
            bc, _, _ = cxx_select_horizon(grid, origin, pos[0], pos[1], gb, N_RAYS,
                                          P['gap_max_range'], prev_bearing=pc)
            bm, _, _ = v2m.select_opening(pos, gb, world, P, prev_bearing=pm)
            # each keeps its own history; they may only differ by the
            # discretisation, so the bound is one step per selector history
            assert _angdiff(bm, bc) <= RAY_STEP + 1e-9, (seed, t, bm, bc)
            pm, pc = bm, bc


def test_stalled_pocket_pose_selects_the_mouth_in_both():
    """The U-trap stall pose (0.3 m short of the back wall) leaves via the mouth."""
    hits = 0
    for seed in range(8):
        scn = sc.make('utrap', seed)
        world = scn.world
        start = np.array(scn.start[:2], float)
        goal = np.array(scn.goal, float)
        d = (goal - start) / np.hypot(*(goal - start))
        pos = start + (scn.meta['xb'] - 0.33 - 0.22 - 0.3) * d
        if world.min_clearance(pos) < world.robot_radius + 0.02:
            continue
        gb = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
        if not _well_conditioned(world, pos, gb):
            continue
        grid, origin = rasterise(world, pos)
        bc, _, rc = cxx_select_horizon(grid, origin, pos[0], pos[1], gb, N_RAYS,
                                       P['gap_max_range'])
        bm, _, rm = v2m.select_opening(pos, gb, world, P)
        mouth = math.atan2(-d[1], -d[0])                  # back along the path
        assert rc == rm
        assert _angdiff(bm, bc) <= RAY_STEP + 1e-9
        if rm:                                            # a ray reaches the horizon
            assert _angdiff(bm, mouth) <= math.radians(60), (seed, bm, mouth)
            hits += 1
    assert hits >= 3


def test_cxx_stand_in_matches_the_documented_edge_semantics():
    # open grid: every ray reaches the horizon; on-axis the map edge (1.5 m) ends it
    grid = np.zeros((SIZE, SIZE), np.uint8)
    origin = (-HALF, -HALF)
    d, ok = cxx_ray_free(grid, origin, 0.0, 0.0, 0.0, 2.0)
    assert ok and d == pytest.approx(HALF, abs=0.03)
    d, ok = cxx_ray_free(grid, origin, 0.0, 0.0, math.pi / 4, 2.0)
    assert ok and d == pytest.approx(2.0)
    grid[30:, 40] = COST_THRESHOLD                        # a wall 0.5 m east of centre
    d, ok = cxx_ray_free(grid, origin, 0.0, 0.0, 0.0, 2.0)
    assert not ok and d == pytest.approx(0.5, abs=0.03)
