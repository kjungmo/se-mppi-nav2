# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Parity of the mirror's escape logic with the C++ the plugin runs.

The mirror (``experiments/prototype/se_mppi_v2.py``) is a transcription of
``opening_search.cpp``, ``path_progress.cpp`` and ``entrapment_detector.hpp``. This file calls the
C++ itself, through ``parity_shim`` (``src/nav2_se_controller/test/deployment/parity_shim.cpp``),
and compares:

* ``selectHorizonGap`` with ``select_horizon_gap`` on the same rays, with avoid lists, a minimum
  clearance, a previous bearing and hysteresis (exact equality of every field);
* ``castGapRays`` on a rasterised costmap with ``scan_rays`` on the analytic world (the same
  horizon flags, clearances within one raster cell, on rays that do not graze a wall);
* the whole chain on the benchmark's U-trap family: C++ rays then C++ selection against the
  mirror's rays and selection (within one ray step on well-conditioned cases);
* ``PathProgressTracker`` + ``EntrapmentDetector`` with ``PathProgressTracker`` + the mirror
  detector on random trajectories with replans (events, adoptions, entrapment after every update;
  exact on dyadic coordinates, where the arc lengths are exact in both).

The shim is built by ``colcon build`` (BUILD_TESTING). Set ``SE_PARITY_SHIM`` to its path
(``<build>/nav2_se_controller/parity_shim``). Without it these tests are skipped, unless
``SE_REQUIRE_PARITY=1``: then a missing shim is a failure (use that in CI and for any claim that
the mirror matches the C++).

Also here: the mirror's default parameters equal the compiled defaults of the plugin and the
values of ``nav2_se_campaign2.yaml``.
"""

import math
import os
import random
import re
import subprocess
import sys

import numpy as np
import pytest

from experiments.benchmark2d import scenarios as sc

_PROTO = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'prototype')
if _PROTO not in sys.path:
    sys.path.insert(0, _PROTO)
import se_mppi_proto as se  # noqa: E402
import se_mppi_v2 as v2m  # noqa: E402

REPO = os.path.normpath(os.path.join(_PROTO, '..', '..'))
P = v2m.v2_params()
N_RAYS = P['gap_num_rays']
RAY_STEP = 2 * math.pi / N_RAYS
RES = 0.05
SIZE = 60                      # 3 m x 3 m rolling window
HALF = SIZE * RES / 2.0
COST_THRESHOLD = 253


# --------------------------------------------------------------------------- #
# The shim
# --------------------------------------------------------------------------- #
class Shim:
    def __init__(self, path):
        self.proc = subprocess.Popen([path], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        assert self.proc.stdout.readline().strip() == 'READY'

    def ask(self, line, rows=1):
        self.proc.stdin.write(line + '\n')
        self.proc.stdin.flush()
        return [self.proc.stdout.readline().split() for _ in range(rows)]

    def send_raw(self, text):
        self.proc.stdin.write(text)
        self.proc.stdin.flush()

    def close(self):
        try:
            self.proc.stdin.write('QUIT\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:                      # noqa: BLE001
            self.proc.kill()


def _shim_path():
    path = os.environ.get('SE_PARITY_SHIM')
    if path and os.access(path, os.X_OK):
        return path
    return None


@pytest.fixture(scope='module')
def shim():
    path = _shim_path()
    if path is None:
        if os.environ.get('SE_REQUIRE_PARITY') == '1':
            pytest.fail('SE_REQUIRE_PARITY=1 but SE_PARITY_SHIM is not an executable '
                        '(build nav2_se_controller and point it to build/nav2_se_controller/parity_shim)')
        pytest.skip('SE_PARITY_SHIM is not set: the C++ parity tests need the built parity_shim')
    s = Shim(path)
    yield s
    s.close()


def f17(x):
    return repr(float(x))


def cxx_select(shim, bearings, free, reached, goal_bearing, min_clearance, prev, hysteresis, avoid):
    parts = ['SELECT', f17(min_clearance), f17(hysteresis), f17(goal_bearing),
             'nan' if prev is None else f17(prev), str(len(avoid))]
    for b, d, r in avoid:
        parts += [f17(b), f17(d), f17(r)]
    parts.append(str(len(bearings)))
    for b, c, h in zip(bearings, free, reached):
        parts += [f17(b), f17(c), '1' if h else '0']
    (row,) = shim.ask(' '.join(parts))
    return (row[0] == '1', float(row[1]), float(row[2]), row[3] == '1', row[4] == '1', int(row[5]))


# --------------------------------------------------------------------------- #
# 1. The selection rule on the same rays
# --------------------------------------------------------------------------- #
def _random_case(rng):
    n = rng.choice([36, 36, 36, 12, 72])
    bearings = [-math.pi + 2 * math.pi * k / n for k in range(n)]
    mode = rng.random()
    free, reached = [], []
    for _ in range(n):
        h = rng.random() < (0.35 if mode < 0.7 else 0.0)        # 30 percent of cases: no horizon ray
        free.append(round(rng.uniform(1.4, 2.0), 3) if h else round(rng.uniform(0.05, 1.9), 3))
        reached.append(h)
    if mode > 0.9:                                              # ties in the longest-ray branch
        free = [round(f * 4) / 4 for f in free]
    goal = rng.uniform(-math.pi, math.pi)
    prev = rng.choice([None, rng.uniform(-math.pi, math.pi), bearings[rng.randrange(n)]])
    hyst = rng.choice([0.5, 0.5, 0.0, rng.uniform(0.0, 2.0)])
    min_clear = rng.choice([0.0, 0.6, 0.6, rng.uniform(0.0, 1.5)])
    avoid = []
    for _ in range(rng.choice([0, 0, 1, 2, 3, 4])):
        radius = rng.choice([0.6, 1.0, rng.uniform(0.2, 1.2)])
        dist = rng.choice([rng.uniform(0.0, 3.0), radius, radius * 1.0000001, rng.uniform(0.3, 2.0)])
        avoid.append((rng.uniform(-math.pi, math.pi), dist, radius))
    return bearings, free, reached, goal, min_clear, prev, hyst, avoid


def test_selection_rule_matches_the_cxx_on_random_rays(shim):
    rng = random.Random(20261018)
    n = found = kept = longest = avoided_changed = 0
    for _ in range(4000):
        bearings, free, reached, goal, mc, prev, hyst, avoid = _random_case(rng)
        cxx = cxx_select(shim, bearings, free, reached, goal, mc, prev, hyst, avoid)
        py = v2m.select_horizon_gap(np.array(bearings), np.array(free), np.array(reached), goal,
                                    mc, prev, hyst, avoid).as_tuple()
        assert py[0] == cxx[0], (py, cxx)
        if cxx[0]:
            assert py[1] == cxx[1] and py[2] == cxx[2], (py, cxx)
            assert py[3:] == cxx[3:], (py, cxx)
        n += 1
        found += cxx[0]
        kept += cxx[0] and cxx[4]
        longest += cxx[0] and not cxx[3]
        if avoid:
            plain = cxx_select(shim, bearings, free, reached, goal, mc, prev, hyst, [])
            avoided_changed += plain != cxx
    # the random cases exercise every branch
    assert n == 4000 and found > 3000 and kept > 100 and longest > 300 and avoided_changed > 200, (
        found, kept, longest, avoided_changed)


def test_selection_rule_boundaries_match_the_cxx(shim):
    n = 36
    b = [-math.pi + 2 * math.pi * k / n for k in range(n)]
    free = [2.0] * n
    reach = [True] * n
    # place of avoidance at distance == radius is ignored; just beyond it blocks the near ray
    k_near = 20
    for dist, expect_blocked in ((0.6, False), (0.6000001, True)):
        av = [(b[k_near], dist, 0.6)]
        cxx = cxx_select(shim, b, free, reach, b[k_near], 0.0, None, 0.5, av)
        py = v2m.select_horizon_gap(np.array(b), np.array(free), np.array(reach), b[k_near],
                                    0.0, None, 0.5, av).as_tuple()
        assert py == cxx
        assert (cxx[1] != b[k_near]) is expect_blocked, (dist, cxx)
    # ray clearance exactly min_clearance is a candidate; just under is not
    for c, ok in ((0.6, True), (0.5999999, False)):
        free2 = [0.1] * n
        free2[7] = c
        reach2 = [False] * n
        reach2[7] = True
        cxx = cxx_select(shim, b, free2, reach2, b[7], 0.6, None, 0.5, [])
        py = v2m.select_horizon_gap(np.array(b), np.array(free2), np.array(reach2), b[7], 0.6,
                                    None, 0.5, []).as_tuple()
        assert py == cxx and (cxx[3] is True) is ok, (c, cxx)


# --------------------------------------------------------------------------- #
# 2. Ray casting: analytic world against the C++ on a rasterised costmap
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


def cxx_cast(shim, grid, origin, rx, ry, num_rays, max_range):
    head = (f'CAST {RES} {SIZE} {f17(origin[0])} {f17(origin[1])} {f17(rx)} {f17(ry)} '
            f'{num_rays} {max_range} {COST_THRESHOLD}\n')
    body = '\n'.join(' '.join(str(int(v)) for v in row) for row in grid) + '\n'
    shim.send_raw(head + body)
    rows = [shim.proc.stdout.readline().split() for _ in range(num_rays)]
    return (np.array([float(r[0]) for r in rows]), np.array([float(r[1]) for r in rows]),
            np.array([r[2] == '1' for r in rows]))


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


# A 0.05 m cost cell cannot resolve a ray that grazes the bumpy circle-chain wall by a few
# millimetres. Such a ray is not a disagreement between the two casts: it is below the grid's
# resolution. A ray is "robust" when its horizon flag and length do not change if the robot
# radius changes by half a cell diagonal.
RADIUS_PERTURB = 0.5 * math.hypot(RES, RES)


def _robust_rays(world, pos):
    r0 = world.robot_radius
    base_free = base_reached = None
    ok = np.ones(N_RAYS, bool)
    try:
        for dr in (0.0, -RADIUS_PERTURB, RADIUS_PERTURB):
            world.robot_radius = r0 + dr
            _, free, reached = v2m.scan_rays(pos, world, N_RAYS, P['gap_max_range'], P['window_half'])
            if base_free is None:
                base_free, base_reached = free, reached
            else:
                ok &= (reached == base_reached) & (np.abs(free - base_free) <= RES + 1e-9)
    finally:
        world.robot_radius = r0
    return ok


def test_ray_cast_matches_the_cxx_on_the_utrap_family(shim):
    rays = skipped = worst = 0
    for seed, world, pos, gb in _cases():
        grid, origin = rasterise(world, pos)
        _, c_free, c_reach = cxx_cast(shim, grid, origin, pos[0], pos[1], N_RAYS, P['gap_max_range'])
        _, m_free, m_reach = v2m.scan_rays(pos, world, N_RAYS, P['gap_max_range'], P['window_half'])
        robust = _robust_rays(world, pos)
        skipped += int((~robust).sum())
        for k in np.flatnonzero(robust):
            assert c_reach[k] == m_reach[k], (seed, pos, k, c_free[k], m_free[k])
            diff = abs(c_free[k] - m_free[k])
            worst = max(worst, diff)
            # the raster marks a whole cell: up to a cell diagonal (0.071 m) plus the sampling
            # of the C++ ray (0.025 m)
            assert diff <= 0.1 + 1e-9, (seed, pos, k, c_free[k], m_free[k])
            rays += 1
    print(f'\nray parity: {rays} robust rays, {skipped} below the grid resolution skipped, '
          f'worst length difference {worst:.3f} m')
    assert rays >= 8000 and skipped <= 0.2 * (rays + skipped)


# --------------------------------------------------------------------------- #
# 3. Cast + selection with avoid lists, C++ against mirror
# --------------------------------------------------------------------------- #
def _well_conditioned(world, pos, gb, avoid, prev):
    """The mirror's own selection stays within one ray step if the robot radius changes by half a
    cell diagonal, and the best horizon ray beats the best ray more than one step away by at
    least MIN_MARGIN rad of alignment. (Cases decided below the grid's resolution are not
    disagreements.)"""
    r0 = world.robot_radius
    base = None
    try:
        for dr in (0.0, -RADIUS_PERTURB, RADIUS_PERTURB):
            world.robot_radius = r0 + dr
            g = v2m.select_opening(pos, gb, world, P, prev, avoid)
            if base is None:
                base = g
                brg, _, reached = v2m.scan_rays(pos, world, N_RAYS, P['gap_max_range'],
                                                P['window_half'])
            elif g.found != base.found or (g.found and _angdiff(g.bearing, base.bearing)
                                           > RAY_STEP + 1e-9):
                return False
    finally:
        world.robot_radius = r0
    if base.found and base.horizon and reached.any():
        far = [abs(float(v2m.wrap(brg[k] - gb))) for k in range(N_RAYS)
               if reached[k] and _angdiff(brg[k], base.bearing) > 1.5 * RAY_STEP]
        near = abs(float(v2m.wrap(base.bearing - gb)))
        if far and min(far) - near < 0.05:
            return False
    return True


def test_cast_and_selection_with_avoid_lists_match_the_cxx(shim):
    rng = random.Random(7)
    n = skipped = with_avoid = 0
    worst = 0.0
    bad = []
    for seed, world, pos, gb in _cases():
        avoid_pts = []
        for _ in range(rng.choice([0, 1, 2, 3])):
            ang = rng.uniform(-math.pi, math.pi)
            dist = rng.uniform(0.3, 2.0)
            avoid_pts.append((pos + dist * np.array([math.cos(ang), math.sin(ang)]),
                              rng.choice([1.0, 0.6])))
        avoid = [(math.atan2(a[1] - pos[1], a[0] - pos[0]), float(np.hypot(*(a - pos))), r)
                 for a, r in avoid_pts]
        prev = rng.choice([None, rng.uniform(-math.pi, math.pi)])
        if not _well_conditioned(world, pos, gb, avoid, prev):
            skipped += 1
            continue
        grid, origin = rasterise(world, pos)
        c_b, c_free, c_reach = cxx_cast(shim, grid, origin, pos[0], pos[1], N_RAYS,
                                        P['gap_max_range'])
        cxx = cxx_select(shim, c_b, c_free, c_reach, gb, P['gap_min_clearance'], prev,
                         P['hysteresis'], avoid)
        py = v2m.select_opening(pos, gb, world, P, prev, avoid)
        assert py.found == cxx[0], (seed, pos.round(3).tolist(), py.as_tuple(), cxx)
        n += 1
        with_avoid += bool(avoid)
        if cxx[0]:
            diff = _angdiff(py.bearing, cxx[1])
            worst = max(worst, diff)
            if diff > RAY_STEP + 1e-9:
                bad.append((seed, pos.round(3).tolist(), round(gb, 3), py.as_tuple(), cxx))
    print(f'\nselection parity: {n} well-conditioned cases ({with_avoid} with an avoid list, '
          f'{skipped} below the grid resolution skipped), worst difference {worst:.4f} rad '
          f'({worst / RAY_STEP:.2f} ray steps), mismatches {len(bad)}')
    assert n >= 300 and skipped <= 0.25 * (n + skipped), (n, skipped)
    assert with_avoid >= 150
    assert not bad, bad[:5]


# --------------------------------------------------------------------------- #
# 4. Progress tracker and detector on random trajectories
# --------------------------------------------------------------------------- #
def _drive_both(shim, rng, steps, dyadic, min_adv, adopt, window):
    """Random walk with replans; return the number of steps compared."""
    shim.ask(f'PROGRESS {f17(min_adv)} {f17(adopt)} {window}')
    mir = v2m.PathProgressTracker(min_adv, adopt)
    det = se.EntrapmentDetector(stall_window=window)
    q = (lambda v: round(v * 16) / 16) if dyadic else (lambda v: v)
    x, y = q(rng.uniform(-1, 1)), q(rng.uniform(-1, 1))
    start = (x, y)
    goal = (x + 8.0, y) if dyadic else (x + rng.uniform(4, 8), y + rng.uniform(-2, 2))
    shim.ask(f'START {f17(start[0])} {f17(start[1])} {f17(goal[0])} {f17(goal[1])}')
    mir.start_task([start, goal])
    mode = 0
    seen_adopt = seen_ent = seen_events = 0
    for i in range(steps):
        if i % 25 == 0:
            mode = rng.choice([0, 1, 2, 3, 4])          # forward, backward, still, sideways, jitter
        step = {0: (0.0625, 0.0), 1: (-0.0625, 0.0), 2: (0.0, 0.0), 3: (0.0, 0.0625),
                4: (rng.choice([-0.0625, 0.0625]), rng.choice([0.0, 0.0625]))}[mode]
        if not dyadic:
            step = (step[0] * rng.uniform(0.2, 1.4), step[1] * rng.uniform(0.2, 1.4))
        x, y = q(x + step[0]), q(y + step[1])
        if i % 10 == 9:                                  # the BT's replan from the robot pose
            if dyadic:
                tx, ty = rng.choice([(x + 4.0, y), (x, y + 4.0), (x - 4.0, y), (x, y - 4.0)])
            else:
                tx, ty = goal if rng.random() < 0.5 else (x + rng.uniform(-3, 3), y + rng.uniform(-3, 3))
            shim.ask(f'REPLAN {f17(x)} {f17(y)} {f17(tx)} {f17(ty)}')
            mir.offer_replan([(x, y), (tx, ty)])
        (row,) = shim.ask(f'UPDATE {f17(x)} {f17(y)}')
        ev = mir.update((x, y))
        ent = det.update(-float(ev))
        assert (int(row[0]), int(row[1]), row[2] == '1') == (ev, mir.adoptions, ent), (
            i, (x, y), row, (ev, mir.adoptions, ent))
        seen_adopt = max(seen_adopt, mir.adoptions)
        seen_ent += ent
        seen_events = ev
    return seen_adopt, seen_ent, seen_events


@pytest.mark.parametrize('dyadic', [True, False])
def test_progress_chain_and_detector_match_the_cxx(shim, dyadic):
    rng = random.Random(11 if dyadic else 12)
    adopted = entrapped = events = 0
    for trial in range(60):
        window = rng.choice([5, 30])
        min_adv, adopt = (0.125, 0.5) if dyadic else (0.10, 0.5)
        a, e, ev = _drive_both(shim, rng, 400, dyadic, min_adv, adopt, window)
        adopted += a
        entrapped += e
        events += ev
    # the random walks reach every branch: events, adoptions and entrapment
    assert adopted > 5 and entrapped > 500 and events > 500, (adopted, entrapped, events)


def test_the_documented_chain_cases_match_the_cxx(shim):
    """The C++ unit cases of path_progress_tracker, replayed through both."""
    shim.ask('PROGRESS 0.125 0.5 30')
    mir = v2m.PathProgressTracker(0.125, 0.5)
    shim.ask('START 0 0 8 0')
    mir.start_task([(0, 0), (8, 0)])

    def both(kind, *a):
        if kind == 'u':
            (row,) = shim.ask(f'UPDATE {a[0]} {a[1]}')
            ev = mir.update((a[0], a[1]))
            assert (int(row[0]), int(row[1])) == (ev, mir.adoptions)
        else:
            shim.ask(f'REPLAN {a[0]} {a[1]} {a[2]} {a[3]}')
            mir.offer_replan([(a[0], a[1]), (a[2], a[3])])
    both('u', 1.0, 0.0)
    both('r', 1.0, 0.0, 9.0, 0.0)
    both('u', 1.0, 0.0)
    both('u', 0.5, 0.0)                    # behind the first pose of the candidate: negative arc
    both('r', 0.5, 0.0, 8.5, 0.0)
    both('u', 0.5, 0.0)
    both('u', 1.0, 0.0)
    assert mir.adoptions == 0 and mir.chain == 0.0


# --------------------------------------------------------------------------- #
# 5. Parameters: the mirror's defaults are the plugin's
# --------------------------------------------------------------------------- #
MIRROR_TO_PLUGIN = {
    'stall_window': 'se_progress_stall_window',
    'progress_min_advance': 'se_progress_min_advance',
    'progress_adopt_advance': 'se_progress_adopt_advance',
    'goal_reached_tolerance': 'se_goal_reached_tolerance',
    'gap_num_rays': 'se_gap_num_rays',
    'gap_max_range': 'se_gap_max_range',
    'gap_min_clearance': 'se_gap_min_clearance',
    'hysteresis': 'se_escape_hysteresis',
    'waypoint_cap': 'se_escape_waypoint_cap',
    'reselect_radius': 'se_escape_reselect_radius',
    'reselect_timeout_s': 'se_escape_reselect_period',
    'stall_time_s': 'se_escape_stall_time',
    'stall_distance': 'se_escape_stall_distance',
    'avoid_radius': 'se_escape_avoid_radius',
    'avoid_hop_radius': 'se_escape_avoid_hop_radius',
    'avoid_hops': 'se_escape_avoid_hops',
}


def _compiled_defaults():
    src = open(os.path.join(REPO, 'src', 'nav2_se_controller', 'src',
                            'safe_escape_controller.cpp')).read()
    out = {}
    for m in re.finditer(r'(?:getParam|bindConfig)\(\s*[A-Za-z_.]+,\s*"(se_[a-z_]+)",\s*([-0-9.]+)', src):
        out[m.group(1)] = float(m.group(2))
    return out


def test_mirror_defaults_equal_the_compiled_defaults_of_the_plugin():
    plugin = _compiled_defaults()
    for mirror_key, plugin_key in MIRROR_TO_PLUGIN.items():
        assert plugin_key in plugin, plugin_key
        assert float(P[mirror_key]) == plugin[plugin_key], (mirror_key, P[mirror_key], plugin[plugin_key])


def test_mirror_defaults_equal_the_frozen_campaign_file():
    import yaml
    with open(os.path.join(REPO, 'experiments', 'sim', 'nav2_se_campaign2.yaml')) as f:
        fp = yaml.safe_load(f)['controller_server']['ros__parameters']['FollowPath']
    for mirror_key, plugin_key in MIRROR_TO_PLUGIN.items():
        if plugin_key in fp:
            assert float(P[mirror_key]) == float(fp[plugin_key]), (mirror_key, plugin_key)
    # the controller's period is 10 Hz: the stall window of 30 cycles is 3 s as in the paper
    assert fp['model_dt'] == 0.1 and fp['se_progress_stall_window'] == P['stall_window']
