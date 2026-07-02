# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for the randomized scenario generators."""

import numpy as np

from experiments.benchmark2d import scenarios as sc


def _geometry(scn):
    return [(round(o.p[0], 6), round(o.p[1], 6), round(o.r, 6),
             round(o.v[0], 6), round(o.v[1], 6)) for o in scn.world.obstacles]


def test_determinism_same_seed_same_world():
    for fam in ('utrap', 'clutter', 'dynamic', 'narrowdyn'):
        a = sc.make(fam, 7)
        b = sc.make(fam, 7)
        assert _geometry(a) == _geometry(b)
        assert a.start == b.start and a.goal == b.goal


def test_distinct_seeds_differ():
    a = sc.make('clutter', 1)
    b = sc.make('clutter', 2)
    assert _geometry(a) != _geometry(b)


def test_clutter_always_feasible():
    # The generator drops blocking obstacles until a free path exists.
    for seed in range(25):
        scn = sc.make('clutter', seed)
        assert scn.meta['feasible'] is True
        assert sc.free_path_exists(scn.world.obstacles, scn.start[:2], scn.goal)


def test_utrap_feasible_and_has_pocket():
    for seed in range(25):
        scn = sc.make('utrap', seed)
        # A finite pocket is always escapable (walls do not reach the bounds).
        assert scn.meta['feasible'] is True
        assert scn.meta['n_obstacles'] >= 6
        assert scn.meta['n_dynamic'] == 0


def test_dynamic_has_movers():
    for seed in range(25):
        scn = sc.make('dynamic', seed)
        movers = [o for o in scn.world.obstacles
                  if np.hypot(o.v[0], o.v[1]) > 1e-9]
        assert len(movers) >= 1
        assert scn.meta['n_dynamic'] == len(movers)


def test_free_path_detects_full_block():
    # A solid wall spanning the whole corridor blocks the path.
    wall = sc._wall((2.0, -4.0), (2.0, 4.0), r=0.35, spacing=0.4)
    assert not sc.free_path_exists(wall, (0.0, 0.0), (4.0, 0.0))
    # A gap in the wall admits a path.
    gapped = [o for o in wall if abs(o.p[1]) > 0.8]
    assert sc.free_path_exists(gapped, (0.0, 0.0), (4.0, 0.0))


def test_narrowdyn_reuses_utrap_pocket_and_adds_movers():
    for seed in range(20):
        nd = sc.make('narrowdyn', seed)
        ut = sc.make('utrap', seed)
        # same start/goal and same static pocket as the utrap of that seed
        assert nd.start == ut.start and nd.goal == ut.goal
        statics = [o for o in nd.world.obstacles
                   if np.hypot(o.v[0], o.v[1]) <= 1e-9]
        assert len(statics) == len(ut.world.obstacles)
        movers = [o for o in nd.world.obstacles
                  if np.hypot(o.v[0], o.v[1]) > 1e-9]
        assert len(movers) >= 1
        assert nd.meta['n_dynamic'] == len(movers)
        assert nd.meta['feasible'] is True   # pocket still escapable


def test_start_heading_points_at_goal():
    scn = sc.make('utrap', 3)
    sx, sy, th = scn.start
    gx, gy = scn.goal
    assert abs(np.arctan2(gy - sy, gx - sx) - th) < 1e-6
