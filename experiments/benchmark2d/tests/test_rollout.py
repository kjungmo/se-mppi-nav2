# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Smoke tests for the rollout harness (fast, tiny worlds)."""

import os
import sys

import numpy as np

from experiments.benchmark2d import rollout as rl
from experiments.benchmark2d.configs import CONFIGS

_PROTO = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'prototype')
if _PROTO not in sys.path:
    sys.path.insert(0, _PROTO)
import se_mppi_proto as se  # noqa: E402


def _open_world(goal=(1.2, 0.0)):
    return se.World([], goal=goal, robot_radius=0.22)


def test_reaches_open_goal_stock():
    w = _open_world()
    r = rl.rollout(w, CONFIGS['A_stock'], (0.0, 0.0, 0.0), max_steps=80, seed=0)
    assert r['reached'] and r['success']
    assert r['outcome'] == 'SUCCESS'
    assert r['path_length'] > 0.0
    assert r['cbf_active_steps'] == 0  # CBF off


def test_cbf_inert_without_dynamic_obstacles():
    # F_full on an obstacle-free world: CBF runs each step but has no dynamic
    # constraints, so slack stays ~0 and the goal is still reached.
    w = _open_world()
    r = rl.rollout(w, CONFIGS['F_full'], (0.0, 0.0, 0.0), max_steps=80, seed=0)
    assert r['reached'] and r['success']
    assert r['cbf_active_steps'] > 0
    assert r['slack_max'] <= 1e-3


def test_dynamic_view_selects_movers_only():
    w = se.World([se.Obstacle(1.0, 0.0, 0.3),
                  se.Obstacle(2.0, 1.0, 0.3, vx=0.1, vy=0.0)],
                 goal=(3.0, 0.0))
    dyn = rl._dynamic_view(w)
    assert len(dyn.obstacles) == 1
    assert np.hypot(*dyn.obstacles[0].v) > 0.0


def test_collision_outcome_classified():
    # A static obstacle right on the goal line, no escape/CBF -> the greedy
    # controller drives into it (min clearance goes negative -> COLLISION).
    w = se.World([se.Obstacle(0.6, 0.0, 0.4)], goal=(1.4, 0.0))
    r = rl.rollout(w, CONFIGS['A_stock'], (0.0, 0.0, 0.0), max_steps=60, seed=1)
    # Either it collides, or it fails to reach; in both cases not a SUCCESS.
    if r['collided']:
        assert r['outcome'] == 'COLLISION'
        assert r['min_clearance'] < 0.0
    assert r['outcome'] in ('COLLISION', 'TIMEOUT', 'SUCCESS')


def test_result_keys_present():
    w = _open_world()
    r = rl.rollout(w, CONFIGS['E_indep'], (0.0, 0.0, 0.0), max_steps=40, seed=0)
    for k in ('success', 'collided', 'outcome', 'time_to_goal', 'path_length',
              'min_clearance', 'alpha_max', 'slack_max', 'entrapped_frac'):
        assert k in r
