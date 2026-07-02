# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for trial metrics + classification (protocol §4, design §2)."""

import math

from experiments.runner import metrics as M


def _straight_run(n=11, x0=0.0, x1=5.0, y=0.0, dt=0.5, obstacles=None):
    samples = []
    for i in range(n):
        f = i / (n - 1)
        s = {'t': i * dt, 'x': x0 + f * (x1 - x0), 'y': y,
             'yaw': 0.0, 'v': 0.5, 'w': 0.0, 'loop_ms': 10.0 + i}
        if obstacles is not None:
            s['obstacles'] = obstacles
        samples.append(s)
    return samples


def test_success_clean_run():
    s = _straight_run()
    m = M.compute_metrics(s, goal=(5.0, 0.0), goal_tol=0.3, robot_radius=0.22)
    assert m['reached_goal'] and not m['collided'] and m['success']
    assert math.isclose(m['path_length'], 5.0, rel_tol=1e-6)
    assert math.isclose(m['time_to_goal'], 5.0, rel_tol=1e-6)
    assert M.classify(m) == M.SUCCESS


def test_collision_detected_from_gt_obstacle():
    # An obstacle sitting on the path: clearance goes negative.
    obstacles = [{'x': 2.5, 'y': 0.0, 'r': 0.3}]
    s = _straight_run(obstacles=obstacles)
    m = M.compute_metrics(s, goal=(5.0, 0.0), goal_tol=0.3, robot_radius=0.22)
    assert m['min_clearance'] < 0.0
    assert m['collided']
    assert not m['success']
    assert M.classify(m) == M.COLLISION


def test_collision_dominates_timeout():
    obstacles = [{'x': 2.5, 'y': 0.0, 'r': 0.3}]
    s = _straight_run(obstacles=obstacles)
    m = M.compute_metrics(s, goal=(5.0, 0.0), goal_tol=0.3, robot_radius=0.22)
    assert M.classify(m, timeout=True) == M.COLLISION


def test_not_reached_is_stuck_then_timeout():
    s = _straight_run(x1=2.0)  # stops short of goal at x=5
    m = M.compute_metrics(s, goal=(5.0, 0.0), goal_tol=0.3, robot_radius=0.22)
    assert not m['reached_goal'] and not m['collided']
    assert M.classify(m) == M.STUCK
    assert M.classify(m, timeout=True) == M.TIMEOUT


def test_setup_fail_overrides_everything():
    s = _straight_run()
    m = M.compute_metrics(s, goal=(5.0, 0.0), goal_tol=0.3, robot_radius=0.22)
    assert M.classify(m, setup_failed=True) == M.SETUP_FAIL


def test_clearance_positive_when_obstacle_far():
    obstacles = [{'x': 2.5, 'y': 5.0, 'r': 0.3}]
    clr = M.clearance_series(_straight_run(obstacles=obstacles), 0.22)
    assert min(clr) > 0.0


def test_spl_and_barn_bounds():
    # Optimal straight path: spl == 1, barn == 0.5 (AT == OT clipped to 2·OT).
    s = _straight_run()
    m = M.compute_metrics(s, goal=(5.0, 0.0), goal_tol=0.3, robot_radius=0.22,
                          optimal_length=5.0, optimal_time=5.0)
    assert math.isclose(m['spl'], 1.0, rel_tol=1e-6)
    assert math.isclose(m['barn_score'], 0.5, rel_tol=1e-6)  # OT/clip(OT,2OT,8OT)


def test_spl_zero_on_failure():
    s = _straight_run(x1=2.0)
    m = M.compute_metrics(s, goal=(5.0, 0.0), goal_tol=0.3, robot_radius=0.22)
    assert m['spl'] == 0.0
    assert m['barn_score'] == 0.0


def test_compute_time_percentiles():
    s = _straight_run(n=21)  # loop_ms = 10..30
    c = M.compute_time_stats(s)
    assert c['n'] == 21
    assert c['max_ms'] == 30.0
    assert 10.0 <= c['median_ms'] <= 30.0
    assert c['p95_ms'] >= c['median_ms']


def test_oscillation_ratio_spin_in_place():
    s = [{'t': i * 0.1, 'x': 0.0, 'y': 0.0, 'v': 0.0, 'w': 1.0} for i in range(10)]
    assert M.oscillation_ratio(s) == 1.0
    moving = [{'t': i * 0.1, 'x': i * 0.1, 'y': 0.0, 'v': 0.5, 'w': 0.0}
              for i in range(10)]
    assert M.oscillation_ratio(moving) == 0.0


def test_empty_samples_safe():
    m = M.compute_metrics([], goal=(1.0, 0.0), goal_tol=0.3, robot_radius=0.22)
    assert m['n_samples'] == 0
    assert not m['success']
    assert M.classify(m) == M.STUCK
