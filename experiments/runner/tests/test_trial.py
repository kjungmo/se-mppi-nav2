# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for the single-trial state machine (design §2, §7) via FakeLauncher."""

from experiments.runner import metrics as M
from experiments.runner.scenario import Scenario
from experiments.runner.trial import (DriveResult, FakeLauncher, TrialConfig,
                                       run_trial)


def _scenario():
    return Scenario(name='demo', tier='barn', map_yaml='/tmp/none.yaml',
                    start=(0.0, 0.0, 0.0), goal=(5.0, 0.0, 0.0),
                    optimal_length=5.0, optimal_time=5.0)


def _reach_samples():
    return [{'t': i * 0.5, 'x': i * 0.5, 'y': 0.0, 'v': 0.5, 'w': 0.0,
             'loop_ms': 12.0} for i in range(11)]


def _no_cleanup_cfg():
    return TrialConfig()


def test_successful_trial():
    launcher = FakeLauncher(DriveResult(samples=_reach_samples()))
    rec = run_trial(launcher, _scenario(), 'F_se_full', {}, seed=0,
                    do_cleanup=False)
    assert rec.outcome == M.SUCCESS
    assert rec.setup_attempts == 1
    assert launcher.teardowns == 1
    assert rec.metrics['success']


def test_collision_trial():
    samples = _reach_samples()
    for s in samples:
        s['obstacles'] = [{'x': 2.5, 'y': 0.0, 'r': 0.3}]
    launcher = FakeLauncher(DriveResult(samples=samples))
    rec = run_trial(launcher, _scenario(), 'A_stock', {}, seed=1,
                    do_cleanup=False)
    assert rec.outcome == M.COLLISION


def test_timeout_trial():
    short = _reach_samples()[:4]  # stops short of goal
    launcher = FakeLauncher(DriveResult(samples=short, timeout=True))
    rec = run_trial(launcher, _scenario(), 'A_stock', {}, seed=2,
                    do_cleanup=False)
    assert rec.outcome == M.TIMEOUT


def test_setup_fail_after_retry():
    # wait_active returns False for every attempt -> SETUP_FAIL, retried once.
    launcher = FakeLauncher(DriveResult(samples=_reach_samples()),
                            fail_setup_times=99)
    cfg = TrialConfig(setup_retries=1)
    rec = run_trial(launcher, _scenario(), 'F_se_full', {}, seed=3, cfg=cfg,
                    do_cleanup=False)
    assert rec.outcome == M.SETUP_FAIL
    assert rec.setup_attempts == 2          # initial + one retry
    assert launcher.launches == 2


def test_setup_recovers_on_retry():
    # First activation fails, second succeeds -> SUCCESS on attempt 2.
    launcher = FakeLauncher(DriveResult(samples=_reach_samples()),
                            fail_setup_times=1)
    rec = run_trial(launcher, _scenario(), 'F_se_full', {}, seed=4,
                    do_cleanup=False)
    assert rec.outcome == M.SUCCESS
    assert rec.setup_attempts == 2


def test_launch_crash_is_setup_fail():
    launcher = FakeLauncher(DriveResult(samples=_reach_samples()),
                            raise_setup_times=99)
    rec = run_trial(launcher, _scenario(), 'F_se_full', {}, seed=5,
                    do_cleanup=False)
    assert rec.outcome == M.SETUP_FAIL
    assert rec.error is not None and 'simulated launch crash' in rec.error


def test_record_serializes():
    launcher = FakeLauncher(DriveResult(samples=_reach_samples()))
    rec = run_trial(launcher, _scenario(), 'F_se_full', {}, seed=0,
                    do_cleanup=False)
    d = rec.to_dict()
    assert d['outcome'] == M.SUCCESS
    assert set(d) >= {'scenario', 'tier', 'config', 'seed', 'outcome', 'metrics'}
