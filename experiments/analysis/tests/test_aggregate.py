# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for result aggregation + summary tables (design §6, §7)."""

import csv

from experiments.analysis import aggregate as A


def _rec(config, success, collided=False, outcome=None, tier='barn',
         ttg=10.0, plen=5.0, clr=0.3):
    if outcome is None:
        outcome = 'SUCCESS' if success else ('COLLISION' if collided else 'STUCK')
    return {
        'scenario': 's1', 'tier': tier, 'config': config, 'seed': 0,
        'outcome': outcome, 'setup_attempts': 1,
        'metrics': {
            'success': success, 'collided': collided, 'reached_goal': success,
            'time_to_goal': ttg, 'path_length': plen, 'min_clearance': clr,
            'spl': 1.0 if success else 0.0, 'barn_score': 0.5 if success else 0.0,
            'oscillation_ratio': 0.0,
            'compute': {'mean_ms': 12.0, 'p95_ms': 18.0},
            'smoothness': {'lin_jerk_mean': 0.1},
        },
    }


def test_summarize_success_rate_and_wilson():
    rows = [_rec('F_se_full', True) for _ in range(8)] + \
           [_rec('F_se_full', False) for _ in range(2)]
    summ = {s['config']: s for s in A.summarize(rows)}
    f = summ['F_se_full']
    assert f['n_trials'] == 10
    assert f['success_rate'] == 0.8
    assert 0.0 <= f['success_ci_low'] <= 0.8 <= f['success_ci_high'] <= 1.0
    assert f['collision_rate'] == 0.0


def test_setup_fail_excluded_from_denominator():
    rows = [_rec('A_stock', True) for _ in range(4)]
    rows += [_rec('A_stock', False, outcome='SETUP_FAIL') for _ in range(2)]
    summ = {s['config']: s for s in A.summarize(rows)}
    a = summ['A_stock']
    assert a['n_trials'] == 4          # setup fails removed
    assert a['n_setup_fail'] == 2
    assert a['success_rate'] == 1.0    # 4/4, not 4/6


def test_collision_rate():
    rows = [_rec('A_stock', False, collided=True) for _ in range(3)] + \
           [_rec('A_stock', True) for _ in range(1)]
    a = {s['config']: s for s in A.summarize(rows)}['A_stock']
    assert a['collision_rate'] == 0.75
    assert a['success_rate'] == 0.25


def test_median_iqr_continuous():
    rows = [_rec('F_se_full', True, ttg=t) for t in (5, 10, 15, 20, 25)]
    f = {s['config']: s for s in A.summarize(rows)}['F_se_full']
    assert f['time_to_goal']['median'] == 15
    assert f['time_to_goal']['q1'] == 10
    assert f['time_to_goal']['q3'] == 20


def test_wilson_interval_edges():
    p, lo, hi = A.wilson_interval(0, 0)
    assert (p, lo, hi) == (0.0, 0.0, 0.0)
    p, lo, hi = A.wilson_interval(10, 10)
    assert p == 1.0 and hi == 1.0 and lo < 1.0


def test_write_long_csv(tmp_path):
    rows = [_rec('A_stock', True), _rec('F_se_full', False, collided=True)]
    out = tmp_path / 'summary.csv'
    A.write_long_csv(rows, str(out))
    with open(out) as f:
        got = list(csv.DictReader(f))
    assert len(got) == 2
    assert got[0]['config'] == 'A_stock'
    assert got[0]['success'] == 'True'
    assert got[1]['collided'] == 'True'


def test_flatten_handles_missing_metrics():
    row = A.flatten({'scenario': 's', 'tier': 'barn', 'config': 'A',
                     'seed': 0, 'outcome': 'SETUP_FAIL'})
    assert row['success'] is False
    assert row['time_to_goal'] is None
