# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for the 2D-benchmark aggregator on synthetic trial rows."""

import math

from experiments.benchmark2d import aggregate as agg


def _row(family, seed, config, success, collided=False, ttg=20.0,
         plen=6.0, minclr=0.3):
    outcome = 'SUCCESS' if success else ('COLLISION' if collided else 'TIMEOUT')
    return {
        'family': family, 'seed': seed, 'config': config, 'outcome': outcome,
        'success': int(success), 'collided': int(collided),
        'reached': int(success), 'time_to_goal': ttg, 'path_length': plen,
        'min_clearance': minclr, 'steps': 100, 'alpha_max': 2.0,
        'alpha_escape_frac': 0.0, 'slack_max': 0.0, 'slack_mean': 0.0,
        'cbf_active_steps': 0, 'entrapped_frac': 0.0,
        'n_obstacles': 5, 'n_dynamic': 0, 'feasible': 1,
    }


def _synthetic():
    rows = []
    # F_full: succeed seeds 0-8, fail seed 9  (9/10)
    for s in range(10):
        rows.append(_row('x', s, 'F_full', success=(s <= 8)))
    # E_indep: succeed seeds 0-6            (7/10)
    for s in range(10):
        rows.append(_row('x', s, 'E_indep', success=(s <= 6)))
    # A_stock: succeed seeds 0-2, seed 9 collides (3/10, 1 collision)
    for s in range(10):
        rows.append(_row('x', s, 'A_stock', success=(s <= 2),
                         collided=(s == 9)))
    return rows


def test_summarize_rates_and_ci():
    summary = agg.summarize(_synthetic())
    idx = {e['config']: e for e in summary}
    assert idx['F_full']['n_trials'] == 10
    assert math.isclose(idx['F_full']['success_rate'], 0.9)
    assert math.isclose(idx['A_stock']['success_rate'], 0.3)
    assert math.isclose(idx['A_stock']['collision_rate'], 0.1)
    # Wilson CI brackets the point estimate.
    e = idx['F_full']
    assert e['success_ci_low'] <= e['success_rate'] <= e['success_ci_high']


def test_time_over_successful_only():
    # time_to_goal / path_length summarized over successful trials only.
    summary = agg.summarize(_synthetic())
    idx = {e['config']: e for e in summary}
    assert idx['F_full']['time_to_goal']['n'] == 9   # 9 successes
    assert idx['F_full']['path_length']['n'] == 9
    # min_clearance summarized over ALL trials.
    assert idx['F_full']['min_clearance']['n'] == 10


def test_mcnemar_f_vs_e_expected_discordant():
    stats = agg.compute_stats(_synthetic())
    comps = {c['baseline']: c for c in stats['x']['comparisons']}
    ef = comps['E_indep']
    # E success seeds 0-6, F success seeds 0-8:
    #   b = E&!F = 0 ; c = !E&F = seeds 7,8 = 2
    assert ef['mcnemar_b'] == 0
    assert ef['mcnemar_c'] == 2
    assert ef['n_pairs'] == 10
    assert 0.0 <= ef['mcnemar_p'] <= 1.0
    assert ef['mcnemar_p_adj'] is not None  # Holm attached


def test_holm_family_covers_all_tests():
    stats = agg.compute_stats(_synthetic())
    # 3 baselines (A,C,D,E minus F itself, but C/D absent here) x (1 success +
    # 3 continuous). Only A_stock and E_indep present as baselines here.
    fam = stats['x']
    assert fam['holm_family_size'] == len(fam['comparisons']) * 4


def test_csv_roundtrip(tmp_path):
    from experiments.benchmark2d import runner
    rows = _synthetic()
    csv_path = tmp_path / 'trials.csv'
    runner.write_csv(rows, str(csv_path))
    loaded = agg.load_trials(str(csv_path))
    assert len(loaded) == len(rows)
    assert loaded[0]['success'] in (0, 1)
    assert isinstance(loaded[0]['time_to_goal'], float)
