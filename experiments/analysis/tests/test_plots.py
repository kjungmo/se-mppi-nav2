# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Offline tests for the comparison plots (design §6, protocol §8; H-6).

Runs on synthetic ``aggregate.summarize`` output, asserts each call writes a
non-empty PNG. Headless (Agg backend) — no display, no scipy.
"""

import os

from experiments.analysis import plots


def _summary_row(tier, config, sr, ci_lo, ci_hi, coll=0.0, tout=0.0):
    return {
        'tier': tier, 'config': config, 'n_trials': 20, 'n_setup_fail': 0,
        'success_rate': sr, 'success_ci_low': ci_lo, 'success_ci_high': ci_hi,
        'collision_rate': coll, 'timeout_rate': tout,
        'time_to_goal': {'median': 12.0, 'q1': 10.0, 'q3': 14.0, 'n': 20},
    }


def _summary():
    return [
        _summary_row('barn', 'A_stock', 0.55, 0.40, 0.70, coll=0.05, tout=0.20),
        _summary_row('barn', 'E_escape_cbf_indep', 0.70, 0.55, 0.82, coll=0.10),
        _summary_row('barn', 'F_se_full', 0.90, 0.78, 0.96, coll=0.02),
    ]


def _nonempty_png(path):
    assert os.path.exists(path)
    with open(path, 'rb') as f:
        head = f.read(8)
    assert head[:4] == b'\x89PNG'           # valid PNG signature
    assert os.path.getsize(path) > 0


def test_plot_tier_outcomes_writes_png(tmp_path):
    out = str(tmp_path / 'outcomes_barn.png')
    got = plots.plot_tier_outcomes(_summary(), 'barn', out)
    assert got == out
    _nonempty_png(out)


def test_plot_e_vs_f_writes_png(tmp_path):
    out = str(tmp_path / 'e_vs_f_barn.png')
    plots.plot_e_vs_f(_summary(), 'barn', out)
    _nonempty_png(out)


def test_plot_e_vs_f_with_mcnemar_annotation(tmp_path):
    # 20 paired trials: F wins 6 that E lost, E wins 1 that F lost.
    e = [True] * 13 + [False] * 7
    f = [True] * 13 + [True] * 6 + [False] * 1
    out = str(tmp_path / 'e_vs_f_annot.png')
    plots.plot_e_vs_f(_summary(), 'barn', out, paired_outcomes=(e, f))
    _nonempty_png(out)


def test_plot_all_one_pair_per_tier(tmp_path):
    summ = _summary() + [
        _summary_row('hunav', 'E_escape_cbf_indep', 0.6, 0.45, 0.74),
        _summary_row('hunav', 'F_se_full', 0.85, 0.72, 0.93),
    ]
    paths = plots.plot_all(summ, str(tmp_path))
    # 2 tiers × (outcomes + e_vs_f) = 4 figures
    assert len(paths) == 4
    for p in paths:
        _nonempty_png(p)
    names = {os.path.basename(p) for p in paths}
    assert {'outcomes_barn.png', 'e_vs_f_barn.png',
            'outcomes_hunav.png', 'e_vs_f_hunav.png'} == names


def test_plot_from_results_end_to_end(tmp_path):
    # Lay down a couple of raw trial JSONs, then aggregate+plot.
    import json
    rec = {'scenario': 's1', 'tier': 'barn', 'config': 'F_se_full', 'seed': 0,
           'outcome': 'SUCCESS', 'setup_attempts': 1,
           'metrics': {'success': True, 'collided': False, 'reached_goal': True,
                       'time_to_goal': 10.0, 'path_length': 5.0,
                       'min_clearance': 0.3, 'spl': 1.0, 'barn_score': 0.5,
                       'oscillation_ratio': 0.0,
                       'compute': {'mean_ms': 12.0, 'p95_ms': 18.0},
                       'smoothness': {'lin_jerk_mean': 0.1}}}
    d = tmp_path / 'barn' / 'F_se_full'
    d.mkdir(parents=True)
    with open(d / 's1_seed0.json', 'w') as f:
        json.dump(rec, f)
    paths = plots.plot_from_results(str(tmp_path))
    assert paths and all(os.path.exists(p) for p in paths)
    # Default out dir is <results>/figures.
    assert all(os.path.dirname(p).endswith('figures') for p in paths)
