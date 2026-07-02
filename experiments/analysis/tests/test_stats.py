# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for the statistics (protocol §6), cross-checked against scipy."""

import math

import pytest

from experiments.analysis import stats as S


def test_mcnemar_exact_known_value():
    # b=10, c=2 -> exact two-sided binomial p ~ 0.03857.
    res = S.mcnemar(10, 2)
    assert res.detail['method'] == 'exact-binomial'
    assert math.isclose(res.p_value, 0.03857, abs_tol=1e-4)


def test_mcnemar_degenerate():
    assert S.mcnemar(0, 0).p_value == 1.0


def test_mcnemar_matches_scipy_exact():
    sp = pytest.importorskip('scipy.stats')
    for b, c in [(8, 1), (12, 3), (5, 5), (20, 4)]:
        ours = S.mcnemar(b, c).p_value
        ref = sp.binomtest(min(b, c), b + c, 0.5).pvalue
        assert math.isclose(ours, ref, abs_tol=1e-9)


def test_paired_success_counts():
    a = [True, True, False, False, True]
    b = [False, True, True, False, False]
    bb, cc = S.paired_success_counts(a, b)
    assert (bb, cc) == (2, 1)  # A&!B at idx0,4 ; !A&B at idx2


def test_cliffs_delta_extremes():
    assert S.cliffs_delta([5, 6, 7], [1, 2, 3]) == 1.0
    assert S.cliffs_delta([1, 2, 3], [5, 6, 7]) == -1.0
    assert S.cliffs_delta([1, 2, 3], [1, 2, 3]) == 0.0


def test_mann_whitney_matches_scipy():
    sp = pytest.importorskip('scipy.stats')
    x = [1.2, 2.3, 3.1, 4.8, 2.2, 5.0, 3.3]
    y = [2.1, 2.9, 1.1, 0.8, 3.0, 1.5]
    ours = S.mann_whitney_u(x, y)
    ref = sp.mannwhitneyu(x, y, alternative='two-sided',
                          use_continuity=True, method='asymptotic')
    # U statistic: ours reports min(U_x, U_y); scipy reports U_x.
    assert math.isclose(ours.detail['U_x'], ref.statistic, abs_tol=1e-6)
    assert math.isclose(ours.p_value, ref.pvalue, rel_tol=1e-2, abs_tol=1e-3)


def test_mann_whitney_with_ties_matches_scipy():
    sp = pytest.importorskip('scipy.stats')
    x = [1, 2, 2, 3, 3, 3]
    y = [2, 3, 3, 4, 5]
    ours = S.mann_whitney_u(x, y)
    ref = sp.mannwhitneyu(x, y, alternative='two-sided',
                          use_continuity=True, method='asymptotic')
    assert math.isclose(ours.p_value, ref.pvalue, rel_tol=2e-2, abs_tol=2e-3)


def test_holm_monotone_and_rejections():
    pvals = {'a': 0.001, 'b': 0.04, 'c': 0.03, 'd': 0.005}
    res = S.holm(pvals, alpha=0.05)
    # Adjusted p is non-decreasing in raw-p order; smallest clearly rejected.
    assert res['a']['reject'] is True
    assert res['a']['p_adj'] <= res['d']['p_adj'] <= res['c']['p_adj']
    # 0.04 * (rank) likely not significant after correction.
    assert res['b']['p_adj'] >= res['b']['p_raw']


def test_holm_list_form():
    res = S.holm([0.01, 0.02, 0.03])
    assert isinstance(res, list) and len(res) == 3
    assert all('p_adj' in r for r in res)
