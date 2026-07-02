# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Statistical tests for the SE-MPPI comparison (protocol §6).

Pure-Python (math only) so results are deterministic and dependency-light:

  * ``mcnemar``        — paired binary outcomes (success rate), the primary
                         E-vs-F hypothesis test. Exact two-sided binomial for
                         small discordant counts, χ²(1) with continuity
                         correction otherwise.
  * ``mann_whitney_u`` — unpaired continuous metrics (time, collision distance),
                         normal approximation with tie correction.
  * ``cliffs_delta``   — non-parametric effect size accompanying Mann–Whitney.
  * ``holm``           — Holm–Bonferroni multiple-comparison correction across
                         the 9 configs × 3 tiers family.

scipy is available in the environment and used only by the tests to cross-check.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def _norm_sf(z: float) -> float:
    """Survival function of the standard normal (1 − Φ)."""
    return 0.5 * math.erfc(z / math.sqrt(2.0))


def _binom_two_sided_p(b: int, c: int) -> float:
    """Exact two-sided p for McNemar via Binomial(n=b+c, p=0.5)."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    # P(X <= k) under Bin(n, 0.5); two-sided by doubling (capped at 1).
    cdf = sum(math.comb(n, i) for i in range(0, k + 1)) / (2.0 ** n)
    return min(1.0, 2.0 * cdf)


@dataclass
class TestResult:
    statistic: float
    p_value: float
    detail: dict


def mcnemar(b: int, c: int, exact_max_n: int = 25) -> TestResult:
    """McNemar's test on discordant pair counts.

    ``b`` = #(A success, B failure), ``c`` = #(A failure, B success). Concordant
    pairs are irrelevant. Returns the χ² statistic (with continuity correction)
    and a p-value: exact binomial when b+c ≤ ``exact_max_n``, else χ²(1).
    """
    n = b + c
    if n == 0:
        return TestResult(0.0, 1.0, {'method': 'degenerate', 'n_discordant': 0})
    chi2 = (abs(b - c) - 1.0) ** 2 / n  # continuity-corrected
    if n <= exact_max_n:
        p = _binom_two_sided_p(b, c)
        method = 'exact-binomial'
    else:
        # χ²(1) survival = 2·(1 − Φ(√χ²)).
        p = 2.0 * _norm_sf(math.sqrt(chi2))
        p = min(1.0, p)
        method = 'chi2-continuity'
    return TestResult(chi2, p, {'method': method, 'b': b, 'c': c, 'n_discordant': n})


def cliffs_delta(x, y) -> float:
    """Cliff's δ effect size: P(X>Y) − P(X<Y) ∈ [−1, 1]."""
    x = list(x)
    y = list(y)
    if not x or not y:
        return 0.0
    gt = lt = 0
    for a in x:
        for b in y:
            if a > b:
                gt += 1
            elif a < b:
                lt += 1
    return (gt - lt) / (len(x) * len(y))


def _rankdata(vals):
    """Average ranks (1-based), handling ties — like scipy.stats.rankdata."""
    order = sorted(range(len(vals)), key=lambda i: vals[i])
    ranks = [0.0] * len(vals)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and vals[order[j + 1]] == vals[order[i]]:
            j += 1
        avg = (i + j) / 2.0 + 1.0  # average of ranks i+1..j+1
        for k in range(i, j + 1):
            ranks[order[k]] = avg
        i = j + 1
    return ranks


def mann_whitney_u(x, y) -> TestResult:
    """Two-sided Mann–Whitney U with tie-corrected normal approximation.

    Returns U (for sample ``x``), an asymptotic two-sided p-value with
    continuity correction, and Cliff's δ in ``detail``.
    """
    x = list(x)
    y = list(y)
    nx, ny = len(x), len(y)
    if nx == 0 or ny == 0:
        return TestResult(0.0, 1.0, {'method': 'degenerate', 'delta': 0.0})

    ranks = _rankdata(x + y)
    rank_x = sum(ranks[:nx])
    u_x = rank_x - nx * (nx + 1) / 2.0
    u_y = nx * ny - u_x
    u = min(u_x, u_y)

    mean_u = nx * ny / 2.0
    n = nx + ny
    # Tie correction term.
    _, counts = _value_counts(x + y)
    tie_term = sum(t ** 3 - t for t in counts)
    var_u = (nx * ny / 12.0) * ((n + 1) - tie_term / (n * (n - 1))) if n > 1 else 0.0
    if var_u <= 0.0:
        return TestResult(u, 1.0, {'method': 'zero-variance',
                                   'delta': cliffs_delta(x, y), 'U_x': u_x})
    z = (abs(u - mean_u) - 0.5) / math.sqrt(var_u)  # continuity correction
    z = max(z, 0.0)
    p = min(1.0, 2.0 * _norm_sf(z))
    return TestResult(u, p, {'method': 'normal-approx', 'U_x': u_x, 'U_y': u_y,
                             'z': z, 'delta': cliffs_delta(x, y)})


def _value_counts(vals):
    counts = {}
    for v in vals:
        counts[v] = counts.get(v, 0) + 1
    keys = list(counts.keys())
    return keys, [counts[k] for k in keys]


def holm(pvalues, alpha: float = 0.05):
    """Holm–Bonferroni step-down correction.

    Takes a dict or list of raw p-values; returns the same shape with each
    entry mapped to ``{'p_raw', 'p_adj', 'reject'}``. Adjusted p-values are
    monotone (enforced step-down) and clipped to 1.
    """
    if isinstance(pvalues, dict):
        keys = list(pvalues.keys())
        raw = [pvalues[k] for k in keys]
    else:
        keys = list(range(len(pvalues)))
        raw = list(pvalues)
    m = len(raw)
    order = sorted(range(m), key=lambda i: raw[i])
    adj = [0.0] * m
    running = 0.0
    for rank, idx in enumerate(order):
        val = (m - rank) * raw[idx]
        running = max(running, val)          # enforce monotonicity
        adj[idx] = min(1.0, running)
    result = {keys[i]: {'p_raw': raw[i], 'p_adj': adj[i], 'reject': adj[i] < alpha}
              for i in range(m)}
    if not isinstance(pvalues, dict):
        return [result[i] for i in range(m)]
    return result


def paired_success_counts(a_success, b_success):
    """Discordant counts (b, c) for McNemar from two aligned boolean lists.

    Returns ``(b, c)`` where b = A-success & B-fail, c = A-fail & B-success.
    """
    if len(a_success) != len(b_success):
        raise ValueError('paired lists must be the same length')
    b = sum(1 for a, bb in zip(a_success, b_success) if a and not bb)
    c = sum(1 for a, bb in zip(a_success, b_success) if not a and bb)
    return b, c
