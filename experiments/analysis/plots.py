# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Comparison plots for the SE-MPPI paper (design §6, protocol §8; H-6).

Consumes the per-(tier, config) summary rows produced by ``aggregate.summarize``
(success_rate + Wilson CI, collision_rate, timeout_rate, median±IQR continuous
metrics) and renders the figures the paper's §VI tables/figures call for:

  * per-tier grouped bars of success / collision / timeout across configs
    (one figure per tier), success bars carrying Wilson CI error bars;
  * the **E-vs-F coordination contrast** — the load-bearing hypothesis figure —
    putting ``E_escape_cbf_indep`` next to ``F_se_full`` for success (with CI)
    and collision, optionally annotated with the McNemar p-value from
    ``stats.mcnemar`` when paired per-trial outcomes are supplied.

Pure matplotlib on the **Agg** backend (set before pyplot import) so it runs
headless in CI with no display and no scipy (scipy is absent in the harness env).
Each function writes a PNG and returns its path; ``plot_all`` is the one-call
entry the runner/analysis driver uses.
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use('Agg')                       # headless: no DISPLAY required
import matplotlib.pyplot as plt             # noqa: E402  (after backend select)

from . import aggregate as agg_mod          # noqa: E402
from . import stats as stats_mod            # noqa: E402

# The coordination contrast (protocol §3 "핵심 비교 E vs F").
E_CONFIG = 'E_escape_cbf_indep'
F_CONFIG = 'F_se_full'


def _ci_err(summary_row) -> tuple:
    """(lower, upper) error-bar magnitudes from a summary row's Wilson CI."""
    p = summary_row.get('success_rate', 0.0)
    lo = summary_row.get('success_ci_low', p)
    hi = summary_row.get('success_ci_high', p)
    return (max(0.0, p - lo), max(0.0, hi - p))


def _by_tier(summary: list) -> dict:
    out: dict = {}
    for row in summary:
        out.setdefault(row.get('tier'), []).append(row)
    return out


def plot_tier_outcomes(summary: list, tier: str, out_path: str) -> str:
    """Grouped success/collision/timeout bars across configs for one tier."""
    rows = [r for r in summary if r.get('tier') == tier]
    rows.sort(key=lambda r: str(r.get('config')))
    configs = [r.get('config') for r in rows]
    success = [r.get('success_rate', 0.0) for r in rows]
    collision = [r.get('collision_rate', 0.0) for r in rows]
    timeout = [r.get('timeout_rate', 0.0) for r in rows]
    lo = [_ci_err(r)[0] for r in rows]
    hi = [_ci_err(r)[1] for r in rows]

    x = range(len(configs))
    w = 0.27
    fig, ax = plt.subplots(figsize=(max(6.0, 1.1 * len(configs)), 4.0))
    ax.bar([i - w for i in x], success, width=w, label='success',
           color='#22aa77', yerr=[lo, hi], capsize=3)
    ax.bar(list(x), collision, width=w, label='collision', color='#cc3333')
    ax.bar([i + w for i in x], timeout, width=w, label='timeout', color='#ffaa00')
    ax.set_xticks(list(x))
    ax.set_xticklabels(configs, rotation=30, ha='right', fontsize=8)
    ax.set_ylim(0, 1.0)
    ax.set_ylabel('rate')
    ax.set_title(f'SE-MPPI outcomes — tier "{tier}" (success CI = Wilson 95%)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def plot_e_vs_f(summary: list, tier: str, out_path: str, *,
                paired_outcomes: tuple | None = None) -> str:
    """The E-vs-F contrast for one tier: success (with CI) + collision side by
    side, the figure that visualises the coordination hypothesis (H).

    ``paired_outcomes`` = ``(e_success_bools, f_success_bools)`` aligned per
    trial; when given, the McNemar p-value (``stats.mcnemar``) is annotated.
    """
    idx = {r.get('config'): r for r in summary if r.get('tier') == tier}
    e, f = idx.get(E_CONFIG), idx.get(F_CONFIG)
    fig, ax = plt.subplots(figsize=(5.0, 4.0))
    labels = ['E: escape+CBF\n(independent)', 'F: SE-MPPI\n(coordinated)']
    succ = [(e or {}).get('success_rate', 0.0), (f or {}).get('success_rate', 0.0)]
    coll = [(e or {}).get('collision_rate', 0.0),
            (f or {}).get('collision_rate', 0.0)]
    elo = _ci_err(e)[0] if e else 0.0
    ehi = _ci_err(e)[1] if e else 0.0
    flo = _ci_err(f)[0] if f else 0.0
    fhi = _ci_err(f)[1] if f else 0.0

    x = [0, 1]
    w = 0.35
    ax.bar([i - w / 2 for i in x], succ, width=w, label='success', color='#22aa77',
           yerr=[[elo, flo], [ehi, fhi]], capsize=4)
    ax.bar([i + w / 2 for i in x], coll, width=w, label='collision', color='#cc3333')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylim(0, 1.0)
    ax.set_ylabel('rate')
    ax.set_title(f'Coordination contrast (E vs F) — tier "{tier}"')
    ax.legend(fontsize=8)

    if paired_outcomes is not None:
        e_succ, f_succ = paired_outcomes
        b, c = stats_mod.paired_success_counts(e_succ, f_succ)
        res = stats_mod.mcnemar(b, c)
        ax.annotate(f'McNemar p = {res.p_value:.3g}  (b={b}, c={c})',
                    xy=(0.5, 0.95), xycoords='axes fraction', ha='center',
                    fontsize=8, color='#333333')
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def plot_all(summary: list, out_dir: str, *,
             paired_by_tier: dict | None = None) -> list:
    """Render every figure for a summary; returns the list of PNG paths.

    One outcome figure per tier + one E-vs-F figure per tier. ``paired_by_tier``
    maps a tier to ``(e_success, f_success)`` paired booleans for the McNemar
    annotation (optional).
    """
    os.makedirs(out_dir, exist_ok=True)
    paired_by_tier = paired_by_tier or {}
    paths = []
    for tier in sorted(_by_tier(summary)):
        paths.append(plot_tier_outcomes(
            summary, tier, os.path.join(out_dir, f'outcomes_{tier}.png')))
        paths.append(plot_e_vs_f(
            summary, tier, os.path.join(out_dir, f'e_vs_f_{tier}.png'),
            paired_outcomes=paired_by_tier.get(tier)))
    return paths


def plot_from_results(results_dir: str, out_dir: str | None = None) -> list:
    """End-to-end: aggregate ``results_dir`` then render all figures.

    Default ``out_dir`` is ``<results_dir>/figures`` (the prototype/figures
    precedent). This is what the analysis driver / CLI calls after a sweep.
    """
    out_dir = out_dir or os.path.join(results_dir, 'figures')
    res = agg_mod.aggregate(results_dir)
    return plot_all(res['summary'], out_dir)
