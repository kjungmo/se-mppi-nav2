# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Turn the aggregated 2D-benchmark data into paper-ready markdown tables.

Reads ``results_2d/trials.csv``, runs ``aggregate`` (summary + statistics),
writes ``summary.csv`` / ``stats.json`` and a ``tables.md`` fragment of
copy-paste-ready markdown tables (per-family config × metric with mean ± 95% CI
and N, plus the F-vs-baseline statistics with McNemar / Mann–Whitney / Cliff's δ
and Holm-adjusted p-values). It also prints a compact factual digest to stdout.

This exists so the numbers in ``docs/papers/2026_2d-benchmark-results.md`` are
generated from the measured CSV, never transcribed by hand. Interpretation prose
is written separately (by a human) after reading the digest.
"""

from __future__ import annotations

import argparse
import os

from .aggregate import aggregate
from .configs import ORDER, KEY_CONTRAST, BASELINES_VS_F

_CFG_LABEL = {
    'A_stock': 'A · stock MPPI',
    'C_escape': 'C · escape only',
    'D_cbf': 'D · CBF only',
    'E_indep': 'E · escape+CBF (indep.)',
    'F_full': 'F · SE-MPPI (coord.)',
    'Fminus_nogap': 'F⁻ · F, gap off',
}
_FAM_TITLE = {'utrap': 'U-trap (static local minima)',
              'clutter': 'Clutter (random static field)',
              'dynamic': 'Dynamic (crossing movers)',
              'narrowdyn': 'Narrow-dynamic (U-trap + movers on the escape route)'}


def _pm(d, unit=''):
    if d is None or d.get('mean') is None:
        return '—'
    ci = d.get('ci')
    if ci is None:
        return f'{d["mean"]:.2f}{unit}'
    return f'{d["mean"]:.2f} ± {ci:.2f}{unit}'


def _pct(x):
    return f'{100.0 * x:.0f}%'


def summary_table_md(summary, family):
    rows = {e['config']: e for e in summary if e['family'] == family}
    cfgs = [c for c in ORDER if c in rows]
    n = rows[cfgs[0]]['n_trials'] if cfgs else 0
    lines = [
        f'**{_FAM_TITLE.get(family, family)}** — N = {n} scenarios per config '
        '(paired; identical scenarios and MPPI noise across configs).',
        '',
        '| Config | Success (95% CI) | Collision | Timeout | '
        'Time-to-goal (s) | Path length (m) | Min clearance (m) |',
        '|---|---|---|---|---|---|---|',
    ]
    for c in cfgs:
        e = rows[c]
        succ = (f'{_pct(e["success_rate"])} '
                f'[{_pct(e["success_ci_low"])}, {_pct(e["success_ci_high"])}]')
        lines.append(
            f'| {_CFG_LABEL.get(c, c)} | {succ} | {_pct(e["collision_rate"])} | '
            f'{_pct(e["timeout_rate"])} | {_pm(e["time_to_goal"])} | '
            f'{_pm(e["path_length"])} | {_pm(e["min_clearance"])} |')
    lines.append('')
    lines.append('*(Time-to-goal and path length are over successful trials '
                 'only; min clearance over all trials.)*')
    return '\n'.join(lines)


def stats_table_md(stats, family):
    fam = stats.get(family, {})
    comps = {c['baseline']: c for c in fam.get('comparisons', [])}
    lines = [
        f'F vs baselines — {_FAM_TITLE.get(family, family)} '
        f'(Holm-corrected within this family; family size '
        f'{fam.get("holm_family_size", 0)}):',
        '',
        '| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | '
        'Path δ (p adj) | Clearance δ (p adj) |',
        '|---|---|---|---|---|---|',
    ]
    for base in BASELINES_VS_F:
        if base not in comps:
            continue
        c = comps[base]
        star = ' \\*' if c.get('mcnemar_reject') else ''
        mc = f'{c["mcnemar_p"]:.3g} ({_adj(c.get("mcnemar_p_adj"))}){star}'

        def cell(m):
            d = c['continuous'][m]
            s = ' \\*' if d.get('reject') else ''
            return f'{d["cliffs_delta"]:+.2f} ({_adj(d.get("p_adj"))}){s}'

        label = 'E→F (**key**)' if base == KEY_CONTRAST[0] else f'{base}→F'
        lines.append(
            f'| {label} | {c["mcnemar_b"]}/{c["mcnemar_c"]} | {mc} | '
            f'{cell("time_to_goal")} | {cell("path_length")} | '
            f'{cell("min_clearance")} |')
    lines.append('')
    lines.append('*(McNemar b = baseline-success & F-fail, c = baseline-fail & '
                 'F-success; Cliff\'s δ is F relative to the baseline, sign as '
                 'listed; \\* = Holm-adjusted p < 0.05.)*')
    return '\n'.join(lines)


def _adj(p):
    return '—' if p is None else f'{p:.3g}'


def slack_table_md(stats, family):
    """CBF slack-usage disclosure per config (from per-trial ``slack_max``)."""
    usage = stats.get(family, {}).get('slack_usage', {})
    lines = [
        f'CBF slack usage — {_FAM_TITLE.get(family, family)} '
        '(per-trial `slack_max`; a trial counts as "slack > 0" if the QP ever '
        'relaxed a barrier row):',
        '',
        '| Config | Trials w/ slack > 0 | Mean slack_max | Max slack_max |',
        '|---|---|---|---|',
    ]
    for c in ORDER:
        if c not in usage:
            continue
        u = usage[c]
        lines.append(
            f'| {_CFG_LABEL.get(c, c)} | {u["n_slack_pos"]}/{u["n"]} '
            f'({100.0 * u["frac_slack_pos"]:.0f}%) | '
            f'{u["slack_max_mean"]:.4f} | {u["slack_max_max"]:.4f} |')
    lines.append('')
    lines.append('*(Configs without a CBF (A, C) are trivially zero; on '
                 'families without movers the CBF sees no obstacles and is '
                 'likewise zero.)*')
    return '\n'.join(lines)


def digest(summary, stats):
    """Compact factual stdout digest (for the human writing the prose)."""
    out = []
    for fam in sorted({e['family'] for e in summary}):
        rows = {e['config']: e for e in summary if e['family'] == fam}
        out.append(f'=== {fam} ===')
        for c in ORDER:
            if c not in rows:
                continue
            e = rows[c]
            out.append(f'  {c:14s} succ={_pct(e["success_rate"]):>4} '
                       f'coll={_pct(e["collision_rate"]):>4} '
                       f'tout={_pct(e["timeout_rate"]):>4} '
                       f'time={_pm(e["time_to_goal"])} '
                       f'clr={_pm(e["min_clearance"])}')
        for comp in stats.get(fam, {}).get('comparisons', []):
            out.append(f'  F vs {comp["baseline"]:9s}: '
                       f'McNemar b={comp["mcnemar_b"]} c={comp["mcnemar_c"]} '
                       f'p={comp["mcnemar_p"]:.3g} adj={_adj(comp.get("mcnemar_p_adj"))} '
                       f'reject={comp.get("mcnemar_reject")}')
        for c, u in stats.get(fam, {}).get('slack_usage', {}).items():
            if u['n_slack_pos']:
                out.append(f'  slack>0 {c:14s}: {u["n_slack_pos"]}/{u["n"]} '
                           f'mean={u["slack_max_mean"]:.4f} '
                           f'max={u["slack_max_max"]:.4f}')
    return '\n'.join(out)


def build(trials_csv, out_dir):
    res = aggregate(trials_csv, out_dir)
    summary, stats = res['summary'], res['stats']
    frags = ['<!-- AUTO-GENERATED tables (experiments/benchmark2d/report.py) -->']
    for fam in sorted({e['family'] for e in summary}):
        frags.append('\n### ' + _FAM_TITLE.get(fam, fam) + '\n')
        frags.append(summary_table_md(summary, fam))
        frags.append('\n')
        frags.append(stats_table_md(stats, fam))
        frags.append('\n')
        frags.append(slack_table_md(stats, fam))
    tables_md = '\n'.join(frags)
    with open(os.path.join(out_dir, 'tables.md'), 'w') as f:
        f.write(tables_md)
    return res, tables_md


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ap.add_argument('--trials', default=os.path.join(
        here, 'experiments', 'results_2d', 'trials.csv'))
    ap.add_argument('--out-dir', default=os.path.join(
        here, 'experiments', 'results_2d'))
    args = ap.parse_args()
    res, _ = build(args.trials, args.out_dir)
    print(digest(res['summary'], res['stats']))


if __name__ == '__main__':
    main()
