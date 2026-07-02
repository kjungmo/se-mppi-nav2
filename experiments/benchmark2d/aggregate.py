# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Aggregate the 2D benchmark trials CSV into summary + statistics tables.

Consumes ``results_2d/trials.csv`` (written by ``runner``) and produces:

  * a per-(family, config) **summary**: N, success rate + Wilson 95% CI,
    collision rate, timeout rate, and mean ± 95% CI (normal approx) for
    time-to-goal, path length, and minimum clearance;
  * the **statistics**: for each family, F_full vs every baseline
    (A_stock, C_escape, D_cbf, E_indep) — McNemar on paired success (the
    primary test), Mann–Whitney U + Cliff's δ on the continuous metrics, with
    Holm–Bonferroni correction applied within each family's test family. The
    load-bearing comparison is **E_indep vs F_full** (isolating coordination).

Reuses the already-unit-tested statistics in ``experiments.analysis.stats`` and
the Wilson interval in ``experiments.analysis.aggregate``; nothing statistical
is re-implemented here. Continuous-metric conventions:

  * time-to-goal, path length — compared over each config's **successful** trials
    (a timeout's capped time / truncated path is not a travel cost);
  * min clearance — compared over **all** trials (a safety metric that is defined,
    and negative on collision, regardless of outcome).
"""

from __future__ import annotations

import csv
import json
import math
import os

from experiments.analysis import stats as S
from experiments.analysis.aggregate import wilson_interval
from .configs import ORDER, BASELINES_VS_F, KEY_CONTRAST

_CONTINUOUS = ('time_to_goal', 'path_length', 'min_clearance')
# Metrics compared over the successful subset only.
_SUCCESS_ONLY = ('time_to_goal', 'path_length')
_INT = {'seed', 'success', 'collided', 'reached', 'steps', 'cbf_active_steps',
        'n_obstacles', 'n_dynamic', 'feasible'}
_FLOAT = {'time_to_goal', 'path_length', 'min_clearance', 'alpha_max',
          'alpha_escape_frac', 'slack_max', 'slack_mean', 'entrapped_frac'}


def load_trials(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            row = dict(r)
            for k in list(row):
                if k in _INT and row[k] != '':
                    row[k] = int(row[k])
                elif k in _FLOAT and row[k] != '':
                    row[k] = float(row[k])
            rows.append(row)
    return rows


def _mean_ci(vals, z=1.96):
    xs = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    n = len(xs)
    if n == 0:
        return {'mean': None, 'ci': None, 'n': 0, 'std': None}
    mean = sum(xs) / n
    if n == 1:
        return {'mean': mean, 'ci': 0.0, 'n': 1, 'std': 0.0}
    var = sum((x - mean) ** 2 for x in xs) / (n - 1)
    std = math.sqrt(var)
    return {'mean': mean, 'ci': z * std / math.sqrt(n), 'n': n, 'std': std}


def _by(rows, family, config):
    return [r for r in rows if r['family'] == family and r['config'] == config]


def families_in(rows):
    return sorted({r['family'] for r in rows})


def configs_in(rows):
    present = {r['config'] for r in rows}
    return [c for c in ORDER if c in present]


def summarize(rows):
    """Per-(family, config) summary rows."""
    out = []
    for family in families_in(rows):
        for config in configs_in(rows):
            recs = _by(rows, family, config)
            if not recs:
                continue
            n = len(recs)
            succ = sum(r['success'] for r in recs)
            coll = sum(r['collided'] for r in recs)
            tout = sum(1 for r in recs if r['outcome'] == 'TIMEOUT')
            p, lo, hi = wilson_interval(succ, n)
            entry = {
                'family': family, 'config': config, 'n_trials': n,
                'n_success': succ,
                'success_rate': p, 'success_ci_low': lo, 'success_ci_high': hi,
                'collision_rate': coll / n, 'timeout_rate': tout / n,
            }
            succ_recs = [r for r in recs if r['success']]
            for m in _CONTINUOUS:
                src = succ_recs if m in _SUCCESS_ONLY else recs
                entry[m] = _mean_ci([r[m] for r in src])
            out.append(entry)
    return out


def _continuous_samples(recs_a, recs_b, metric):
    if metric in _SUCCESS_ONLY:
        a = [r[metric] for r in recs_a if r['success']]
        b = [r[metric] for r in recs_b if r['success']]
    else:
        a = [r[metric] for r in recs_a]
        b = [r[metric] for r in recs_b]
    return a, b


def slack_usage(rows):
    """Per-(family, config) CBF slack usage from the per-trial ``slack_max``.

    For each cell: N, the number/fraction of trials in which the QP ever used
    slack (``slack_max > 0``, i.e. the barrier row had to be relaxed at least
    once during the trial), and the mean / max of ``slack_max`` across trials.
    This is the empirical barrier-engagement disclosure the paper commits to
    (Sec. VI-C): configs without a CBF are trivially zero.
    """
    out = {}
    for family in families_in(rows):
        fam = {}
        for config in configs_in(rows):
            recs = _by(rows, family, config)
            if not recs:
                continue
            vals = [r['slack_max'] for r in recs]
            n = len(vals)
            n_pos = sum(1 for v in vals if v > 0.0)
            fam[config] = {
                'n': n,
                'n_slack_pos': n_pos,
                'frac_slack_pos': n_pos / n,
                'slack_max_mean': sum(vals) / n,
                'slack_max_max': max(vals),
            }
        out[family] = fam
    return out


def compute_stats(rows):
    """F_full vs each baseline, per family, with Holm correction within family.

    Returns ``{family: {'comparisons': [...], 'holm': {...}}}`` where each
    comparison holds the McNemar result (paired success) and, per continuous
    metric, the Mann–Whitney p-value and Cliff's δ.
    """
    result = {}
    f_cfg = KEY_CONTRAST[1]
    for family in families_in(rows):
        f_recs = _by(rows, family, f_cfg)
        f_by_seed = {r['seed']: r for r in f_recs}
        comparisons = []
        pvals = {}  # (baseline, test) -> p  for Holm within this family
        for base in BASELINES_VS_F:
            if base == f_cfg:
                continue
            b_recs = _by(rows, family, base)
            b_by_seed = {r['seed']: r for r in b_recs}
            seeds = sorted(set(f_by_seed) & set(b_by_seed))
            # Paired success (McNemar): b = base-success & F-fail; c = base-fail & F-success.
            base_succ = [bool(b_by_seed[s]['success']) for s in seeds]
            f_succ = [bool(f_by_seed[s]['success']) for s in seeds]
            b_disc, c_disc = S.paired_success_counts(base_succ, f_succ)
            mc = S.mcnemar(b_disc, c_disc)
            pvals[(base, 'success')] = mc.p_value
            comp = {
                'baseline': base, 'n_pairs': len(seeds),
                'base_success_rate': sum(base_succ) / len(seeds) if seeds else 0.0,
                'f_success_rate': sum(f_succ) / len(seeds) if seeds else 0.0,
                'mcnemar_b': b_disc, 'mcnemar_c': c_disc,
                'mcnemar_p': mc.p_value, 'mcnemar_method': mc.detail.get('method'),
                'continuous': {},
            }
            for m in _CONTINUOUS:
                xs, ys = _continuous_samples(b_recs, f_recs, m)
                mw = S.mann_whitney_u(xs, ys)
                delta = S.cliffs_delta(ys, xs)  # δ of F relative to baseline
                comp['continuous'][m] = {
                    'p': mw.p_value, 'cliffs_delta': delta,
                    'n_base': len(xs), 'n_f': len(ys),
                    'base_mean': (sum(xs) / len(xs)) if xs else None,
                    'f_mean': (sum(ys) / len(ys)) if ys else None,
                }
                pvals[(base, m)] = mw.p_value
            comparisons.append(comp)
        holm = S.holm(pvals) if pvals else {}
        # attach adjusted p-values back onto each comparison
        for comp in comparisons:
            key = (comp['baseline'], 'success')
            comp['mcnemar_p_adj'] = holm.get(key, {}).get('p_adj')
            comp['mcnemar_reject'] = holm.get(key, {}).get('reject')
            for m in _CONTINUOUS:
                hk = holm.get((comp['baseline'], m), {})
                comp['continuous'][m]['p_adj'] = hk.get('p_adj')
                comp['continuous'][m]['reject'] = hk.get('reject')
        result[family] = {'comparisons': comparisons,
                          'holm_family_size': len(pvals)}
    return result


def write_summary_csv(summary, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = ['family', 'config', 'n_trials', 'n_success', 'success_rate',
              'success_ci_low', 'success_ci_high', 'collision_rate',
              'timeout_rate']
    for m in _CONTINUOUS:
        fields += [f'{m}_mean', f'{m}_ci', f'{m}_n']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(fields)
        for e in summary:
            row = [e['family'], e['config'], e['n_trials'], e['n_success'],
                   round(e['success_rate'], 4), round(e['success_ci_low'], 4),
                   round(e['success_ci_high'], 4), round(e['collision_rate'], 4),
                   round(e['timeout_rate'], 4)]
            for m in _CONTINUOUS:
                d = e[m]
                row += [None if d['mean'] is None else round(d['mean'], 4),
                        None if d['ci'] is None else round(d['ci'], 4), d['n']]
            w.writerow(row)


def write_stats_json(stats, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(stats, f, indent=2)


def aggregate(trials_csv, out_dir):
    rows = load_trials(trials_csv)
    summary = summarize(rows)
    stats = compute_stats(rows)
    usage = slack_usage(rows)
    for family, fam_usage in usage.items():
        stats.setdefault(family, {})['slack_usage'] = fam_usage
    write_summary_csv(summary, os.path.join(out_dir, 'summary.csv'))
    write_stats_json(stats, os.path.join(out_dir, 'stats.json'))
    return {'rows': rows, 'summary': summary, 'stats': stats}
