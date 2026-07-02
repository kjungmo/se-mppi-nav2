# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Aggregate raw trial JSON into the paper's summary tables (design §6).

Reads ``results/<tier>/<config>/<scenario>_seed*.json`` written by the runner,
flattens them to a long-format CSV (one row per trial), and groups by
(tier, config) into a summary: success rate with a Wilson confidence interval,
collision/timeout rates, and median ± IQR of the continuous metrics. Pure
stdlib + math so it runs anywhere and is unit-tested on synthetic records.
"""

from __future__ import annotations

import csv
import glob
import json
import math
import os
from collections import defaultdict

# Continuous metrics summarized as median ± IQR.
_CONTINUOUS = ('time_to_goal', 'path_length', 'min_clearance', 'spl',
               'barn_score', 'oscillation_ratio')


def load_results(results_dir: str) -> list:
    """Load every trial JSON under ``results_dir`` into a list of dicts."""
    rows = []
    for path in sorted(glob.glob(os.path.join(results_dir, '**', '*.json'),
                                 recursive=True)):
        with open(path) as f:
            rows.append(json.load(f))
    return rows


def flatten(rec: dict) -> dict:
    """One trial dict -> a flat row for CSV (metrics hoisted to top level)."""
    m = rec.get('metrics', {}) or {}
    comp = m.get('compute', {}) or {}
    smooth = m.get('smoothness', {}) or {}
    row = {
        'scenario': rec.get('scenario'),
        'tier': rec.get('tier'),
        'config': rec.get('config'),
        'seed': rec.get('seed'),
        'outcome': rec.get('outcome'),
        'success': bool(m.get('success')),
        'collided': bool(m.get('collided')),
        'reached_goal': bool(m.get('reached_goal')),
        'time_to_goal': m.get('time_to_goal'),
        'path_length': m.get('path_length'),
        'min_clearance': m.get('min_clearance'),
        'spl': m.get('spl'),
        'barn_score': m.get('barn_score'),
        'oscillation_ratio': m.get('oscillation_ratio'),
        'compute_mean_ms': comp.get('mean_ms'),
        'compute_p95_ms': comp.get('p95_ms'),
        'lin_jerk_mean': smooth.get('lin_jerk_mean'),
        'setup_attempts': rec.get('setup_attempts'),
    }
    return row


def write_long_csv(rows: list, path: str) -> None:
    """Write flattened trial rows to a long-format CSV."""
    flat = [flatten(r) for r in rows]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = list(flat[0].keys()) if flat else []
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in flat:
            w.writerow(r)


def wilson_interval(successes: int, n: int, z: float = 1.96):
    """Wilson score CI for a binomial proportion (better than normal at small n)."""
    if n == 0:
        return (0.0, 0.0, 0.0)
    p = successes / n
    denom = 1.0 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / denom
    return (p, max(0.0, center - half), min(1.0, center + half))


def _median_iqr(vals):
    xs = sorted(v for v in vals if v is not None and not _is_nan(v))
    if not xs:
        return {'median': None, 'q1': None, 'q3': None, 'n': 0}
    return {'median': _pct(xs, 50), 'q1': _pct(xs, 25), 'q3': _pct(xs, 75),
            'n': len(xs)}


def _is_nan(v):
    return isinstance(v, float) and math.isnan(v)


def _pct(sorted_vals, pct):
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    rank = (pct / 100.0) * (len(sorted_vals) - 1)
    lo, hi = int(math.floor(rank)), int(math.ceil(rank))
    if lo == hi:
        return sorted_vals[lo]
    frac = rank - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def summarize(rows: list) -> list:
    """Group trials by (tier, config); return summary rows (design table 1–3).

    Setup failures are reported separately (``n_setup_fail``) and excluded from
    the success-rate denominator, per design §7.
    """
    groups = defaultdict(list)
    for r in rows:
        groups[(r.get('tier'), r.get('config'))].append(r)

    out = []
    for (tier, config), recs in sorted(groups.items(),
                                       key=lambda kv: (str(kv[0][0]), str(kv[0][1]))):
        valid = [r for r in recs if r.get('outcome') != 'SETUP_FAIL']
        n = len(valid)
        n_setup_fail = len(recs) - n
        succ = sum(1 for r in valid if (r.get('metrics') or {}).get('success'))
        coll = sum(1 for r in valid if (r.get('metrics') or {}).get('collided'))
        tout = sum(1 for r in valid if r.get('outcome') == 'TIMEOUT')
        p, lo, hi = wilson_interval(succ, n)
        entry = {
            'tier': tier, 'config': config,
            'n_trials': n, 'n_setup_fail': n_setup_fail,
            'success_rate': p, 'success_ci_low': lo, 'success_ci_high': hi,
            'collision_rate': (coll / n) if n else 0.0,
            'timeout_rate': (tout / n) if n else 0.0,
        }
        for key in _CONTINUOUS:
            vals = [(r.get('metrics') or {}).get(key) for r in valid]
            entry[key] = _median_iqr(vals)
        out.append(entry)
    return out


def aggregate(results_dir: str, out_csv: str | None = None) -> dict:
    """End-to-end: load -> long CSV -> summary. Returns {'rows', 'summary'}."""
    rows = load_results(results_dir)
    if out_csv:
        write_long_csv(rows, out_csv)
    return {'rows': rows, 'summary': summarize(rows)}
