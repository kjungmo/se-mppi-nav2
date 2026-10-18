# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Batch runner for the randomized 2D benchmark (paper §VI-C).

Paired design: for each ``(family, seed)`` scenario the world geometry is
generated *once per config* from the same seed (byte-identical geometry) and the
MPPI noise stream uses that same seed, so every config sees the same scenario and
the same random rollouts — the common-random-numbers setup McNemar requires.

Writes one long-format CSV: ``results_2d/trials.csv`` (one row per trial). The
aggregator and figure scripts consume that CSV; nothing here fabricates numbers.
"""

from __future__ import annotations

import argparse
import csv
import os
import time
from multiprocessing import Pool

from . import scenarios as sc
from .configs import CONFIGS, ORDER, VARIANTS  # noqa: F401
from .rollout import rollout

_CSV_FIELDS = [
    'family', 'seed', 'config', 'outcome', 'success', 'collided', 'reached',
    'time_to_goal', 'path_length', 'min_clearance', 'steps',
    'alpha_max', 'alpha_escape_frac', 'slack_max', 'slack_mean',
    'cbf_active_steps', 'entrapped_frac',
    'n_obstacles', 'n_dynamic', 'feasible',
]
# Extra columns written only by the opt-in v2 variant (the v1 file is unchanged).
_V2_FIELDS = ['v2_zero_cmd_steps', 'v2_progress_events', 'v2_adoptions', 'v2_hops']
# Variant columns: both v2 variants write the v2 columns and a 'variant' column, so that the shipped
# arm ('v2') and the design as advised ('v2_advised') cannot be mixed up in one file.
_V2_VARIANTS = ('v2', 'v2_advised')


def variant_fields(variant):
    return _CSV_FIELDS + (_V2_FIELDS + ['variant'] if variant in _V2_VARIANTS else [])


def refuse_paper_file(variant, out, paper_v1):
    """Message when a non-v1 variant would write the committed paper-v1 file, else None."""
    if variant != 'v1' and os.path.abspath(out) == os.path.abspath(paper_v1):
        return (f'--variant {variant} must not write the committed paper-v1 file; pass --out')
    return None


def run_one_scenario(task):
    """Run all configs for one (family, seed) scenario. Returns list of rows."""
    family, seed, config_names, max_steps, *rest = task
    variant = rest[0] if rest else 'v1'
    configs = VARIANTS[variant]
    rows = []
    for name in config_names:
        scn = sc.make(family, seed)  # fresh world per config (movers mutate it)
        r = rollout(scn.world, configs[name], scn.start,
                    goal_tol=0.25, max_steps=max_steps, seed=seed)
        rows.append({
            'family': family, 'seed': seed, 'config': name,
            'outcome': r['outcome'], 'success': int(r['success']),
            'collided': int(r['collided']), 'reached': int(r['reached']),
            'time_to_goal': round(r['time_to_goal'], 4),
            'path_length': round(r['path_length'], 4),
            'min_clearance': round(r['min_clearance'], 4),
            'steps': r['steps'],
            'alpha_max': r['alpha_max'],
            'alpha_escape_frac': round(r['alpha_escape_frac'], 4),
            'slack_max': round(r['slack_max'], 6),
            'slack_mean': round(r['slack_mean'], 6),
            'cbf_active_steps': r['cbf_active_steps'],
            'entrapped_frac': round(r['entrapped_frac'], 4),
            'n_obstacles': scn.meta.get('n_obstacles'),
            'n_dynamic': scn.meta.get('n_dynamic', 0),
            'feasible': int(scn.meta.get('feasible', True)),
            **({**{k: r[k] for k in _V2_FIELDS}, 'variant': variant}
               if variant in _V2_VARIANTS else {}),
        })
    return rows


def _done_scenarios(out_csv, config_names):
    """(family, seed) pairs already fully covered (all configs) in ``out_csv``.

    Enables resume after an interruption (this host shares CPU/RAM with a live
    Gazebo stack, so a run may be killed mid-way — partial CSV is never lost and
    completed scenarios are skipped on the next invocation).
    """
    if not os.path.exists(out_csv):
        return set()
    seen = {}
    with open(out_csv, newline='') as f:
        for r in csv.DictReader(f):
            seen.setdefault((r['family'], int(r['seed'])), set()).add(r['config'])
    need = set(config_names)
    return {k for k, cfgs in seen.items() if need.issubset(cfgs)}


def run_benchmark(families, n, out_csv, *, config_names=None, max_steps=350,
                  workers=6, chunksize=1, progress=True, resume=True,
                  variant='v1'):
    """Run the full matrix, appending to ``out_csv`` incrementally.

    Rows are flushed after every completed scenario so an interrupted run keeps
    all finished work; re-invoking with ``resume=True`` skips scenarios already
    present. Returns the list of rows produced *this* invocation.
    """
    config_names = config_names or ORDER
    done = _done_scenarios(out_csv, config_names) if resume else set()
    # Interleave families by seed (seed0 all families, seed1 all families, ...)
    # so an early stop still leaves a balanced N across families.
    tasks = [(fam, seed, config_names, max_steps) +
             (() if variant == 'v1' else (variant,))
             for seed in range(n) for fam in families
             if (fam, seed) not in done]
    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
    new_file = not os.path.exists(out_csv) or os.path.getsize(out_csv) == 0
    if not new_file:
        with open(out_csv, newline='') as f:
            header = next(csv.reader(f), [])
        if header != variant_fields(variant):
            raise ValueError(f'{out_csv} has the columns of another variant; '
                             f'--variant {variant} needs its own file')
        if variant in _V2_VARIANTS:
            with open(out_csv, newline='') as f:
                other = {r['variant'] for r in csv.DictReader(f)} - {variant}
            if other:
                raise ValueError(f'{out_csv} holds rows of variant {sorted(other)}; '
                                 f'--variant {variant} needs its own file')

    t0 = time.perf_counter()
    produced = []
    if progress and done:
        print(f'resume: {len(done)} scenarios already done, '
              f'{len(tasks)} remaining', flush=True)

    with open(out_csv, 'a', newline='') as fout:
        writer = csv.DictWriter(fout, fieldnames=variant_fields(variant))
        if new_file:
            writer.writeheader()
            fout.flush()

        def emit(rows):
            for row in rows:
                writer.writerow(row)
            fout.flush()
            produced.extend(rows)

        if workers <= 1:
            for i, task in enumerate(tasks):
                emit(run_one_scenario(task))
                if progress:
                    print(f'[{i + 1}/{len(tasks)}] {task[0]} seed={task[1]} '
                          f'({time.perf_counter() - t0:.0f}s)', flush=True)
        else:
            with Pool(processes=workers) as pool:
                for i, rows in enumerate(
                        pool.imap_unordered(run_one_scenario, tasks, chunksize)):
                    emit(rows)
                    if progress:
                        print(f'[{i + 1}/{len(tasks)}] scenarios done '
                              f'({time.perf_counter() - t0:.0f}s)', flush=True)

    elapsed = time.perf_counter() - t0
    if progress:
        print(f'\n{len(produced)} new trials in {elapsed:.0f}s '
              f'({elapsed / max(1, len(produced)):.2f}s/trial) -> {out_csv}',
              flush=True)
    return produced


def write_csv(rows, path):
    """Overwrite ``path`` with ``rows`` (used by tests; the runner appends)."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main():
    ap = argparse.ArgumentParser(description='SE-MPPI randomized 2D benchmark')
    ap.add_argument('--families', nargs='+',
                    default=['utrap', 'clutter', 'dynamic'])
    ap.add_argument('-n', '--n', type=int, default=100,
                    help='scenarios per family')
    ap.add_argument('--max-steps', type=int, default=300)
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--no-resume', action='store_true')
    ap.add_argument('--variant', choices=sorted(VARIANTS), default='v1',
                    help="v1 = paper results (default); v2 = SE-MPPI v2 escape "
                         "logic as shipped; v2_advised = v2 without the four "
                         "additions (the design as advised). Write v2 and "
                         "v2_advised to separate --out files")
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'experiments', 'results_2d', 'trials.csv'))
    args = ap.parse_args()
    paper_v1 = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), 'experiments', 'results_2d', 'trials.csv')
    msg = refuse_paper_file(args.variant, args.out, paper_v1)
    if msg:
        ap.error(msg)
    run_benchmark(args.families, args.n, args.out,
                  max_steps=args.max_steps, workers=args.workers,
                  resume=not args.no_resume, variant=args.variant)


if __name__ == '__main__':
    main()
