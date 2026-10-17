# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Diagnostic: how the v2 horizon rule depends on the sensing horizon (smoke only).

Runs ``C_escape`` with every v2 switch on, varying the ray range and the
half-size of the sensing window (both in metres), on the first ``-n`` seeds of
the trap families. The paper-v2 parameter set is range 2.0 / window 1.5. Counts
only; nothing here is a result.

    python3 -m experiments.benchmark2d.v2_horizon_sweep -n 10 --workers 4
"""

from __future__ import annotations

import argparse
from multiprocessing import Pool

from . import scenarios as sc
from .configs import make_v2_configs
from .rollout import rollout

SETTINGS = [(2.0, 1.5), (3.0, 3.0), (4.0, 4.0), (6.0, 6.0)]   # (range, window half)


def _run(task):
    family, seed, (rng, win), max_steps = task
    cfg = make_v2_configs(gap_max_range=rng, window_half=win)['C_escape']
    scn = sc.make(family, seed)
    r = rollout(scn.world, cfg, scn.start, goal_tol=0.25, max_steps=max_steps,
                seed=seed)
    return family, (rng, win), r['outcome']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', type=int, default=10)
    ap.add_argument('--families', nargs='+', default=['utrap', 'narrowdyn'])
    ap.add_argument('--max-steps', type=int, default=400)
    ap.add_argument('--workers', type=int, default=4)
    args = ap.parse_args()
    tasks = [(f, s, st, args.max_steps) for f in args.families
             for st in SETTINGS for s in range(args.n)]
    with Pool(args.workers) as pool:
        res = pool.map(_run, tasks, chunksize=1)
    print('| range m | window half m | ' +
          ' | '.join(f'{f} S/C/T' for f in args.families) + ' |')
    print('|---|---|' + '---|' * len(args.families))
    for st in SETTINGS:
        cells = []
        for fam in args.families:
            o = [r[2] for r in res if r[0] == fam and r[1] == st]
            cells.append(f"{o.count('SUCCESS')}/{o.count('COLLISION')}/"
                         f"{o.count('TIMEOUT')}")
        print(f'| {st[0]} | {st[1]} | ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
