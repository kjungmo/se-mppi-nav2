# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Single-flag diagnostic for the v2 mirror (smoke only, not a result).

Runs the ``C_escape`` config on the first ``-n`` seeds of each trap family for
every combination of the three escape flags (progress source, opening rule,
waypoint hopping), with the zero-command mode on and every other v2 value at
its default. Purpose: show which switch changes the outcome, so the lead can see
what the flag set does before the full re-run. Counts only.

    python3 -m experiments.benchmark2d.v2_flag_ablation -n 10 --workers 4
"""

from __future__ import annotations

import argparse
import itertools
from multiprocessing import Pool

from . import scenarios as sc
from .configs import make_v2_configs
from .rollout import rollout


def _run(task):
    family, seed, flags, max_steps = task
    cfg = make_v2_configs(**flags)['C_escape']
    scn = sc.make(family, seed)
    r = rollout(scn.world, cfg, scn.start, goal_tol=0.25, max_steps=max_steps,
                seed=seed)
    return family, tuple(sorted(flags.items())), r['outcome'], r['v2_hops']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', type=int, default=10)
    ap.add_argument('--families', nargs='+', default=['utrap', 'narrowdyn'])
    ap.add_argument('--max-steps', type=int, default=400)
    ap.add_argument('--workers', type=int, default=4)
    args = ap.parse_args()
    combos = [dict(progress=p, gap_rule=g, hop=h)
              for p, g, h in itertools.product(('goal_distance', 'anchor'),
                                               ('closest_viable', 'horizon'),
                                               (False, True))]
    tasks = [(fam, s, c, args.max_steps) for fam in args.families
             for c in combos for s in range(args.n)]
    with Pool(args.workers) as pool:
        res = pool.map(_run, tasks, chunksize=1)
    print('| progress | gap_rule | hop | ' +
          ' | '.join(f'{f} S/C/T' for f in args.families) + ' |')
    print('|---|---|---|' + '---|' * len(args.families))
    for c in combos:
        key = tuple(sorted(c.items()))
        cells = []
        for fam in args.families:
            o = [r[2] for r in res if r[0] == fam and r[1] == key]
            cells.append(f"{o.count('SUCCESS')}/{o.count('COLLISION')}/"
                         f"{o.count('TIMEOUT')}")
        print(f"| {c['progress']} | {c['gap_rule']} | {c['hop']} | "
              + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
