# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Per-cell comparison of a v2 trial file with the committed paper-v1 rows.

Reads a v2 ``trials.csv`` (``runner --variant v2``), takes the committed
``results_2d/trials.csv`` rows for exactly the same (family, seed, config)
triples, and prints success / collision / timeout counts per family and config
for both. Counts only: no test, no interval (a smoke subset is not a result).

    python3 -m experiments.benchmark2d.compare_v1_v2 \
        experiments/results_2d_v2_smoke/trials.csv > .../SMOKE_TABLE.md
"""

from __future__ import annotations

import argparse
import csv
import os
from collections import defaultdict

from .configs import ORDER

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_V1 = os.path.join(_ROOT, 'experiments', 'results_2d', 'trials.csv')


def _load(path):
    with open(path, newline='') as f:
        return {(r['family'], int(r['seed']), r['config']): r
                for r in csv.DictReader(f)}


def compare(v2_path, v1_path=_V1):
    v2 = _load(v2_path)
    v1 = _load(v1_path)
    cells = defaultdict(lambda: {'n': 0, 'v1': defaultdict(int),
                                 'v2': defaultdict(int), 'v2x': defaultdict(float)})
    for (fam, seed, cfg), r in v2.items():
        c = cells[(fam, cfg)]
        c['n'] += 1
        c['v2'][r['outcome']] += 1
        c['v1'][v1[(fam, seed, cfg)]['outcome']] += 1
        for k in ('v2_zero_cmd_steps', 'v2_hops', 'v2_adoptions'):
            c['v2x'][k] += float(r.get(k, 0) or 0)
    return cells


def table(cells):
    fams = sorted({f for f, _ in cells})
    out = ['| family | config | n | v1 S/C/T | v2 S/C/T | v2 zero-cmd steps | v2 hops |',
           '|---|---|---|---|---|---|---|']
    for fam in fams:
        for cfg in ORDER:
            if (fam, cfg) not in cells:
                continue
            c = cells[(fam, cfg)]

            def sct(d):
                return f"{d['SUCCESS']}/{d['COLLISION']}/{d['TIMEOUT']}"
            out.append(f"| {fam} | {cfg} | {c['n']} | {sct(c['v1'])} | "
                       f"{sct(c['v2'])} | {int(c['v2x']['v2_zero_cmd_steps'])} | "
                       f"{int(c['v2x']['v2_hops'])} |")
    tot = defaultdict(lambda: [0, 0, 0])
    for (fam, cfg), c in cells.items():
        tot[cfg][0] += c['n']
        tot[cfg][1] += c['v1']['SUCCESS']
        tot[cfg][2] += c['v2']['SUCCESS']
    out += ['', '| config | n | v1 success | v2 success |', '|---|---|---|---|']
    for cfg in ORDER:
        if cfg in tot:
            n, a, b = tot[cfg]
            out.append(f'| {cfg} | {n} | {a} | {b} |')
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('v2_csv')
    ap.add_argument('--v1-csv', default=_V1)
    args = ap.parse_args()
    print(table(compare(args.v2_csv, args.v1_csv)))


if __name__ == '__main__':
    main()
