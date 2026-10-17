# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""The default (paper-v1) mirror still reproduces the committed result rows.

The SE-MPPI v2 flags are opt-in. These trials re-run a few committed
``results_2d/trials.csv`` rows through the runner with the default variant and
compare every column as text, exactly as the CSV stores them. The full check
(every row of a subset) is the regeneration documented in the v2 hand-back.
"""

import csv
import os

import pytest

from experiments.benchmark2d import runner

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
_CSV = os.path.join(_ROOT, 'experiments', 'results_2d', 'trials.csv')

# fast, informative: an escape trap, a free-space clutter run, a dynamic run
_CASES = [('utrap', 4, ['A_stock', 'C_escape', 'F_full']),
          ('clutter', 1, ['C_escape', 'F_full']),
          ('dynamic', 0, ['D_cbf', 'F_full'])]


def _committed():
    with open(_CSV, newline='') as f:
        return {(r['family'], int(r['seed']), r['config']): r
                for r in csv.DictReader(f)}


@pytest.mark.parametrize('family,seed,configs', _CASES)
def test_default_variant_reproduces_committed_rows(family, seed, configs):
    old = _committed()
    rows = runner.run_one_scenario((family, seed, configs, 400))
    assert len(rows) == len(configs)
    for r in rows:
        ref = old[(r['family'], r['seed'], r['config'])]
        assert list(r.keys()) == list(ref.keys())            # no extra columns
        assert {k: str(v) for k, v in r.items()} == ref, (family, seed, r['config'])
