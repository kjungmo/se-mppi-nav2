# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""The ``v2_advised`` variant: the design as advised, without the four additions.

LEAD_DECISIONS_2 E-3. ``v2`` is the shipped algorithm (``escape_extras`` true), ``v2_advised`` is the
same code with ``escape_extras`` false. The two arms must differ in that flag only, write their own
columns, and neither may write the committed paper-v1 file.
"""

import csv
import os

import pytest

from experiments.benchmark2d import configs, runner

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
_PAPER_V1 = os.path.join(_ROOT, 'experiments', 'results_2d', 'trials.csv')


def test_variant_is_registered_and_differs_from_v2_in_the_extras_flag_only():
    assert set(configs.VARIANTS) == {'v1', 'v2', 'v2_advised'}
    for name in configs.ORDER:
        a, b = configs.VARIANTS['v2'][name], configs.VARIANTS['v2_advised'][name]
        assert a['v2']['escape_extras'] is True
        assert b['v2']['escape_extras'] is False
        assert {**a['v2'], 'escape_extras': False} == b['v2']
        assert {k: v for k, v in a.items() if k != 'v2'} == {k: v for k, v in b.items() if k != 'v2'}


def test_paper_v1_configs_do_not_carry_the_v2_dict():
    for name in configs.ORDER:
        assert 'v2' not in configs.VARIANTS['v1'][name]


@pytest.mark.parametrize('variant', ['v2', 'v2_advised'])
def test_refuses_to_write_the_committed_paper_v1_file(variant):
    msg = runner.refuse_paper_file(variant, _PAPER_V1, _PAPER_V1)
    assert msg and variant in msg
    assert runner.refuse_paper_file(variant, _PAPER_V1 + '.x', _PAPER_V1) is None
    assert runner.refuse_paper_file('v1', _PAPER_V1, _PAPER_V1) is None   # v1 owns that file


def test_own_columns():
    v1 = runner.variant_fields('v1')
    for variant in ('v2', 'v2_advised'):
        f = runner.variant_fields(variant)
        assert f[:len(v1)] == v1
        assert f[len(v1):] == runner._V2_FIELDS + ['variant']
    assert 'variant' not in v1


@pytest.mark.parametrize('variant', ['v2', 'v2_advised'])
def test_rows_carry_the_variant_and_its_columns(variant):
    r = runner.run_one_scenario(('utrap', 0, ['C_escape'], 120, variant))
    assert len(r) == 1 and r[0]['variant'] == variant
    assert set(r[0]) == set(runner.variant_fields(variant))


def test_one_file_never_holds_two_arms(tmp_path):
    out = str(tmp_path / 'trials.csv')
    runner.run_benchmark(['utrap'], 1, out, config_names=['A_stock'], max_steps=40,
                         workers=1, progress=False, variant='v2')
    with open(out, newline='') as f:
        assert {r['variant'] for r in csv.DictReader(f)} == {'v2'}
    with pytest.raises(ValueError):
        runner.run_benchmark(['utrap'], 2, out, config_names=['A_stock'], max_steps=40,
                             workers=1, progress=False, variant='v2_advised')
    with pytest.raises(ValueError):                  # the v1 header differs as well
        runner.run_benchmark(['utrap'], 2, out, config_names=['A_stock'], max_steps=40,
                             workers=1, progress=False, variant='v1')
