# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for ablation config resolution (protocol §3 → param overlays)."""

import os

import pytest

from experiments.runner import config as cfg

ABLATIONS = os.path.join(os.path.dirname(__file__), '..', '..',
                         'configs', 'ablations.yaml')
FP = 'controller_server.ros__parameters.FollowPath'


def test_set_and_get_by_path():
    tree = {}
    cfg.set_by_path(tree, 'a.b.c', 7)
    assert cfg.get_by_path(tree, 'a.b.c') == 7
    assert tree == {'a': {'b': {'c': 7}}}
    with pytest.raises(KeyError):
        cfg.get_by_path(tree, 'a.b.x')


def _base_tree():
    return {
        'controller_server': {'ros__parameters': {'FollowPath': {
            'se_enabled': True,
            'se_alpha_base': 2.0,
            'se_alpha_escape': 6.0,
            'se_obstacle_max_speed': 2.0,
            'critics': ['CostCritic', 'GoalCritic', 'EscapeCritic'],
            'EscapeCritic': {'always_on': False, 'use_apf': True,
                             'use_gap_search': True},
        }}}
    }


def test_load_ablations_real_file():
    suite = cfg.load_ablations(ABLATIONS)
    assert suite.fp_prefix == FP
    # All nine matrix cells present.
    for name in ('A_stock', 'B_escape_alwayson', 'C_escape_detect', 'D_cbf_only',
                 'E_escape_cbf_indep', 'F_se_full', 'F_minus_gap', 'F_proxy',
                 'F_static'):
        assert name in suite.specs


def test_stock_drops_escape_and_disables_se():
    suite = cfg.load_ablations(ABLATIONS)
    out = cfg.resolve_params(suite, 'A_stock', base_tree=_base_tree())
    fp = cfg.get_by_path(out, FP)
    assert fp['se_enabled'] is False
    assert 'EscapeCritic' not in fp['critics']


def test_independent_has_equal_alphas_full_does_not():
    suite = cfg.load_ablations(ABLATIONS)
    e = cfg.get_by_path(cfg.resolve_params(suite, 'E_escape_cbf_indep',
                                           base_tree=_base_tree()), FP)
    f = cfg.get_by_path(cfg.resolve_params(suite, 'F_se_full',
                                           base_tree=_base_tree()), FP)
    # The E-vs-F contrast IS the coordination ablation: equal vs modulated alpha.
    assert e['se_alpha_escape'] == e['se_alpha_base']
    assert f['se_alpha_escape'] != f['se_alpha_base']
    assert f['se_enabled'] is True


def test_inherit_flattening_static_variant():
    suite = cfg.load_ablations(ABLATIONS)
    out = cfg.get_by_path(cfg.resolve_params(suite, 'F_static',
                                             base_tree=_base_tree()), FP)
    # F_static inherits F_se_full (coordination on) then zeroes prediction speed.
    assert out['se_obstacle_max_speed'] == 0.0
    assert out['se_alpha_escape'] == 6.0  # inherited from F
    assert out['se_enabled'] is True


def test_resolve_does_not_mutate_base():
    suite = cfg.load_ablations(ABLATIONS)
    base = _base_tree()
    cfg.resolve_params(suite, 'A_stock', base_tree=base)
    # Original critics list untouched (deep copy).
    assert 'EscapeCritic' in base['controller_server']['ros__parameters'][
        'FollowPath']['critics']
