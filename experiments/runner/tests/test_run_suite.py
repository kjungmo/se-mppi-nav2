# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for the suite orchestrator (design §3): matrix loop, isolation, resume."""

import json
import os

from experiments.runner import config as cfg
from experiments.runner import run_suite as rs
from experiments.runner.scenario import Scenario
from experiments.runner.trial import DriveResult, FakeLauncher

ABLATIONS = os.path.join(os.path.dirname(__file__), '..', '..',
                         'configs', 'ablations.yaml')


def _base_tree():
    return {'controller_server': {'ros__parameters': {'FollowPath': {
        'se_enabled': True, 'se_alpha_base': 2.0, 'se_alpha_escape': 6.0,
        'se_obstacle_max_speed': 2.0,
        'critics': ['CostCritic', 'EscapeCritic'],
        'EscapeCritic': {'always_on': False, 'use_apf': True,
                         'use_gap_search': True}}}}}


def _scenarios():
    return [Scenario(name='s1', tier='barn', map_yaml='x', start=(0, 0, 0),
                     goal=(5, 0, 0), optimal_length=5.0, optimal_time=5.0)]


def _reach():
    return DriveResult(samples=[{'t': i * 0.5, 'x': i * 0.5, 'y': 0.0,
                                 'v': 0.5, 'w': 0.0} for i in range(11)])


def _factory():
    return FakeLauncher(_reach())


def test_suite_runs_full_matrix(tmp_path):
    suite = cfg.load_ablations(ABLATIONS)
    cfgs = ['A_stock', 'F_se_full']
    report = rs.run_suite(
        _factory, suite, _scenarios(), seeds=[0, 1], results_dir=str(tmp_path),
        config_names=cfgs, base_tree=_base_tree(), validate_scenarios=False)
    assert report.n_total == 2 * 2          # 2 configs × 2 seeds × 1 scenario
    assert report.n_run == 4
    assert report.outcomes.get('SUCCESS') == 4
    # One JSON per trial, namespaced by tier/config.
    for c in cfgs:
        for seed in (0, 1):
            p = os.path.join(tmp_path, 'barn', c, f's1_seed{seed}.json')
            assert os.path.exists(p)
            with open(p) as f:
                assert json.load(f)['outcome'] == 'SUCCESS'


def test_resume_skips_completed(tmp_path):
    suite = cfg.load_ablations(ABLATIONS)
    cfgs = ['A_stock']
    args = dict(suite=suite, scenarios=_scenarios(), seeds=[0, 1],
                results_dir=str(tmp_path), config_names=cfgs,
                base_tree=_base_tree(), validate_scenarios=False)
    first = rs.run_suite(_factory, **args)
    assert first.n_run == 2
    # Second pass: everything already on disk -> all skipped, none re-run.
    second = rs.run_suite(_factory, **args)
    assert second.n_run == 0
    assert second.n_skipped == 2
    assert second.outcomes.get('SUCCESS') == 2


def test_validate_skips_unreachable_scenarios(tmp_path):
    suite = cfg.load_ablations(ABLATIONS)
    # map_yaml does not exist -> validate() fails -> scenario skipped entirely.
    report = rs.run_suite(
        _factory, suite, _scenarios(), seeds=[0], results_dir=str(tmp_path),
        config_names=['A_stock'], base_tree=_base_tree(),
        validate_scenarios=True)
    assert report.n_total == 0
    assert report.n_run == 0


SUITE_YAML = os.path.join(os.path.dirname(__file__), '..', '..',
                          'configs', 'suite.yaml')


def test_load_suite_parses_settings():
    s = rs.load_suite(SUITE_YAML)
    assert s.tiers == ['barn', 'dynabarn', 'hunav']
    assert len(s.seeds) == 20
    assert os.path.isabs(s.ablations_path)
    assert s.ablations_path.endswith('ablations.yaml')
    # results_dir + tier dirs resolve relative to suite.yaml's parent tree.
    assert os.path.isabs(s.results_dir)
    assert os.path.isdir(os.path.join(s.experiments_root, 'barn'))
    assert s.trial_cfg.robot_radius == 0.22 and s.trial_cfg.setup_retries == 1


def test_make_launcher_factory_fake_drives_to_goal():
    s = rs.load_suite(SUITE_YAML)
    factory = rs.make_launcher_factory('fake', s)
    launcher = factory()
    res = launcher.drive(None, _scenarios()[0], watchdog_s=10.0)
    assert res.samples[0]['x'] == 0.0
    assert abs(res.samples[-1]['x'] - 5.0) < 1e-9   # reaches goal x=5


def test_make_launcher_factory_rejects_unknown():
    s = rs.load_suite(SUITE_YAML)
    try:
        rs.make_launcher_factory('bogus', s)
        assert False, 'expected ValueError'
    except ValueError as e:
        assert 'fake|ros' in str(e)


def test_cli_main_runs_fake_matrix(tmp_path):
    # End-to-end turnkey path: barn tier only (the example_utrap fixture), two
    # seeds, fake launcher -> real result JSON written, exit 0.
    rc = rs.main(['--config', SUITE_YAML, '--launcher', 'fake',
                  '--results-dir', str(tmp_path), '--tiers', 'barn',
                  '--seeds', '0', '1'])
    assert rc == 0
    produced = list(tmp_path.glob('barn/*/example_utrap_seed*.json'))
    assert produced, 'CLI should have written per-trial result JSON'


def test_cli_scenario_filter_restricts_to_allowlist(tmp_path):
    # --scenarios keeps the live matrix to the real scenarios (S1/S2) and leaves
    # the offline example_utrap fixture out of a run.
    rc = rs.main(['--config', SUITE_YAML, '--launcher', 'fake',
                  '--results-dir', str(tmp_path), '--tiers', 'barn',
                  '--seeds', '0', '--scenarios', 's2_passage'])
    assert rc == 0
    assert list(tmp_path.glob('barn/*/s2_passage_seed*.json'))
    assert not list(tmp_path.glob('barn/*/example_utrap_seed*.json'))
