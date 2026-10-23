"""Tests of the campaign-2 plan, parameter file, trees and scenario files."""

import filecmp
import glob
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

import plan

REPO = plan.REPO


# --------------------------------------------------------------------------- #
# Plan
# --------------------------------------------------------------------------- #
def test_plan_has_88_trials_in_two_phases():
    allp = plan.plan('all')
    assert len(allp) == 88
    assert len(plan.plan('A')) == 9
    assert len(plan.plan('B')) == 79
    assert allp[:9] == plan.plan('A') and allp[9:] == plan.plan('B')
    assert len({t.trial_id for t in allp}) == 88          # no duplicate


def test_plan_counts_per_cell():
    counts = {}
    for t in plan.plan('all'):
        counts[t.cell] = counts.get(t.cell, 0) + 1
    for s in plan.SCENARIO_ORDER:
        for c in plan.configs_of(s):
            assert counts[(s, c)] == 6, (s, c)
    # s4_ltrap runs the two no-recovery configurations only (PREREG.md, amendment 1)
    assert plan.configs_of('s4_ltrap') == ('C3', 'C4')
    assert not any(k[0] == 's4_ltrap' and k[1] in ('C1', 'C2') for k in counts)
    assert counts[('s4_ltrap', 'C3')] == counts[('s4_ltrap', 'C4')] == 6
    assert counts[(plan.EVIDENCE_SCENARIO, 'C2')] == 2
    assert counts[(plan.EVIDENCE_SCENARIO, 'C4')] == 2
    assert sum(counts.values()) == 72 + 12 + 4
    assert not any(k[0] == plan.EVIDENCE_SCENARIO and k[1] in ('C1', 'C3') for k in counts)


def test_phase_a_is_c4_x6_and_c3_x3_on_s1():
    a = plan.plan('A')
    assert sorted((t.config, t.rep) for t in a) == sorted(
        [('C4', r) for r in range(6)] + [('C3', r) for r in range(3)])
    assert {t.scenario for t in a} == {'s1_utrap'}
    assert {t.phase for t in a} == {'A'}


def test_phase_b_excludes_phase_a_trials():
    a_ids = {t.trial_id for t in plan.plan('A')}
    b_ids = {t.trial_id for t in plan.plan('B')}
    assert not a_ids & b_ids
    assert 'correct__s1_utrap__C3__rep3' in b_ids and 'correct__s1_utrap__C3__rep2' not in b_ids
    assert not any('s1_utrap__C4' in i for i in b_ids)


def test_cell_order_rotates_with_the_repetition():
    b = plan.plan('B')
    firsts = []
    for rep in range(plan.REPS):
        trials = [t for t in b if t.rep == rep]
        firsts.append(trials[0].cell)
        # the order within a repetition is a rotation of the canonical order
        canon = [t.cell for t in trials]
        assert len(set(canon)) == len(canon)
    assert len(set(firsts)) == plan.REPS                  # a different cell first every repetition
    a = plan.plan('A')
    assert a[0].config == 'C4' and a[1].config == 'C3'    # rep 0
    assert a[2].config == 'C3' and a[3].config == 'C4'    # rep 1: rotated


def test_repetitions_run_in_order():
    for phase in ('A', 'B'):
        reps = [t.rep for t in plan.plan(phase)]
        assert reps == sorted(reps)


def test_shakedown_plan_is_one_trial_on_s0_open_not_counted():
    sd = plan.plan('shakedown')
    assert len(sd) == 1 and sd[0].scenario == 's0_open' and sd[0].phase == 'shakedown'
    assert not sd[0].counted
    assert all(t.counted for t in plan.plan('all') if t.scenario != plan.EVIDENCE_SCENARIO)
    assert not any(t.counted for t in plan.plan('all') if t.scenario == plan.EVIDENCE_SCENARIO)


def test_no_trial_runs_on_a_scenario_without_a_file():
    for t in plan.plan('all') + plan.plan('shakedown'):
        assert os.path.exists(os.path.join(REPO, plan.SCENARIO_FILES[t.scenario]))


def test_configurations_match_the_design_table():
    c = plan.CONFIGS
    assert [c[k]['ablation'] for k in plan.CONFIG_ORDER] == ['A_stock', 'F_se_full', 'A_stock', 'F_se_full']
    assert [c[k]['tree'] for k in plan.CONFIG_ORDER] == [
        plan.BT_DEFAULT, plan.BT_DEFAULT, plan.BT_REPLAN_ONLY, plan.BT_REPLAN_ONLY]
    assert [c[k]['allowance_s'] for k in plan.CONFIG_ORDER] == [10.0, 10.0, 30.0, 30.0]
    assert [c[k]['se'] for k in plan.CONFIG_ORDER] == [False, True, False, True]


# --------------------------------------------------------------------------- #
# Parameter file
# --------------------------------------------------------------------------- #
def _params():
    with open(plan.PARAMS_YAML) as f:
        return yaml.safe_load(f)


def _get(tree, dotted):
    for k in dotted.split('.'):
        tree = tree[k]
    return tree


def test_frozen_escape_parameters_of_design_advice_4_2():
    fp = _get(_params(), plan.FP)
    expect = {
        'vx_min': -0.35, 'se_enabled': True, 'se_progress_min_advance': 0.10,
        'se_progress_adopt_advance': 0.5, 'se_progress_stall_window': 30, 'se_gap_num_rays': 36,
        'se_gap_max_range': 2.0, 'se_escape_hysteresis': 0.5, 'se_escape_waypoint_cap': 2.5,
        'se_escape_reselect_radius': 0.4, 'se_escape_reselect_period': 15.0,
        'se_escape_path_substitution': True, 'time_steps': 28, 'model_dt': 0.1, 'batch_size': 1500}
    for k, v in expect.items():
        assert fp[k] == v, k
    assert fp['EscapeCritic']['gap_weight'] == 0.0
    assert fp['EscapeCritic']['use_apf'] is True            # APF kept
    assert _get(_params(), 'controller_server.ros__parameters.controller_frequency') == 10.0
    inflation = _get(_params(), 'local_costmap.local_costmap.ros__parameters.inflation_layer')
    assert inflation['inflation_radius'] == 0.45


def test_nav2_default_stall_handling():
    p = _params()
    cs = p['controller_server']['ros__parameters']
    assert cs['failure_tolerance'] == 0.0
    assert cs['progress_checker']['required_movement_radius'] == 0.5
    assert cs['progress_checker']['movement_time_allowance'] == 10.0


def test_shipped_file_differs_from_campaign_1_only_where_declared():
    with open(os.path.join(REPO, 'experiments', 'sim', 'nav2_se_loopback.yaml')) as f:
        old = yaml.safe_load(f)
    new = _params()

    def flat(d, prefix=''):
        out = {}
        for k, v in d.items():
            if isinstance(v, dict):
                out.update(flat(v, f'{prefix}{k}.'))
            else:
                out[f'{prefix}{k}'] = v
        return out
    fo, fn = flat(old), flat(new)
    changed = {k for k in fo if k in fn and fo[k] != fn[k]}
    added = set(fn) - set(fo)
    assert not set(fo) - set(fn)                              # nothing removed
    assert changed == {
        f'{plan.FP}.vx_min', 'controller_server.ros__parameters.failure_tolerance',
        'controller_server.ros__parameters.progress_checker.required_movement_radius',
        'controller_server.ros__parameters.progress_checker.movement_time_allowance',
        plan.BT_KEY, f'{plan.FP}.EscapeCritic.gap_weight'}
    assert all(k.startswith(f'{plan.FP}.se_') for k in added)


def test_overlay_keys_exist_in_the_file_and_set_the_tree_and_allowance():
    p = _params()
    for c in plan.CONFIG_ORDER:
        ov = plan.overlay(c)
        for k in ov:
            _get(p, k)                                          # KeyError if absent
        assert ov[plan.BT_KEY].endswith(plan.CONFIGS[c]['tree'])
        assert ov[plan.PROGRESS_TIME_KEY] == plan.CONFIGS[c]['allowance_s']
    # C3 and C4 differ from C1 and C2 only in the tree and the allowance
    assert set(plan.overlay('C1')) == set(plan.overlay('C4'))


def test_refusal_of_se_enabled_with_positive_vx_min():
    p = _params()
    plan.refuse_unsafe(p)                                       # the shipped file is accepted
    bad = _params()
    _get(bad, plan.FP)['vx_min'] = 0.1
    with pytest.raises(SystemExit, match='vx_min = 0.1'):
        plan.refuse_unsafe(bad)
    _get(bad, plan.FP)['vx_min'] = 0.0
    plan.refuse_unsafe(bad)                                     # the boundary is accepted
    _get(bad, plan.FP)['vx_min'] = 1e-9
    with pytest.raises(SystemExit):
        plan.refuse_unsafe(bad)
    _get(bad, plan.FP)['se_enabled'] = False
    plan.refuse_unsafe(bad)                                     # stock: not this rule


# --------------------------------------------------------------------------- #
# Behaviour trees
# --------------------------------------------------------------------------- #
def _tree(name):
    return ET.parse(os.path.join(plan.BT_DIR, name)).getroot()


def test_replanning_only_tree_has_no_recovery_node_and_no_recovery_action():
    root = _tree(plan.BT_REPLAN_ONLY)
    tags = {e.tag for e in root.iter()}
    assert 'RecoveryNode' not in tags
    for forbidden in ('Spin', 'BackUp', 'Wait', 'ClearEntireCostmap', 'RoundRobin', 'ReactiveFallback'):
        assert forbidden not in tags
    assert {'PipelineSequence', 'RateController', 'ComputePathToPose', 'FollowPath'} <= tags
    rate = root.find('.//RateController')
    assert rate.get('hz') == '1.0'
    assert root.find('.//FollowPath').get('controller_id') == '{selected_controller}'


def test_default_tree_is_a_copy_of_nav2s_and_has_the_recovery_branch():
    root = _tree(plan.BT_DEFAULT)
    tags = {e.tag for e in root.iter()}
    assert {'RecoveryNode', 'Spin', 'BackUp', 'Wait', 'ClearEntireCostmap'} <= tags
    assert root.find('.//RecoveryNode').get('number_of_retries') == '6'
    prefix = os.environ.get('CONDA_PREFIX', '')
    nav2 = os.path.join(prefix, 'share', 'nav2_bt_navigator', 'behavior_trees',
                        'navigate_to_pose_w_replanning_and_recovery.xml')
    if not os.path.exists(nav2):
        pytest.skip('Nav2 default tree not installed here')
    assert filecmp.cmp(nav2, os.path.join(plan.BT_DIR, plan.BT_DEFAULT), shallow=False)


def test_replanning_only_tree_keeps_the_default_trees_planner_and_controller_nodes():
    a, b = _tree(plan.BT_DEFAULT), _tree(plan.BT_REPLAN_ONLY)
    for tag in ('ControllerSelector', 'PlannerSelector', 'RateController', 'ComputePathToPose', 'FollowPath'):
        assert a.find(f'.//{tag}').attrib == b.find(f'.//{tag}').attrib, tag


def test_trees_are_installed_by_the_testbed_package():
    with open(os.path.join(REPO, 'experiments', 'se_mppi_utrap', 'CMakeLists.txt')) as f:
        assert 'behavior_trees' in f.read()


# --------------------------------------------------------------------------- #
# Scenarios and worlds
# --------------------------------------------------------------------------- #
def _scenario(name):
    with open(os.path.join(REPO, plan.SCENARIO_FILES[name])) as f:
        return yaml.safe_load(f)


def test_scenarios_share_start_goal_and_map():
    base = _scenario('s1_utrap')
    for n in ('s3_ctrap', 's4_ltrap', 's0_open'):
        s = _scenario(n)
        assert s['start'] == base['start'] == [-3.0, 0.0, 0.0]
        assert s['goal'] == base['goal'] == [3.0, 0.0, 0.0]
        assert s['map'] == base['map']
        assert s['meta']['launch'] == base['meta']['launch']


def test_campaign2_scenarios_name_an_existing_world_file_next_to_them():
    from experiments.runner import scenario as scenario_mod
    for n in ('s3_ctrap', 's4_ltrap', 's0_open'):
        sc = scenario_mod.load_scenario(os.path.join(REPO, plan.SCENARIO_FILES[n]))
        assert sc.name == n
        assert sc.world and os.path.exists(sc.world), sc.world
        assert os.path.basename(sc.world) == f'{n}.sdf'
        assert os.path.exists(sc.map_yaml)
        root = ET.parse(sc.world).getroot()
        assert root.find('world').get('name') == 'testbed'     # the recorder's ground-truth topic


def test_s1_uses_the_default_world_without_a_world_argument():
    from experiments.runner import scenario as scenario_mod
    for n in ('s1_utrap', 's1_utrap_evidence'):
        sc = scenario_mod.load_scenario(os.path.join(REPO, plan.SCENARIO_FILES[n]))
        assert sc.world is None


def test_held_out_world_names_only_inside_campaign2():
    hits = []
    for path in glob.glob(os.path.join(REPO, 'experiments', '**', '*'), recursive=True):
        if not os.path.isfile(path) or path.endswith(('.pyc', '.png', '.pgm', '.pdf')):
            continue
        if os.sep + 'campaign2' + os.sep in path:
            continue
        try:
            with open(path, errors='ignore') as f:
                text = f.read()
            if any(name in text for name in plan.HELD_OUT_POCKETS):
                hits.append(path)
        except OSError:
            pass
    assert hits == []
