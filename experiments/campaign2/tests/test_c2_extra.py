"""Additional campaign-2 tests: survivors of the mutation run, freeze manifest, recorder."""

import json
import os
import subprocess
import sys

import pytest

import analyze as an
import freeze_manifest as fm
import plan
from test_c2_analyze import analyze_one, drive_to_goal, make_trial, pocket_traj, rec


def test_setup_failure_is_not_a_valid_driven_trial():
    cell = an.cell_stats([rec('s0_open', 'C1', 'SETUP_FAIL'), rec('s0_open', 'C1', 'REACHED')])
    assert (cell['n_trials'], cell['n_valid'], cell['successes']) == (2, 1, 1)
    assert not an.is_valid_driven(rec('s0_open', 'C1', 'SETUP_FAIL'))
    assert not an.is_valid_driven(rec('s0_open', 'C1', 'NO_GOAL_SENT'))
    assert not an.is_valid_driven(rec('s0_open', 'C1', 'INVALID'))
    assert an.is_valid_driven(rec('s0_open', 'C1', 'NOT_REACHED_TIMEOUT'))
    assert an.is_valid_driven(rec('s0_open', 'C1', 'REACHED_WITH_CONTACT'))


def test_mann_whitney_u_is_the_smaller_statistic_whichever_group_is_first():
    low_first = an.mann_whitney([1.0, 2.0, 3.0], [4.0, 5.0, 6.0])
    high_first = an.mann_whitney([4.0, 5.0, 6.0], [1.0, 2.0, 3.0])
    assert low_first['U'] == high_first['U'] == 0
    assert low_first['p'] == pytest.approx(high_first['p'])
    partial = an.mann_whitney([1.0, 4.0, 5.0], [2.0, 3.0, 6.0])
    assert partial['U'] == 4 and partial['p'] == pytest.approx(1.0)


def test_s1_pocket_boundary_counts_as_inside():
    poly = an.POCKETS['s1_utrap']
    for pt in ((0.85, 0.0), (-1.0, 0.0), (0.0, 0.85), (0.0, -0.85), (-1.0, -0.85), (0.85, 0.85)):
        assert an.inside_polygon(pt, poly), pt
    for pt in ((0.8501, 0.0), (-1.0001, 0.0), (0.0, 0.8501), (0.0, -0.8501)):
        assert not an.inside_polygon(pt, poly), pt


def test_s3_pocket_mouth_and_corners_in_the_pocket_frame():
    """The mouth line is the end of the longer side wall (x -0.3), the faces are at 1.15 and +-1.05."""
    poly = an.POCKETS['s3_ctrap']
    rot = lambda x, y: an._rotate((x, y), an.S3_PIVOT, an.S3_ROT_DEG)
    for pt in ((-0.25, 1.0), (-0.25, -1.0), (1.1, 1.0), (1.1, -1.0), (0.4, 0.0)):
        assert an.inside_polygon(rot(*pt), poly), pt
    for pt in ((-0.35, 1.0), (-0.35, -1.0), (-0.35, 0.0), (1.2, 0.0), (0.4, 1.1), (0.4, -1.1)):
        assert not an.inside_polygon(rot(*pt), poly), pt
    assert len(poly) == 4


def test_a_reached_trial_that_never_entered_the_pocket_reached_without_escape(tmp_path):
    t = analyze_one(tmp_path, 'n', 's1_utrap', 'C4', drive_to_goal(30.0, via=[(-2.0, 2.5), (3.0, 2.5)]))
    assert t['success'] is True and t['entered_pocket_s'] is None
    assert t['enter_before_leaving_pocket'] is None and t['reached_without_escape'] is True


def test_h3_counts_only_valid_trials():
    cells = {('s0_open', 'C1'): an.cell_stats(
        [rec('s0_open', 'C1', 'REACHED')] * 5 + [rec('s0_open', 'C1', 'INVALID_HOST_LOAD')])}
    h = an.h3(cells)
    assert (h['reached'], h['n_valid']) == (5, 5)


def test_count_invalid_attempts_counts_only_invalid_directories_of_the_cell(tmp_path):
    root = str(tmp_path)
    for name in ('correct__s1_utrap__C1__rep0.invalid1', 'correct__s1_utrap__C1__rep0.invalid2',
                 'correct__s1_utrap__C1__rep3.invalid1', 'correct__s1_utrap__C1__rep0.aborted1',
                 'correct__s1_utrap__C2__rep0.invalid1', 'correct__s1_utrap__C1__rep0'):
        os.makedirs(os.path.join(root, 'trials', name))
    assert an.count_invalid_attempts(root, ('s1_utrap', 'C1')) == 3
    assert an.count_invalid_attempts(root, ('s1_utrap', 'C2')) == 1
    assert an.count_invalid_attempts(root, ('s3_ctrap', 'C1')) == 0


def test_invalid_attempts_reach_the_stop_rule_through_build(tmp_path):
    root = str(tmp_path / 'raw')
    for k in range(1, 8):
        make_trial(root, f'correct__s0_open__C1__rep0.invalid{k}', 's0_open', 'C1', drive_to_goal(20.0), foreign=2.0)
    make_trial(root, 'correct__s0_open__C1__rep0', 's0_open', 'C1', drive_to_goal(20.0))
    result = an.build(root)
    assert any(code == 'INVALID_CAP' for code, _ in result['stops'])


# --------------------------------------------------------------------------- #
# Freeze manifest
# --------------------------------------------------------------------------- #
def test_frozen_files_all_exist_and_cover_the_inputs():
    files = fm.files()
    for rel in files:
        assert os.path.exists(os.path.join(fm.REPO, rel)), rel
    for must in ('experiments/sim/nav2_se_campaign2.yaml',
                 'experiments/se_mppi_utrap/behavior_trees/nav_to_pose_replanning_only.xml',
                 'experiments/se_mppi_utrap/behavior_trees/nav_to_pose_default_recovery.xml',
                 'experiments/campaign2/analyze.py', 'experiments/campaign2/plan.py',
                 'experiments/campaign2/tests/test_c2_analyze.py',
                 'experiments/campaign2/worlds/s3_ctrap.sdf',
                 'experiments/campaign2/worlds/s4_ltrap.sdf',
                 'experiments/campaign2/s4_ltrap.scenario.yaml',
                 'src/nav2_se_controller/src/safe_escape_controller.cpp',
                 'experiments/results_pilot_rerun/hostload.py'):
        assert must in files, must
    assert 'experiments/campaign2/PREREG.md' not in files
    assert not any(f.startswith('experiments/campaign2/results') for f in files)


def test_compare_reports_changed_missing_and_new():
    assert fm.compare({'a': '1', 'b': '2'}, {'a': '1', 'b': '2'}) == []
    d = fm.compare({'a': '1', 'b': '2', 'c': '3'}, {'a': '1', 'b': 'X', 'd': '4'})
    assert d == ['changed: b', 'missing: c', 'new (not in the manifest): d']


def test_check_fails_without_a_manifest_and_after_a_change(tmp_path):
    assert 'not frozen' in fm.check(str(tmp_path / 'none.json'))[0]
    path = tmp_path / 'm.json'
    data = {'files': fm.compute()}
    path.write_text(json.dumps(data))
    assert fm.check(str(path)) == []
    data['files']['experiments/sim/nav2_se_campaign2.yaml'] = '0' * 64
    path.write_text(json.dumps(data))
    assert fm.check(str(path)) == ['changed: experiments/sim/nav2_se_campaign2.yaml']


def test_sha256_matches_the_standard_tool(tmp_path):
    p = tmp_path / 'f'
    p.write_bytes(b'campaign 2\n')
    out = subprocess.run(['sha256sum', str(p)], capture_output=True, text=True).stdout.split()[0]
    assert fm.sha256(str(p)) == out == plan.sha256_file(str(p))


# --------------------------------------------------------------------------- #
# Process exit codes: teardown signals are normal, anything else is a crash
# --------------------------------------------------------------------------- #
def died(code, pid=4242):
    return (f"[ERROR] [component_container_isolated-5]: process has died [pid {pid}, exit code {code}, "
            "cmd '/x/lib/rclcpp_components/component_container_isolated --ros-args'].")


@pytest.mark.parametrize('code,crash', [(0, False), (-2, False), (-15, False), (-9, False),
                                        (-11, True), (-6, True), (1, True), (134, True)])
def test_exit_codes_of_the_teardown_are_not_crashes(code, crash):
    p = an.parse_launch_log(died(code), 0.0)
    assert p['process_exit_codes'] == [code] and p['process_crash'] is crash


def test_no_died_line_is_no_crash_and_several_lines_are_all_listed():
    p = an.parse_launch_log('nothing', 0.0)
    assert p['process_exit_codes'] == [] and p['process_crash'] is False
    p = an.parse_launch_log(died(-9, 1) + '\n' + died(-11, 2) + '\n' + died(-2, 3), 0.0)
    assert p['process_exit_codes'] == [-9, -11, -2] and p['process_crash'] is True


def test_a_crash_in_the_launch_log_reaches_the_record_and_the_stop_rule(tmp_path):
    t = analyze_one(tmp_path, 'c', 's0_open', 'C1', drive_to_goal(30.0), launch=died(-11))
    assert t['process_crash'] is True and t['process_exit_codes'] == [-11]
    stops = an.evaluate_stops([t])
    assert [c for c, _ in stops] == ['CONTROLLER_CRASH'] and '-11' in stops[0][1]
    ok = analyze_one(tmp_path, 'ok', 's0_open', 'C1', drive_to_goal(30.0), launch=died(-9))
    assert ok['process_crash'] is False and an.evaluate_stops([ok]) == []
