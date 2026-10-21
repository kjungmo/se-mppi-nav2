"""Tests of amendment 3 (PREREG.md, 2026-10-22): a launched process that dies with a non-teardown
exit code is a TEARDOWN_CRASH, and never stops the campaign, only if launch.log shows that it
died after the driver's teardown began. Every other case stays a CONTROLLER_CRASH."""

import json
import os

import pytest

import analyze as an
import plan
import provenance as pv
import run_campaign2 as drv
from test_c2_analyze import T0, analyze_one, drive_to_goal, make_trial

FIX = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures', 's4_ltrap_C3_rep0')
CONT = 'component_container_isolated-5'


def fixture_text(name):
    with open(os.path.join(FIX, name)) as f:
        return f.read()


def fixture_meta():
    return json.loads(fixture_text('meta_excerpt.json'))


def sig(name, stamp):
    return f'[{name}] [INFO] [{stamp:.9f}] [rclcpp]: signal_handler(SIGINT/SIGTERM)'


def died(name, code, pid=4242):
    return (f"[ERROR] [{name}]: process has died [pid {pid}, exit code {code}, "
            "cmd '/x/lib/rclcpp_components/component_container_isolated --ros-args'].")


# window of the synthetic trials: the teardown starts at T0 + 62, the trial ends at T0 + 70
LO, HI = T0 + 62.0, T0 + 70.0
WINDOW = (LO, HI)


# --------------------------------------------------------------------------- #
# The real trial that fired the rule
# --------------------------------------------------------------------------- #
def test_the_real_trial_is_a_teardown_crash_from_its_existing_files():
    meta = fixture_meta()
    window = an.teardown_window(meta)
    assert window == (meta['drive_wall_end'], meta['wall_end'])      # recorded before amendment 3
    p = an.parse_launch_log(fixture_text('launch_excerpt.log'), None, window)
    assert p['process_exit_codes'] == [-6]
    assert p['process_crash'] is False
    (e,) = p['teardown_crashes']
    assert e['process'] == CONT and e['exit_code'] == -6
    assert e['signal_handler_stamp'] == pytest.approx(1792603420.1337, abs=1e-3)
    assert e['stamp_after_window_start_s'] == pytest.approx(3.0, abs=0.01)
    assert e['stamp_before_trial_end_s'] == pytest.approx(4.0, abs=0.01)
    assert an.evaluate_stops([dict(trial_id='t', scenario='s4_ltrap', config='C3', outcome='X', **p)]) == []


def test_the_real_trial_without_a_window_stays_a_crash():
    p = an.parse_launch_log(fixture_text('launch_excerpt.log'), None)
    assert p['process_crash'] is True and p['teardown_crashes'] == []


def test_the_real_trial_with_a_window_that_ends_before_the_signal_stays_a_crash():
    meta = fixture_meta()
    p = an.parse_launch_log(fixture_text('launch_excerpt.log'), None, (meta['drive_wall_end'], 1792603419.0))
    assert p['process_crash'] is True


# --------------------------------------------------------------------------- #
# Synthetic cases of the rule
# --------------------------------------------------------------------------- #
def classify(lines, window=WINDOW):
    return an.parse_launch_log('\n'.join(lines) + '\n', None, window)


def test_a_death_after_the_processs_own_teardown_signal_is_a_teardown_crash():
    p = classify([sig(CONT, T0 + 63.0), died(CONT, -11)])
    assert p['process_crash'] is False and [e['exit_code'] for e in p['teardown_crashes']] == [-11]


def test_a_death_before_the_teardown_is_a_crash():
    # launch logs the death first; the signal lines come later (the launch or the driver stops the rest)
    p = classify([died(CONT, -11), sig(CONT, T0 + 63.0)])
    assert p['process_crash'] is True and p['teardown_crashes'] == []


def test_a_signal_stamp_before_the_teardown_window_is_a_crash():
    p = classify([sig(CONT, T0 + 40.0), died(CONT, -6)])          # earlier than the drive end
    assert p['process_crash'] is True


def test_a_signal_stamp_after_the_end_of_the_trial_is_a_crash():
    p = classify([sig(CONT, HI + 5.0), died(CONT, -6)])
    assert p['process_crash'] is True


def test_another_processs_signal_line_does_not_explain_this_death():
    p = classify([sig('parameter_bridge-4', T0 + 63.0), died(CONT, -11)])
    assert p['process_crash'] is True


def test_ordering_unknown_is_a_crash():
    assert classify([died(CONT, -11)])['process_crash'] is True               # no signal line at all
    assert classify([sig(CONT, T0 + 63.0), died(CONT, -11)], None)['process_crash'] is True
    assert classify([sig(CONT, T0 + 63.0), 'odd ' + died(CONT, -11)])['process_crash'] is True
    # a window whose bounds the driver did not record
    assert an.teardown_window({'drive_wall_end': 5.0}) is None
    assert an.teardown_window({'wall_end': 5.0}) is None


def test_one_crash_and_one_teardown_crash_in_the_same_log_are_a_crash():
    p = classify([died('gz-1', 1), sig(CONT, T0 + 63.0), died(CONT, -6)])
    assert p['process_crash'] is True and len(p['teardown_crashes']) == 1


@pytest.mark.parametrize('code', [0, -2, -15, -9])
def test_teardown_exit_codes_are_neither_crash_nor_teardown_crash(code):
    p = classify([died(CONT, code)])
    assert p['process_crash'] is False and p['teardown_crashes'] == []


def test_the_window_prefers_the_recorded_sigint_then_the_teardown_start_then_the_drive_end():
    meta = {'drive_wall_end': 10.0, 'wall_end': 30.0}
    assert an.teardown_window(meta) == (10.0, 30.0)
    assert an.teardown_window(dict(meta, teardown_wall_start=12.0)) == (12.0, 30.0)
    assert an.teardown_window(dict(meta, teardown_wall_start=12.0, launch_sigint_wall=15.0)) == (15.0, 30.0)


# --------------------------------------------------------------------------- #
# The record, the stop rule and the driver's stop check
# --------------------------------------------------------------------------- #
def trial_with_teardown(root, name, launch_lines, *, sub='trials', config='C3', scenario='s0_open',
                        with_window=True):
    d = make_trial(root, name, scenario, config, [(0.0, -3.0, 0.0, 0.0), (60.0, 3.0, 0.0, 0.0)],
                   sub=sub, launch='\n'.join(launch_lines) + '\n')
    path = os.path.join(d, 'meta.json')
    with open(path) as f:
        meta = json.load(f)
    if with_window:
        meta.update(teardown_wall_start=T0 + 61.0, launch_sigint_wall=LO, wall_end=HI)
    with open(path, 'w') as f:
        json.dump(meta, f)
    return d


def test_the_analysis_record_carries_the_teardown_crash_and_no_stop(tmp_path):
    root = str(tmp_path / 'raw')
    trial_with_teardown(root, 'correct__s0_open__C3__rep0', [sig(CONT, T0 + 63.0), died(CONT, -6)])
    cells = sorted({t.cell for t in plan.plan('all')})
    assert drv.check_stops(root, cells) == []
    (t,) = an.registered_records(root)
    assert t['process_crash'] is False and t['teardown_crashes'][0]['exit_code'] == -6
    assert t['outcome'] == 'REACHED'                                # the trial keeps its outcome


def test_a_crash_before_the_teardown_stops_the_driver(tmp_path):
    root = str(tmp_path / 'raw')
    trial_with_teardown(root, 'correct__s0_open__C3__rep0', [died(CONT, -6), sig(CONT, T0 + 63.0)])
    cells = sorted({t.cell for t in plan.plan('all')})
    assert [c for c, _ in drv.check_stops(root, cells)] == ['CONTROLLER_CRASH']


def test_a_trial_recorded_without_a_window_stays_a_crash(tmp_path):
    root = str(tmp_path / 'raw')
    d = trial_with_teardown(root, 'correct__s0_open__C3__rep0',
                            [sig(CONT, T0 + 63.0), died(CONT, -6)], with_window=False)
    meta = json.load(open(os.path.join(d, 'meta.json')))
    assert 'wall_end' not in meta
    cells = sorted({t.cell for t in plan.plan('all')})
    assert [c for c, _ in drv.check_stops(root, cells)] == ['CONTROLLER_CRASH']


def test_a_crash_in_an_invalid_attempt_is_still_scanned(tmp_path):
    root = str(tmp_path / 'raw')
    trial_with_teardown(root, 'correct__s0_open__C3__rep0.invalid1', [died(CONT, -11), sig(CONT, T0 + 63.0)])
    cells = sorted({t.cell for t in plan.plan('all')})
    stops = drv.check_stops(root, cells)
    assert [c for c, _ in stops] == ['CONTROLLER_CRASH'] and 'rep0.invalid1' in stops[0][1]


def test_a_teardown_crash_in_an_invalid_attempt_is_listed_and_does_not_stop(tmp_path):
    root = str(tmp_path / 'raw')
    trial_with_teardown(root, 'correct__s0_open__C3__rep0.invalid1', [sig(CONT, T0 + 63.0), died(CONT, -6)])
    cells = sorted({t.cell for t in plan.plan('all')})
    assert drv.check_stops(root, cells) == []
    rep = an.teardown_crash_report([], an.superseded_records(root))
    assert rep['total'] == 1 and rep['rows'][0]['registered'] is False


def test_the_results_list_every_teardown_crash_with_its_timing_and_the_counts(tmp_path):
    root = str(tmp_path / 'raw')
    trial_with_teardown(root, 'correct__s0_open__C3__rep0', [sig(CONT, T0 + 63.0), died(CONT, -6)])
    trial_with_teardown(root, 'correct__s0_open__C3__rep1', [sig(CONT, T0 + 64.0), died(CONT, -11)],
                        config='C3')
    trial_with_teardown(root, 'correct__s0_open__C1__rep0', [], config='C1')
    result = an.build(root)
    tc = result['teardown_crashes']
    assert tc['total'] == 2 and tc['by_config'] == {'C3': 2} and tc['by_scenario'] == {'s0_open': 2}
    assert tc['by_cell'] == {'s0_open/C3': 2}
    assert {r['trial_id'] for r in tc['rows']} == {'correct__s0_open__C3__rep0', 'correct__s0_open__C3__rep1'}
    md = an.render(result)
    assert '## Teardown crashes' in md and 'correct__s0_open__C3__rep1' in md
    assert f'{T0 + 64.0:.3f}' in md and 'C3 2' in md


def test_the_driver_records_the_teardown_times(monkeypatch, tmp_path):
    from test_c2_review_fixes import FakeProc, TRIAL, fake_stack, read_meta, run
    FakeProc.behaviour = 'ok'
    run(fake_stack(monkeypatch), tmp_path)
    meta = read_meta(str(tmp_path / TRIAL.trial_id))
    assert meta['drive_wall_end'] <= meta['teardown_wall_start'] <= meta['launch_sigint_wall'] <= meta['wall_end']


# --------------------------------------------------------------------------- #
# MIXED_ATTEMPTS and the foreign-trial check with the amended prior identity
# --------------------------------------------------------------------------- #
PRIOR = {'freeze_manifest_sha256': 'd96cc1c24798536d7e64bb43eef1f15436d77086c7fd9bbb7667fdadfef17f00',
         'controller_tag_commit': '012f4f1db11de8ce607854578cb99ab932bf3969'}
NEW = {'freeze_manifest_sha256': 'n' * 64, 'controller_tag_commit': 'n' * 40}
THIRD = {'freeze_manifest_sha256': 'x' * 64, 'controller_tag_commit': 'x' * 40}


def ident_rec(ident):
    return {'trial_id': 't', 'scenario': 's1_utrap', 'config': 'C4', 'outcome': 'REACHED',
            'success': True, **ident}


def stop_codes(*idents):
    return [c for c, _ in an.evaluate_stops([ident_rec(i) for i in idents])]


def test_the_prior_identity_beside_the_current_one_is_not_mixed():
    assert 'MIXED_ATTEMPTS' not in stop_codes(PRIOR, NEW)
    assert 'MIXED_ATTEMPTS' not in stop_codes(PRIOR)
    assert 'MIXED_ATTEMPTS' not in stop_codes(NEW)


def test_any_other_identity_beside_them_is_still_mixed():
    assert 'MIXED_ATTEMPTS' in stop_codes(PRIOR, NEW, THIRD)
    assert 'MIXED_ATTEMPTS' in stop_codes(NEW, THIRD)
    # half of the prior pair is not the prior pair
    half = {'freeze_manifest_sha256': PRIOR['freeze_manifest_sha256'], 'controller_tag_commit': 'z' * 40}
    assert 'MIXED_ATTEMPTS' in stop_codes(NEW, half)


def test_the_driver_accepts_a_trial_of_the_prior_identity_and_refuses_a_third():
    assert pv.foreign_reason(dict(PRIOR), NEW) is None
    assert pv.foreign_reason(dict(THIRD), NEW) is not None
    assert pv.foreign_reason({}, NEW) is not None


def test_the_prior_identity_is_the_recorded_one():
    assert pv.AMENDED_PRIOR_IDENTITIES == (PRIOR,)
    meta = fixture_meta()
    assert {k: meta[k] for k in PRIOR} == PRIOR
