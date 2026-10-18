"""Tests for the fixes of the first Opus review (2026-10-18): resume and restart, the block-B
gate, the stopping rules, the pre-registration text against the frozen files."""

import json
import os
import re
import sys
import types

import pytest
import yaml

import analyze as an
import freeze_manifest as fm
import plan
import provenance as pv
import run_campaign2 as drv
from test_c2_analyze import cell_trials, make_trial, rec

PREREG = os.path.join(plan.HERE, 'PREREG.md')
NOTE = os.path.join(plan.REPO, 'docs', 'architecture', '2026-10_escape-v2.md')


def prereg_text():
    with open(PREREG) as f:
        return f.read()


def flat(text):
    return re.sub(r'\s+', ' ', text)


# --------------------------------------------------------------------------- #
# Fisher numbers quoted in PREREG (finding: the "smallest n" sentence was false)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('a,b,c,d,p', [
    (6, 0, 0, 6, 0.0022), (5, 1, 0, 6, 0.0152), (4, 2, 0, 6, 0.0606),
    (5, 0, 0, 5, 0.0079), (4, 1, 0, 5, 0.0476)])
def test_fisher_numbers_in_the_prereg_are_the_repository_values(a, b, c, d, p):
    assert an.pilot.fisher_two_sided(a, b, c, d) == pytest.approx(p, abs=5e-5)
    assert f'{p:.4f}' in prereg_text()


def test_n5_with_one_failure_is_below_the_threshold_so_the_old_claim_was_false():
    assert an.pilot.fisher_two_sided(4, 1, 0, 5) < 0.05
    text = flat(prereg_text())
    assert 'smallest n at which one failure still leaves a significant contrast' not in text
    assert 'at n = 5 one failure gives p = 0.048' in text


def test_decision_rule_cells_at_n6():
    # the statements of the Decision rule section
    assert an.pilot.fisher_two_sided(5, 1, 0, 6) < 0.05           # 5 of 6 against 0 of 6: yes
    assert an.pilot.fisher_two_sided(6, 0, 1, 5) < 0.05           # 6 of 6 against 1 of 6: yes
    assert an.pilot.fisher_two_sided(5, 1, 1, 5) > 0.05           # 5 of 6 against 1 of 6: no
    assert an.pilot.fisher_two_sided(4, 2, 0, 6) > 0.05           # 4 of 6 against 0 of 6: no


# --------------------------------------------------------------------------- #
# PREREG against the frozen parameter file and the verbatim wording
# --------------------------------------------------------------------------- #
def test_every_escape_key_of_the_frozen_file_is_a_row_of_the_prereg_table_with_its_value():
    with open(plan.PARAMS_YAML) as f:
        fp = yaml.safe_load(f)['controller_server']['ros__parameters']['FollowPath']
    keys = [k for k in fp if k.startswith('se_')]
    assert len(keys) >= 17
    rows = {}
    for ln in prereg_text().splitlines():
        m = re.match(r'^\| `(se_[a-z_]+)` \| (.*?) \|', ln)
        if m:
            rows[m.group(1)] = ln
    for k in keys:
        assert k in rows, f'{k} is in nav2_se_campaign2.yaml and not in the PREREG table'
        v = fp[k]
        shown = rows[k].split('|')[2].strip()
        if isinstance(v, bool):
            assert shown == ('true' if v else 'false'), k
        else:
            assert re.match(rf'^{re.escape(str(v))}( |$)', shown) or float(re.match(r'[-0-9.]+', shown).group()) == float(v), (k, v, shown)


def test_the_additions_are_marked_as_not_in_the_advice():
    rows = {m.group(1): ln for ln in prereg_text().splitlines()
            if (m := re.match(r'^\| `(se_[a-z_]+)` \|', ln))}
    for k in ('se_escape_stall_time', 'se_escape_stall_distance', 'se_escape_avoid_radius',
              'se_escape_avoid_hop_radius', 'se_escape_avoid_hops'):
        assert '**no (addition' in rows[k], k
    for k in ('se_progress_min_advance', 'se_escape_waypoint_cap', 'se_gap_num_rays'):
        assert '| advice |' in rows[k], k
    text = flat(prereg_text())
    for needle in ('at most 6 failed waypoints', 'closer than 0.2 m is not taken',
                   'static gap tests at the stall pose of the live scenario `s1_utrap`',
                   '`6d0d9ce` and `5f17e19`', 'departure from decision D-1',
                   'The Python mirror was not the source'):
        assert needle in text, needle
    assert 'set from the mirror and the development-geometry test' not in text


def test_the_yaml_header_does_not_call_every_escape_parameter_advised():
    with open(plan.PARAMS_YAML) as f:
        head = flat(''.join(ln[1:] for ln in f.read().splitlines()[:40] if ln.startswith('#')))
    assert 'Only part of them are in DESIGN_ADVICE section 4.2' in head
    for k in ('se_escape_avoid_radius', 'se_escape_stall_time', '0.2 m floor'):
        assert k in head, k
    assert 'The escape parameters of DESIGN_ADVICE section 4.2 are written out' not in head


QUOTE = '''> - *Demonstrated*: "SE-MPPI without recoveries reached the goal in k of 6 on s1
>   and k' of 6 on the held-out s3; stock without recoveries in 0 of 6; with
>   Nav2's default recovery tree, stock m of 6 and SE-MPPI m' of 6", Fisher exact
>   p per contrast, counts and Wilson intervals only, "on one simulated host". s1
>   is the development scenario; s3 carries the claim. Section 4.4 and the
>   abstract's "no live run has yet reached its goal" are replaced; the pilot's
>   host attribution is withdrawn (campaign 1: RTF 0.96).
> - *s1 yes, s3 no*: report both; the claim shrinks to "escapes the development
>   geometry live" and the paper says the mechanism is geometry-sensitive.
> - *Not demonstrated*: no working-escape claim. Hold the submission, or publish a
>   system report whose abstract says the plugin does not reach the goal live and
>   whose contribution is the mirror-to-controller gap and the four defects. I
>   advise holding unless a deadline forces the second.
>
> In every case `3_method.tex` eq. (detect) and the gap equation, Algorithm 1
> (substitution before the base call; exception handling), `app:mirror`,
> `NUMBERS.md` and `check_paper_numbers.py` change with the code. "Coordination is
> null" stands; campaign 2 does not test it.'''


def test_the_verbatim_outcome_wording_is_unchanged_and_next_to_the_decision_rule():
    text = prereg_text()
    assert QUOTE in text                                   # D-4(c): copied verbatim, unchanged
    assert text.index('### Decision rule') < text.index('## Outcome wording')
    assert text.index('## Outcome wording') < text.index(QUOTE)


def test_decision_rule_names_level_branches_and_descriptive_parts():
    sec = flat(prereg_text().split('### Decision rule')[1].split('## Stopping rules')[0])
    for needle in ('0.05, two-sided, per contrast, with no correction for multiple tests',
                   'Each tests its own hypothesis',
                   'two-sided Fisher p of C4 against C3 on that pocket is below 0.05',
                   'C4 reached more often than C3',
                   'Demonstrated = s1 yes and s3 yes',
                   '"s1 yes, s3 no" = s1 yes and s3 no',
                   'Not demonstrated = s1 no, whatever s3 shows',
                   'H2 (C2 against C1)', 'decide nothing'):
        assert needle in sec, needle
    h = flat(prereg_text().split('## Hypotheses')[1].split('## Fixed inputs')[0])
    assert '**H2 is descriptive.**' in h and 'cannot be tested at n = 6' in h


def test_s3_is_disclosed_as_having_a_near_mirror_twin_and_h4_is_narrowed():
    text = flat(prereg_text())
    assert 'w22_d15_r-25_a6' in text and 'near mirror twin' in text
    assert 'does **not** mean that s3 lies outside the span of the development family' in text
    h = flat(prereg_text().split('## Hypotheses')[1].split('## Fixed inputs')[0])
    assert 'not driven live and not run in the headless test' in h
    assert 'H4 does not claim a geometry outside the development family' in h
    assert 'a geometry that was not used during development' not in h


def test_the_s3_twin_is_a_valid_member_of_the_family_and_close_to_s3():
    """The numbers in 'Held-out status of s3' against the family rule and the s3 world."""
    # family rule (UTRAP_DEV_GEOMETRIES.md): widths 2.2/2.6, depth 1.5, rotation -25, asymmetric 0.6
    md = open(os.path.join(plan.REPO, 'src', 'nav2_se_controller', 'test', 'deployment',
                           'UTRAP_DEV_GEOMETRIES.md')).read()
    assert re.search(r'width `w` \(m\) \| 1\.3, 1\.6, 2\.2, 2\.6', md)
    assert re.search(r'depth `d` \(m\) \| 1\.1, 1\.5, 1\.8', md)
    assert re.search(r'rotation `theta` \(deg\) \| -25, 0, 15, 30', md)
    boxes = dict(an.world_boxes(an.WORLDS['s3_ctrap']))
    assert {'c_back', 'c_top', 'c_bot'} <= set(boxes)
    # inner size from the pocket polygon (analysis constants)
    xs = [p[0] for p in an.S3_POCKET_FRAME]
    ys = [p[1] for p in an.S3_POCKET_FRAME]
    width, depth = max(ys) - min(ys), max(xs) - min(xs)
    assert (round(width, 2), round(depth, 2)) == (2.1, 1.45)
    assert abs(width - 2.2) <= 0.1 + 1e-9 and abs(depth - 1.5) <= 0.05 + 1e-9
    assert an.S3_ROT_DEG == 25.0                                     # the twin is -25 (reflection)
    assert abs(1.1 / 1.6 - 0.6) <= 0.09 + 1e-9


def test_failure_tolerance_zero_and_the_detector_update_are_stated():
    text = flat(prereg_text())
    assert 'cannot act in this campaign' in text and 'publishZeroVelocity()' in text
    note = flat(open(NOTE).read())
    assert 'cannot act under `failure_tolerance` 0.0' in note and 'reset()' in note
    cl = open(os.path.join(plan.REPO, 'src', 'nav2_se_controller', 'test', 'deployment',
                           'test_closed_loop_utrap.cpp')).read()
    assert 'if (r.throws != 0)' in cl and 'C4 would abort' in cl


def test_freeze_steps_commit_prereg_and_manifest_before_the_tag():
    steps = prereg_text().split('## Before the freeze')[1]
    i_commit = steps.index('Commit this file and `FREEZE_MANIFEST.json`')
    i_tag = steps.index('Tag that commit')
    assert i_commit < i_tag
    assert 'the tag must not come first' in steps
    assert steps.index('freeze_manifest.py') < i_commit


def test_resume_and_stop_rules_are_stated_in_the_prereg():
    text = flat(prereg_text())
    for needle in ('marked `interrupted` and never `complete`', '`.abortedN`', '--attempt 2',
                   'attempt2/', '`MIXED_ATTEMPTS`', 'also scans the superseded attempts',
                   '2 or more valid failures', 'refuses `--block B`',
                   'byte-identical to the repository file', 'build stamp',
                   '`git_describe` must be exactly `campaign2-controller`'):
        assert needle in text, needle


def test_freeze_manifest_covers_provenance_and_the_installing_package():
    files = fm.files()
    for must in ('experiments/campaign2/provenance.py', 'experiments/se_mppi_utrap/CMakeLists.txt',
                 'src/nav2_se_controller/cmake/write_build_stamp.cmake',
                 'experiments/campaign2/tests/test_c2_review_fixes.py',
                 'experiments/campaign2/tests/test_c2_equivalence.py'):
        assert must in files, must


# --------------------------------------------------------------------------- #
# Stopping rules
# --------------------------------------------------------------------------- #
def test_stack_broken_fires_at_the_second_failure_not_after_the_sixth_trial():
    for cfg in ('C1', 'C3'):
        assert not an.evaluate_stops(cell_trials('s0_open', cfg, 1, n=2) [:1])      # one trial
        one_fail = [rec('s0_open', cfg, 'REACHED'), rec('s0_open', cfg, 'NOT_REACHED_ABORTED')]
        assert not any(c == 'STACK_BROKEN' for c, _ in an.evaluate_stops(one_fail))
        two_fail = one_fail + [rec('s0_open', cfg, 'NOT_REACHED_TIMEOUT')]
        stops = an.evaluate_stops(two_fail)
        assert [c for c, _ in stops] == ['STACK_BROKEN']
        assert 'at most 4 of 6' in stops[0][1]


def test_stack_broken_counts_contact_as_a_failure_and_ignores_invalid_and_se_cells():
    base = [rec('s0_open', 'C1', 'REACHED_WITH_CONTACT'), rec('s0_open', 'C1', 'NOT_REACHED_ABORTED')]
    assert any(c == 'STACK_BROKEN' for c, _ in an.evaluate_stops(base))
    inv = [rec('s0_open', 'C1', 'INVALID_HOST_LOAD'), rec('s0_open', 'C1', 'NOT_REACHED_ABORTED'),
           rec('s0_open', 'C1', 'SETUP_FAIL')]
    assert not any(c == 'STACK_BROKEN' for c, _ in an.evaluate_stops(inv))
    se = [rec('s0_open', 'C2', 'NOT_REACHED_ABORTED')] * 4 + [rec('s0_open', 'C4', 'NOT_REACHED_ABORTED')] * 4
    assert not any(c == 'STACK_BROKEN' for c, _ in an.evaluate_stops(se))


def test_stack_broken_boundary_with_the_plan_constants():
    assert plan.STOCK_S0_MIN_FAILURES == plan.REPS - plan.STOCK_S0_MAX_SUCCESSES == 2


def crash_rec(tid):
    r = rec('s1_utrap', 'C4', 'NOT_REACHED_ABORTED', trial_id=tid)
    r.update(process_crash=True, process_exit_codes=[-11])
    return r


def test_a_crash_in_a_superseded_attempt_is_a_stop():
    assert an.evaluate_stops([], None, None) == []
    stops = an.evaluate_stops([], None, [crash_rec('correct__s1_utrap__C4__rep0.invalid1')])
    assert [c for c, _ in stops] == ['CONTROLLER_CRASH'] and 'rep0.invalid1' in stops[0][1]
    # and the same record among the registered trials is also a stop (unchanged)
    assert any(c == 'CONTROLLER_CRASH' for c, _ in an.evaluate_stops([crash_rec('x')]))


def test_the_driver_sees_a_crash_in_an_attempt_that_was_renamed_invalid(tmp_path):
    root = str(tmp_path / 'raw')
    died = ('[ERROR] [launch]: process has died [pid 4242, exit code -11, cmd \'component_container_isolated\']\n')
    make_trial(root, 'correct__s1_utrap__C4__rep0.invalid1', 's1_utrap', 'C4',
               [(0.0, -3.0, 0.0, 0.0), (5.0, -2.0, 0.0, 0.0)], launch=died)
    cells = sorted({t.cell for t in plan.plan('all')})
    stops = drv.check_stops(root, cells)
    assert any(c == 'CONTROLLER_CRASH' for c, _ in stops), stops


def test_mixed_attempts_stop():
    a = rec('s1_utrap', 'C4', 'REACHED', freeze_manifest_sha256='aa', controller_tag_commit='cc')
    b = rec('s1_utrap', 'C3', 'NOT_REACHED_ABORTED', freeze_manifest_sha256='bb', controller_tag_commit='cc')
    assert any(c == 'MIXED_ATTEMPTS' for c, _ in an.evaluate_stops([a, b]))
    assert not any(c == 'MIXED_ATTEMPTS' for c, _ in an.evaluate_stops([a, dict(b, freeze_manifest_sha256='aa')]))


# --------------------------------------------------------------------------- #
# Block B and s3 wait for Phase A
# --------------------------------------------------------------------------- #
def phase_a_records(c4_reached, c4_n=6, c3_n=3):
    return (cell_trials('s1_utrap', 'C4', c4_reached, n=c4_n) +
            [rec('s1_utrap', 'C3', 'NOT_REACHED_ABORTED') for _ in range(c3_n)])


def test_s3_refusal_until_phase_a_is_complete_and_the_gate_has_not_fired():
    assert 'not complete' in an.s3_refusal([])
    assert 'not complete' in an.s3_refusal(phase_a_records(6, c4_n=5))          # 5 of 6 valid C4
    assert 'not complete' in an.s3_refusal(phase_a_records(6, c3_n=2))          # 2 of 3 valid C3
    assert an.s3_refusal(phase_a_records(6)) is None
    assert an.s3_refusal(phase_a_records(2)) is None                           # gate: 2 of 6 > 1
    assert 'gate fired' in an.s3_refusal(phase_a_records(1))
    assert 'gate fired' in an.s3_refusal(phase_a_records(0))


def test_invalid_trials_do_not_complete_phase_a():
    recs = phase_a_records(6)[:5] + [rec('s1_utrap', 'C4', 'INVALID_HOST_LOAD')] + phase_a_records(6)[6:]
    assert 'not complete' in an.s3_refusal(recs)


def test_phase_a_status_counts():
    st = an.phase_a_status(phase_a_records(4))
    assert st == {'c4_valid': 6, 'c3_valid': 3, 'c4_reached': 4, 'complete': True, 'gate_fired': False}


def test_block_b_and_every_s3_trial_are_refused_by_the_driver(monkeypatch, tmp_path):
    monkeypatch.setattr(an, 'registered_records', lambda raw: [])
    s3 = plan.Trial('s3_ctrap', 'C4', 0, 'B')
    s1_b = plan.Trial('s1_utrap', 'C1', 0, 'B')
    s1_a = plan.Trial('s1_utrap', 'C4', 0, 'A')
    assert 'not complete' in drv.phase_gate_refusal(str(tmp_path), 'B')
    assert 'not complete' in drv.phase_gate_refusal(str(tmp_path), s3)
    assert 'not complete' in drv.phase_gate_refusal(str(tmp_path), s1_b)
    s3_in_a = plan.Trial('s3_ctrap', 'C4', 0, 'A')                  # not in the plan, but s3 either way
    assert 'not complete' in drv.phase_gate_refusal(str(tmp_path), s3_in_a)
    assert drv.phase_gate_refusal(str(tmp_path), s1_a) is None
    assert drv.phase_gate_refusal(str(tmp_path), 'A') is None
    assert drv.phase_gate_refusal(str(tmp_path), 'all') is None
    monkeypatch.setattr(an, 'registered_records', lambda raw: phase_a_records(0))
    assert 'gate fired' in drv.phase_gate_refusal(str(tmp_path), 'B')
    monkeypatch.setattr(an, 'registered_records', lambda raw: phase_a_records(5))
    assert drv.phase_gate_refusal(str(tmp_path), 'B') is None
    assert drv.phase_gate_refusal(str(tmp_path), s3) is None


def test_trial_refusal_skips_nothing_that_is_complete_and_refuses_the_rest(monkeypatch, tmp_path):
    monkeypatch.setattr(an, 'registered_records', lambda raw: [])
    s3 = plan.Trial('s3_ctrap', 'C4', 0, 'B')
    out_root = str(tmp_path / 'trials')
    assert 'not complete' in drv.trial_refusal(str(tmp_path), out_root, s3, IDENT)
    d = tmp_path / 'trials' / s3.trial_id
    d.mkdir(parents=True)
    (d / 'meta.json').write_text(json.dumps(dict(IDENT, complete=True, active=True)))
    assert drv.trial_refusal(str(tmp_path), out_root, s3, IDENT) is None      # already done: skipped
    (d / 'meta.json').write_text(json.dumps(dict(IDENT, complete=False)))
    assert 'not complete' in drv.trial_refusal(str(tmp_path), out_root, s3, IDENT)
    a = plan.Trial('s1_utrap', 'C4', 0, 'A')
    assert drv.trial_refusal(str(tmp_path), out_root, a, IDENT) is None


def test_interrupted_wins_over_complete_in_existing_state(tmp_path):
    d = tmp_path / 't'
    d.mkdir()
    (d / 'meta.json').write_text(json.dumps(dict(IDENT, complete=True, interrupted=True, active=True)))
    assert drv.existing_state(str(d), IDENT) == 'incomplete'


def test_main_refuses_block_b_before_phase_a(monkeypatch, capsys, tmp_path):
    import freeze_manifest
    monkeypatch.setattr(freeze_manifest, 'check', lambda *a, **k: [])
    monkeypatch.setattr(sys, 'argv', ['run_campaign2.py', '--block', 'B', '--raw', str(tmp_path)])
    assert drv.main() == drv.EXIT_REFUSED
    out = capsys.readouterr().out
    assert 'REFUSED block B' in out and 'Phase A is not complete' in out


def test_main_does_not_refuse_block_a_at_the_gate(monkeypatch, capsys, tmp_path):
    """Block A is the gate itself; the refusal text must not appear for it."""
    import freeze_manifest
    monkeypatch.setattr(freeze_manifest, 'check', lambda *a, **k: [])
    monkeypatch.setattr(sys, 'argv', ['run_campaign2.py', '--block', 'A', '--raw', str(tmp_path)])
    try:
        drv.main()
    except BaseException:                                  # noqa: BLE001  (no ROS in the test)
        pass
    assert 'REFUSED block' not in capsys.readouterr().out


# --------------------------------------------------------------------------- #
# Interrupt, resume and foreign attempts (run_trial with a fake stack)
# --------------------------------------------------------------------------- #
class FakeProc:
    behaviour = 'ok'

    def __init__(self, args, **kw):
        self.args = args
        self.pid = 4242
        self.is_launch = args and args[0] != 'python3' and 'ros_gz_bridge' not in ' '.join(map(str, args))
        if self.is_launch and FakeProc.behaviour == 'interrupt_setup':
            raise KeyboardInterrupt()

    def poll(self):
        return None

    def wait(self, timeout=None):
        if FakeProc.behaviour == 'interrupt_drive':
            raise KeyboardInterrupt()
        return 0


def fake_stack(monkeypatch):
    class Sampler:
        def __init__(self, *a):
            pass

        def start(self):
            pass

        def stop(self):
            pass
    hostload = types.SimpleNamespace(
        wait_for_quiet=lambda log=None: {'load1_at_start': 0.1, 'load_gate_waited_s': 0.0},
        Sampler=Sampler, top_consumers=lambda: [], read_rows=lambda p: [],
        verdict=lambda rows, a, b: {'valid': True})
    rl = types.SimpleNamespace(
        launch_env=lambda: {}, build_launch_argv=lambda *a, **k: ['ros2-launch-fake'],
        launch_spec_for=lambda s: None, drive_env=lambda *a, **k: {},
        RosLauncher=lambda: types.SimpleNamespace(smoke_drive='smoke_drive.py'))
    rr = types.SimpleNamespace(
        hostload=hostload, rl=rl, TAG_VAR='SE_TAG', ROS_DOMAIN_ID='7', SETUP_RETRIES=0,
        SETUP_TIMEOUT_S=5, WATCHDOG_S=10, GT_GZ_TOPIC='/x', stack_is_active=lambda text: True,
        stop_group=lambda *a, **k: None, sweep_tag=lambda tag: [])
    monkeypatch.setattr(drv.subprocess, 'Popen', FakeProc)
    monkeypatch.setattr(drv.time, 'sleep', lambda s: None)
    monkeypatch.setattr(drv, 'build_params', lambda *a, **k: {'k': 1})
    return rr


SCEN = types.SimpleNamespace(name='s0_open', world=None, start=(-3.0, 0.0, 0.0), goal=(3.0, 0.0, 0.0))
IDENT = {'freeze_manifest_sha256': 'aa', 'controller_tag_commit': 'cc'}
TRIAL = plan.Trial('s0_open', 'C2', 0, 'A')


def run(rr, out_root, ident=IDENT):
    return drv.run_trial(rr, None, None, None, SCEN, TRIAL, str(out_root), {}, probe=False,
                         ident=ident, prov={'git_describe': 'campaign2-controller'})


def read_meta(path):
    with open(os.path.join(path, 'meta.json')) as f:
        return json.load(f)


def test_a_normal_run_is_complete_and_carries_the_identity_and_provenance(monkeypatch, tmp_path):
    FakeProc.behaviour = 'ok'
    rr = fake_stack(monkeypatch)
    meta = run(rr, tmp_path)
    d = str(tmp_path / TRIAL.trial_id)
    assert meta['complete'] is True and 'interrupted' not in meta
    on_disk = read_meta(d)
    assert on_disk['complete'] is True and 'drive_wall_end' in on_disk
    assert on_disk['freeze_manifest_sha256'] == 'aa' and on_disk['controller_tag_commit'] == 'cc'
    assert on_disk['provenance'] == {'git_describe': 'campaign2-controller'}
    assert drv.existing_state(d, IDENT) == 'complete_valid'
    assert run(rr, tmp_path) is None                                  # resume skips it


@pytest.mark.parametrize('where', ['interrupt_drive', 'interrupt_setup'])
def test_ctrl_c_leaves_an_interrupted_trial_that_resume_repeats(monkeypatch, tmp_path, where):
    rr = fake_stack(monkeypatch)
    FakeProc.behaviour = where
    with pytest.raises(KeyboardInterrupt):
        run(rr, tmp_path)
    d = str(tmp_path / TRIAL.trial_id)
    meta = read_meta(d)
    assert meta['interrupted'] is True and meta['complete'] is False
    assert 'drive_wall_end' not in meta and 'host_load' not in meta
    assert drv.existing_state(d, IDENT) == 'incomplete'
    # the analyzer reports it as INVALID (not SETUP_FAIL, no KeyError)
    r = an.analyze_trial(d, [], (0.5, 0.3))
    assert r['outcome'] == 'INVALID' and 'interrupted' in r['invalid_reason']
    assert not any(c == 'SETUP_FAIL' for c, _ in an.evaluate_stops([r]))
    # resume: the directory is kept as .aborted1 and the trial runs again
    FakeProc.behaviour = 'ok'
    meta2 = run(rr, tmp_path)
    assert meta2['complete'] is True
    assert os.path.isdir(d + '.aborted1') and read_meta(d + '.aborted1')['interrupted'] is True
    assert drv.existing_state(d, IDENT) == 'complete_valid'


def test_a_stack_that_never_becomes_active_is_a_complete_setup_failure_not_an_interruption(
        monkeypatch, tmp_path):
    FakeProc.behaviour = 'ok'
    rr = fake_stack(monkeypatch)
    rr.stack_is_active = lambda text: False                # the launch never reports active
    monkeypatch.setattr(FakeProc, 'poll', lambda self: 1)  # and the launch process has exited
    meta = run(rr, tmp_path)
    assert meta['complete'] is True and meta['active'] is False and 'interrupted' not in meta
    d = str(tmp_path / TRIAL.trial_id)
    assert drv.existing_state(d, IDENT) == 'complete_valid'
    assert an.analyze_trial(d, [], (0.5, 0.3))['outcome'] == 'SETUP_FAIL'


def test_the_attempt_number_selects_the_raw_root_in_main(monkeypatch, capsys):
    import freeze_manifest
    monkeypatch.setattr(freeze_manifest, 'check', lambda *a, **k: [])
    seen = []
    monkeypatch.setattr(drv, 'phase_gate_refusal', lambda raw, what: seen.append(raw) or 'stop here')
    for argv, expect in ((['--attempt', '2'], plan.raw_root(2)), ([], plan.raw_root(1)),
                         (['--attempt', '3', '--raw', '/somewhere'], '/somewhere')):
        seen.clear()
        monkeypatch.setattr(sys, 'argv', ['run_campaign2.py', '--block', 'B'] + argv)
        assert drv.main() == drv.EXIT_REFUSED
        assert seen == [expect], (argv, seen)


def test_the_analysis_record_carries_the_identity_of_the_attempt(tmp_path):
    d = make_trial(str(tmp_path), 'correct__s0_open__C2__rep0', 's0_open', 'C2',
                   [(0.0, -3.0, 0.0, 0.0), (5.0, 3.0, 0.0, 0.0)])
    meta = read_meta(d)
    meta.update(IDENT)
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    r = an.analyze_trial(d, an.world_boxes(an.WORLDS['s0_open']), an.pilot.chassis_size())
    assert (r['freeze_manifest_sha256'], r['controller_tag_commit']) == ('aa', 'cc')


def test_a_directory_of_another_attempt_is_foreign_and_refused(monkeypatch, tmp_path):
    FakeProc.behaviour = 'ok'
    rr = fake_stack(monkeypatch)
    run(rr, tmp_path, ident=IDENT)
    other = dict(IDENT, freeze_manifest_sha256='zz')
    d = str(tmp_path / TRIAL.trial_id)
    assert drv.existing_state(d, other) == 'foreign'
    with pytest.raises(SystemExit, match='another attempt'):
        run(rr, tmp_path, ident=other)
    assert drv.foreign_trials(str(tmp_path), other) == [TRIAL.trial_id]
    assert drv.foreign_trials(str(tmp_path), IDENT) == []
    assert drv.foreign_trials(str(tmp_path), None) == []
    # superseded directories of attempt 1 are foreign too
    os.rename(d, d + '.invalid1')
    assert drv.foreign_trials(str(tmp_path), other) == [TRIAL.trial_id + '.invalid1']


def test_main_refuses_a_raw_root_with_trials_of_another_attempt(monkeypatch, capsys, tmp_path):
    import freeze_manifest
    monkeypatch.setattr(freeze_manifest, 'check', lambda *a, **k: [])
    monkeypatch.setattr(sys, 'argv', ['run_campaign2.py', '--block', 'A', '--raw', str(tmp_path)])
    ident = {'freeze_manifest_sha256': 'aa', 'controller_tag_commit': 'cc'}
    monkeypatch.setattr(pv, 'identity', lambda *a, **k: ident)
    d = tmp_path / 'trials' / 'correct__s1_utrap__C4__rep0'
    d.mkdir(parents=True)
    (d / 'meta.json').write_text(json.dumps(dict(ident, controller_tag_commit='old', complete=True)))
    assert drv.main() == drv.EXIT_REFUSED
    out = capsys.readouterr().out
    assert 'another attempt' in out and 'correct__s1_utrap__C4__rep0' in out and '--attempt 2' in out


def test_attempt_two_has_its_own_raw_root():
    assert plan.raw_root(1) == plan.RAW_ROOT
    assert plan.raw_root(2) == plan.RAW_ROOT + '/attempt2'
    assert plan.raw_root(3) == plan.RAW_ROOT + '/attempt3'
    with pytest.raises(ValueError):
        plan.raw_root(0)
    assert not plan.raw_root(2).startswith(plan.raw_root(1) + '/trials')


# --------------------------------------------------------------------------- #
# The analyzer on interrupted or truncated metas
# --------------------------------------------------------------------------- #
def test_complete_meta_without_a_drive_interval_is_invalid_not_a_key_error(tmp_path):
    root = str(tmp_path)
    d = make_trial(root, 'correct__s0_open__C2__rep0', 's0_open', 'C2',
                   [(0.0, -3.0, 0.0, 0.0), (5.0, 3.0, 0.0, 0.0)])
    meta = read_meta(d)
    del meta['drive_wall_start'], meta['drive_wall_end']
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    r = an.analyze_trial(d, an.world_boxes(an.WORLDS['s0_open']), an.pilot.chassis_size())
    assert r['outcome'] == 'INVALID' and 'no drive interval' in r['invalid_reason']


def test_a_trial_interrupted_during_the_drive_does_not_crash_the_stop_check(tmp_path):
    root = str(tmp_path / 'raw')
    d = make_trial(root, 'correct__s1_utrap__C4__rep0', 's1_utrap', 'C4',
                   [(0.0, -3.0, 0.0, 0.0), (5.0, -2.0, 0.0, 0.0)], complete=False)
    meta = read_meta(d)
    meta['interrupted'] = True
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    cells = sorted({t.cell for t in plan.plan('all')})
    assert drv.check_stops(root, cells) == []
