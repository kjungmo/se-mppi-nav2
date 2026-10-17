"""Tests of the campaign-2 driver that need no ROS: the plan listing and the resume logic."""

import json
import os
import subprocess
import sys

import pytest

import plan
import run_campaign2 as drv

DRIVER = os.path.join(plan.HERE, 'run_campaign2.py')


def run_list(*args):
    return subprocess.run([sys.executable, DRIVER, '--list', *args], capture_output=True, text=True,
                          check=True).stdout.splitlines()


def test_list_prints_the_76_trial_plan():
    lines = run_list()
    assert lines[-1] == '76 trials'
    rows = lines[:-1]
    assert len(rows) == 76
    assert rows[0].split()[:3] == ['1', 'A', 'correct__s1_utrap__C4__rep0']
    assert sum(r.split()[1] == 'A' for r in rows) == 9
    assert sum(r.split()[1] == 'B' for r in rows) == 67
    assert len({r.split()[2] for r in rows}) == 76


def test_list_of_blocks():
    assert run_list('--block', 'A')[-1] == '9 trials'
    assert run_list('--block', 'B')[-1] == '67 trials'
    sd = run_list('--block', 'shakedown')
    assert sd[-1] == '1 trials' and sd[0].split()[1:] == ['shakedown', 'correct__s0_open__C2__rep0']


def test_list_does_not_need_ros():
    env = {k: v for k, v in os.environ.items() if k not in ('AMENT_PREFIX_PATH',)}
    out = subprocess.run([sys.executable, DRIVER, '--list'], capture_output=True, text=True, env=env)
    assert out.returncode == 0


def write_meta(path, **kw):
    os.makedirs(path, exist_ok=True)
    with open(os.path.join(path, 'meta.json'), 'w') as f:
        json.dump(kw, f)


def test_existing_state_decides_what_resume_does(tmp_path):
    p = str(tmp_path / 'x')
    assert drv.existing_state(p) == 'absent'
    os.makedirs(p)
    assert drv.existing_state(p) == 'incomplete'                      # directory without meta.json
    write_meta(p, complete=False)
    assert drv.existing_state(p) == 'incomplete'
    with open(os.path.join(p, 'meta.json'), 'w') as f:
        f.write('{"complete": tr')                                    # killed while writing
    assert drv.existing_state(p) == 'incomplete'
    write_meta(p, complete=True, active=True, host_load={'valid': True})
    assert drv.existing_state(p) == 'complete_valid'
    write_meta(p, complete=True, active=True, host_load={'valid': False, 'reason': 'x'})
    assert drv.existing_state(p) == 'complete_invalid'
    write_meta(p, complete=True, active=False)                        # setup failure: no verdict
    assert drv.existing_state(p) == 'complete_valid'


def test_next_free_never_reuses_a_name(tmp_path):
    base = str(tmp_path / 't')
    assert drv.next_free(base, 'invalid') == base + '.invalid1'
    os.makedirs(base + '.invalid1')
    os.makedirs(base + '.invalid2')
    assert drv.next_free(base, 'invalid') == base + '.invalid3'
    assert drv.next_free(base, 'aborted') == base + '.aborted1'


def test_resume_picks_the_first_unfinished_trial_in_plan_order(tmp_path):
    """Skip rule of the driver loop: complete+valid directories are skipped, anything else runs."""
    done = plan.plan('all')[:11]
    for t in done:
        write_meta(str(tmp_path / t.trial_id), complete=True, active=True, host_load={'valid': True})
    # the 12th was interrupted
    write_meta(str(tmp_path / plan.plan('all')[11].trial_id), complete=False)
    pending = [t for t in plan.plan('all')
               if drv.existing_state(str(tmp_path / t.trial_id)) != 'complete_valid']
    assert pending[0] == plan.plan('all')[11]
    assert len(pending) == 76 - 11


def test_unsafe_configuration_is_refused_before_any_launch():
    import yaml
    with open(plan.PARAMS_YAML) as f:
        tree = yaml.safe_load(f)
    tree['controller_server']['ros__parameters']['FollowPath']['vx_min'] = 0.1
    with pytest.raises(SystemExit, match='vx_min = 0.1'):
        plan.refuse_unsafe(tree)
