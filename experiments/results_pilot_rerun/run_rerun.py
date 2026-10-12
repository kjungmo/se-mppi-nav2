#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Driver of the live re-run on a verified nav2_mppi_controller (PREREG.md).

One trial = launch the U-trap testbed with Nav2 (the same ``ros2 launch`` line
the evaluation harness builds), wait for the stack to be active, start the
recorder, send the goal with ``experiments/sim/smoke_drive.py``, tear down.

It reuses the harness's pure helpers (parameter overlay, launch line, AMCL
seeding, software-rendering environment) and replaces two things:

* Clean-up. ``experiments/runner/cleanup.py`` runs ``pkill -f nav2`` and
  similar patterns, which also match unrelated processes whose command line
  contains the repository path. Here every process of a trial is started with a
  unique environment tag and stopped by process group, then by PID for any
  process that still carries the tag. Nothing else on the host is touched.
* Outcome. The harness compares odometry-frame positions with the map-frame
  goal and has no obstacle list, so its ``reached_goal`` and ``collided``
  fields are not usable for the U-trap scenes. Outcomes are computed by
  ``analyze.py`` from the simulator's ground-truth pose.

Run from the repository root inside an activated ROS 2 environment whose
workspace (nav2_se_controller, se_mppi_utrap) is sourced:

    python3 experiments/results_pilot_rerun/run_rerun.py --block main \
        --env-label correct
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import signal
import subprocess
import sys
import time
import uuid

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, REPO)

from experiments.runner import config as config_mod  # noqa: E402
from experiments.runner import ros_launcher as rl  # noqa: E402
from experiments.runner import scenario as scenario_mod  # noqa: E402

SETUP_TIMEOUT_S = 120.0      # experiments/configs/suite.yaml
WATCHDOG_S = 300.0           # experiments/configs/suite.yaml
SETUP_RETRIES = 1            # experiments/configs/suite.yaml
ROS_DOMAIN_ID = '87'
TAG_VAR = 'SE_RERUN_TAG'
LOAD_GATE = 6.0              # 1-minute load average (12 logical CPUs)
LOAD_GATE_WAIT_S = 1200.0
GT_GZ_TOPIC = '/world/testbed/dynamic_pose/info'
FP = 'controller_server.ros__parameters.FollowPath'

# Parameter sets (PREREG.md, "Parameter sets"). Paths are dotted keys of the
# resolved Nav2 parameter tree.
PARAM_SETS = {
    # The file the July pilot used, unchanged.
    'P': {},
    # P with the one MPPI limit the June tuning changed in direct answer to the
    # reversing symptom ("forbid reverse"): back to the value of the first
    # committed parameter file, which is also the released default.
    'V': {f'{FP}.vx_min': -0.35},
    # The MPPI block, controller rate and inflation of the first committed
    # parameter file (history commit f83039c, 2026-06-08), before any live
    # tuning. Everything else (AMCL, behaviour tree, timeouts, transform
    # tolerances, costmap layers, EscapeCritic) stays as in P.
    'O': {
        'controller_server.ros__parameters.controller_frequency': 20.0,
        f'{FP}.time_steps': 56,
        f'{FP}.model_dt': 0.05,
        f'{FP}.batch_size': 2000,
        f'{FP}.vx_min': -0.35,
        f'{FP}.GoalAngleCritic.threshold_to_consider': 0.5,
        f'{FP}.PathAngleCritic.threshold_to_consider': 0.5,
        'local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius': 0.70,
        'global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius': 0.70,
    },
}

SCENARIOS = ('s1_utrap', 's1_utrap_evidence')
CONFIGS = ('F_se_full', 'A_stock')


def plan(block: str) -> list:
    """The registered trial list of a block, in run order (PREREG.md)."""
    cells = [(s, c) for s in SCENARIOS for c in CONFIGS]
    out = []
    if block == 'main':
        for pset, reps in (('P', 5), ('V', 3), ('O', 3)):
            for rep in range(reps):
                # Rotate the cell order with the repetition so that no cell
                # always runs first after a pause.
                order = cells[rep % 4:] + cells[:rep % 4]
                out += [(s, c, pset, rep) for s, c in order]
    elif block == 'affected':
        out = [(s, c, 'P', 0) for s, c in cells]
    elif block == 'shakedown':
        out = [('s1_utrap', 'F_se_full', 'P', 0)]
    else:
        raise ValueError(block)
    return out


def stack_is_active(log_text: str) -> bool:
    """True when the NAVIGATION lifecycle manager reports its nodes active.

    The harness's ``log_is_active`` accepts the localization manager's line as
    well, so it reports a stack as active whose controller_server failed to
    configure (seen in the first shakedown run).
    """
    return any('lifecycle_manager_navigation]' in ln and 'Managed nodes are active' in ln
               for ln in log_text.splitlines())


def wait_for_load(limit: float = LOAD_GATE, max_wait_s: float = LOAD_GATE_WAIT_S) -> dict:
    """Wait until the 1-minute load average is below ``limit`` (PREREG.md,
    amendment 3). Starts anyway after ``max_wait_s`` and says so."""
    t0 = time.time()
    while True:
        with open('/proc/loadavg') as f:
            load1 = float(f.read().split()[0])
        waited = time.time() - t0
        if load1 < limit or waited >= max_wait_s:
            return {'load1_at_start': load1, 'load_gate_waited_s': round(waited, 1),
                    'load_gate_passed': load1 < limit}
        time.sleep(15.0)


def trial_id(env_label, scenario, cfg, pset, rep):
    return f'{env_label}__{scenario}__{cfg}__{pset}__rep{rep}'


# --------------------------------------------------------------------------- #
# Process handling: by process group and by environment tag, never by pattern.
# --------------------------------------------------------------------------- #
def tagged_pids(tag: str) -> list:
    """PIDs of this user's processes whose environment carries ``tag``."""
    needle = f'{TAG_VAR}={tag}'.encode()
    me = os.getpid()
    out = []
    for path in glob.glob('/proc/[0-9]*/environ'):
        pid = int(path.split('/')[2])
        if pid == me:
            continue
        try:
            with open(path, 'rb') as f:
                if needle in f.read().split(b'\0'):
                    out.append(pid)
        except OSError:
            continue
    return out


def stop_group(proc, grace: float = 10.0) -> None:
    """SIGINT, SIGTERM, SIGKILL the process group ``proc`` leads."""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except (ProcessLookupError, OSError):
        return
    # The leader can exit before its children do, so every signal goes to the
    # whole group; a group that is already gone ends the sequence.
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
        try:
            os.killpg(pgid, sig)
        except (ProcessLookupError, OSError):
            return
        deadline = time.time() + wait
        while time.time() < deadline and proc.poll() is None:
            time.sleep(0.5)
        time.sleep(1.0)


def sweep_tag(tag: str) -> list:
    """Kill, by PID, whatever still carries the tag. Returns the PIDs killed."""
    killed = []
    for sig in (signal.SIGTERM, signal.SIGKILL):
        pids = tagged_pids(tag)
        if not pids:
            break
        for pid in pids:
            try:
                os.kill(pid, sig)
                killed.append(pid)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(3.0)
    return sorted(set(killed))


# --------------------------------------------------------------------------- #
def build_params(suite, scenario, cfg, pset, override_guard):
    params = config_mod.resolve_params(suite, cfg)
    for path, value in PARAM_SETS[pset].items():
        config_mod.get_by_path(params, path)       # the key must already exist
        config_mod.set_by_path(params, path, value)
    if override_guard:
        config_mod.set_by_path(params, f'{FP}.se_enforce_rollout_check', False)
    return rl.apply_scenario_localization(params, scenario)


def run_trial(suite, scenario, cfg, pset, rep, env_label, out_root,
              override_guard, env_info):
    tid = trial_id(env_label, scenario.name, cfg, pset, rep)
    out = os.path.join(out_root, tid)
    meta_path = os.path.join(out, 'meta.json')
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            if json.load(f).get('complete'):
                print(f'[rerun] skip (complete): {tid}', flush=True)
                return
        k = 1
        while os.path.exists(f'{out}.aborted{k}'):
            k += 1
        os.rename(out, f'{out}.aborted{k}')    # kept and reported, never deleted
    os.makedirs(out)

    params = build_params(suite, scenario, cfg, pset, override_guard)
    params_file = os.path.join(out, 'params.yaml')
    with open(params_file, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)

    tag = uuid.uuid4().hex
    env = rl.launch_env()
    env.update({
        'ROS_DOMAIN_ID': ROS_DOMAIN_ID,
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'GZ_PARTITION': f'se_rerun_{tag[:12]}',
        TAG_VAR: tag,
        'TURTLEBOT3_MODEL': 'waffle',
    })
    argv = rl.build_launch_argv(params_file, headless=True, use_rviz=False,
                                spec=rl.launch_spec_for(scenario), scenario=scenario)
    meta = {
        'trial_id': tid, 'env_label': env_label, 'scenario': scenario.name,
        'config': cfg, 'param_set': pset, 'rep': rep,
        'override_guard': override_guard, 'launch_argv': argv,
        'start': list(scenario.start), 'goal': list(scenario.goal),
        'watchdog_s': WATCHDOG_S, 'setup_timeout_s': SETUP_TIMEOUT_S,
        'ros_domain_id': ROS_DOMAIN_ID, 'env': env_info,
        'wall_start': time.time(), 'complete': False, 'setup_attempts': 0,
        'active': False,
    }

    def save():
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
    save()
    meta.update(wait_for_load())
    meta['wall_start'] = time.time()
    save()

    launch = None
    extras = []
    try:
        for attempt in range(1, SETUP_RETRIES + 2):
            meta['setup_attempts'] = attempt
            log_path = os.path.join(out, 'launch.log' if attempt == 1
                                    else f'launch_attempt{attempt}.log')
            log_file = open(log_path, 'wb')
            launch = subprocess.Popen(argv, stdout=log_file, stderr=subprocess.STDOUT,
                                      env=env, start_new_session=True)
            deadline = time.time() + SETUP_TIMEOUT_S
            active = False
            while time.time() < deadline and launch.poll() is None:
                with open(log_path, errors='replace') as f:
                    text = f.read()
                if 'Aborting bringup' in text:
                    break
                if stack_is_active(text):
                    active = True
                    break
                time.sleep(1.0)
            if active:
                meta['active'] = True
                meta['launch_log'] = os.path.basename(log_path)
                break
            stop_group(launch)
            log_file.close()
            sweep_tag(tag)
            launch = None
        save()
        if not meta['active']:
            return

        bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             f'{GT_GZ_TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            stdout=open(os.path.join(out, 'gt_bridge.log'), 'wb'),
            stderr=subprocess.STDOUT, env=env, start_new_session=True)
        recorder = subprocess.Popen(
            ['python3', os.path.join(HERE, 'recorder.py'),
             os.path.join(out, 'telemetry.csv'), os.path.join(out, 'diagnostics.jsonl')],
            stdout=open(os.path.join(out, 'recorder.log'), 'wb'),
            stderr=subprocess.STDOUT, env=env, start_new_session=True)
        extras = [recorder, bridge]
        time.sleep(3.0)

        denv = rl.drive_env(scenario, base_env=env, localizer='amcl',
                            drive_timeout=WATCHDOG_S)
        meta['drive_wall_start'] = time.time()
        with open(os.path.join(out, 'drive.log'), 'w') as dlog:
            drive = subprocess.Popen(['python3', rl.RosLauncher().smoke_drive],
                                     stdout=dlog, stderr=subprocess.STDOUT,
                                     env=denv, start_new_session=True)
            extras.append(drive)
            try:
                meta['drive_returncode'] = drive.wait(timeout=WATCHDOG_S + 60.0)
            except subprocess.TimeoutExpired:
                meta['drive_returncode'] = None
                meta['drive_killed_by_driver'] = True
        meta['drive_wall_end'] = time.time()
    finally:
        for p in extras:
            stop_group(p, grace=5.0)
        stop_group(launch)
        meta['swept_pids'] = sweep_tag(tag)
        meta['wall_end'] = time.time()
        meta['complete'] = True
        save()
    print(f'[rerun] done: {tid} active={meta["active"]} '
          f'drive_rc={meta.get("drive_returncode")}', flush=True)


def environment_info() -> dict:
    """What this trial ran on: package builds, commit, guard verdict."""
    prefix = os.environ.get('CONDA_PREFIX', '')
    builds = sorted(
        os.path.basename(p)[:-5] for pat in ('*nav2-mppi-controller*', 'ros2-distro-mutex*',
                                             'xtensor-*', 'xsimd-*', '*gz-sim8-*',
                                             'libgz-sim8-*', '*ros-gz-sim-[0-9]*')
        for p in glob.glob(os.path.join(prefix, 'conda-meta', pat + '.json')))
    def sh(cmd):
        r = subprocess.run(cmd, capture_output=True, text=True)
        return (r.stdout + r.stderr).strip(), r.returncode
    check, rc = sh(['ros2', 'run', 'nav2_se_controller', 'check_mppi_rollout'])
    commit, _ = sh(['git', '-C', REPO, 'rev-parse', 'HEAD'])
    dirty, _ = sh(['git', '-C', REPO, 'status', '--porcelain', '--', 'src', 'experiments/sim',
                   'experiments/se_mppi_utrap', 'experiments/barn', 'experiments/configs',
                   'experiments/runner'])
    with open('/proc/cpuinfo') as f:
        cpu = next((ln.split(':', 1)[1].strip() for ln in f if ln.startswith('model name')), '')
    return {
        'conda_env': os.path.basename(prefix), 'package_builds': builds,
        'check_mppi_rollout': check[:400], 'check_mppi_rollout_exit': rc,
        'repo_commit': commit, 'tracked_inputs_modified': dirty,
        'kernel': os.uname().release, 'cpu': cpu, 'cpus': os.cpu_count(),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--block', required=True, choices=('main', 'affected', 'shakedown'))
    ap.add_argument('--env-label', required=True, choices=('correct', 'affected'))
    ap.add_argument('--out', default=os.path.join(HERE, 'trials'))
    ap.add_argument('--override-guard', action='store_true',
                    help='set se_enforce_rollout_check: false (affected library only)')
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args()

    trials = plan(args.block)
    if args.list:
        for i, (s, c, p, r) in enumerate(trials, 1):
            print(f'{i:2d} {trial_id(args.env_label, s, c, p, r)}')
        return 0
    if not rl.ros_available():
        raise SystemExit('no ros2 on PATH: activate the environment and source the workspace')

    info = environment_info()
    print(f'[rerun] environment: {json.dumps(info, indent=1)}', flush=True)
    expect_ok = args.env_label == 'correct'
    if expect_ok != (info['check_mppi_rollout_exit'] == 0):
        raise SystemExit(f'--env-label {args.env_label} does not match check_mppi_rollout '
                         f'(exit {info["check_mppi_rollout_exit"]})')
    if expect_ok and args.override_guard:
        raise SystemExit('--override-guard is only for the affected library')

    suite = config_mod.load_ablations(os.path.join(REPO, 'experiments', 'configs',
                                                   'ablations.yaml'))
    scenarios = {s.name: s for s in scenario_mod.discover(
        os.path.join(REPO, 'experiments'), tier='barn')}
    out_root = args.out if args.block != 'shakedown' else args.out + '_shakedown'
    os.makedirs(out_root, exist_ok=True)
    for s, c, p, r in trials:
        run_trial(suite, scenarios[s], c, p, r, args.env_label, out_root,
                  args.override_guard, info)
        time.sleep(5.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
