#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Driver of campaign 2 (PREREG.md), derived from results_pilot_rerun/run_rerun.py.

One trial = launch the testbed with Nav2 (the same ``ros2 launch`` line the campaign-1
driver builds, plus the scenario's world file), wait for the stack to be active, start the
recorder, send the goal with ``experiments/sim/smoke_drive.py``, tear down. Unchanged from
campaign 1: the host admission gate and the per-trial validity rule (``hostload.py``,
amendments 5 and 8), process clean-up by environment tag (never ``pkill -f``), the
setup timeout, the watchdog, ground-truth outcomes computed by ``analyze.py``.

New in campaign 2:

* Configurations C1-C4 (``plan.py``): ablation (A_stock or F_se_full) x behaviour tree x
  progress-checker allowance, on top of ``experiments/sim/nav2_se_campaign2.yaml``.
* The plan: Phase A (gate, 9 trials) and Phase B (67 trials), cell order rotating with the
  repetition; ``--list`` prints it.
* Resume at the first unfinished trial: finished trials are skipped, an unfinished
  directory (including one interrupted by Ctrl-C or an exception, which carries
  ``interrupted`` and never ``complete``) is kept as ``.abortedN`` and the trial is run again,
  a finished trial whose host-load verdict is invalid is kept as ``.invalidN`` and repeated.
  A directory whose ``meta.json`` carries another freeze-manifest hash or controller tag commit
  is foreign: the driver refuses to run. A restart is attempt 2 with its own raw root
  (``--attempt 2``).
* Stopping rules after every trial (``analyze.evaluate_stops``): the Phase-A gate, a stock
  controller at or below 4 of 6 on s0_open, more than 6 invalid attempts on a cell, a stack
  that does not become active.
* Raw data go to ``/mnt/d/wsl/se-mppi/campaign2/`` (``trials/``, ``shakedown/``).
* Refusal to run ``se_enabled`` with ``vx_min > 0`` (DESIGN_ADVICE D1), and refusal to run a
  counted block unless ``freeze_manifest.py --check`` passes.
* Provenance before every counted trial (``provenance.py``): the installed behaviour trees,
  worlds, launch and URDF files equal the repository (and the manifest), the installed
  controller libraries are hashed into ``meta.json``, the build stamp names the tag
  ``campaign2-controller`` at the tag commit, and the repository is at the tag with no
  uncommitted change to a tracked file (the analysis outputs excepted).
* Block B, and every ``s3_ctrap`` trial, is refused until Phase A is complete and its gate has
  not fired (``analyze.s3_refusal``), so s3 stays unseen if the second design iteration is needed.

Run from the repository root inside the ``semppi_rerun`` environment with the workspace
(nav2_se_controller, se_mppi_utrap) sourced:

    python3 experiments/campaign2/run_campaign2.py --list
    python3 experiments/campaign2/run_campaign2.py --block shakedown
    python3 experiments/campaign2/run_campaign2.py --block A
    python3 experiments/campaign2/run_campaign2.py --block all          # resumes
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
PILOT_DIR = os.path.join(REPO, 'experiments', 'results_pilot_rerun')
sys.path.insert(0, REPO)
sys.path.insert(0, PILOT_DIR)
sys.path.insert(0, HERE)

import plan as plan_mod  # noqa: E402
import provenance  # noqa: E402

EXIT_REFUSED = 2
EXIT_STOP = 3


def existing_state(out: str, ident: dict | None = None) -> str:
    """'absent', 'incomplete', 'foreign', 'complete_valid' or 'complete_invalid' for a trial directory.

    'incomplete': no meta.json, unreadable, ``interrupted``, or not ``complete``. 'foreign': with
    ``ident`` given, the directory carries another freeze-manifest hash or controller tag commit
    than this attempt (a trial of an earlier attempt must never be reused).
    """
    meta_path = os.path.join(out, 'meta.json')
    if not os.path.exists(meta_path):
        return 'absent' if not os.path.exists(out) else 'incomplete'
    try:
        with open(meta_path) as f:
            meta = json.load(f)
    except ValueError:
        return 'incomplete'
    if provenance.foreign_reason(meta, ident) is not None:
        return 'foreign'
    if meta.get('interrupted') or not meta.get('complete'):
        return 'incomplete'
    load = meta.get('host_load')
    if load is not None and not load.get('valid', True):
        return 'complete_invalid'
    return 'complete_valid'


def foreign_trials(trials_dir: str, ident: dict | None) -> list:
    """Trial directories (any suffix) of ``trials_dir`` that do not belong to this attempt."""
    if ident is None or not os.path.isdir(trials_dir):
        return []
    out = []
    for name in sorted(os.listdir(trials_dir)):
        d = os.path.join(trials_dir, name)
        if os.path.isdir(d) and existing_state(d, ident) == 'foreign':
            out.append(name)
    return out


def next_free(base: str, kind: str) -> str:
    k = 1
    while os.path.exists(f'{base}.{kind}{k}'):
        k += 1
    return f'{base}.{kind}{k}'


def build_params(rr, config_mod, suite, base_tree, scenario, config):
    import copy
    spec = plan_mod.CONFIGS[config]
    params = config_mod.resolve_params(suite, spec['ablation'], base_tree=copy.deepcopy(base_tree))
    for path, value in plan_mod.overlay(config).items():
        config_mod.get_by_path(params, path)           # the key must already exist
        config_mod.set_by_path(params, path, value)
    plan_mod.refuse_unsafe(params)
    return rr.rl.apply_scenario_localization(params, scenario)


def environment_info(rr) -> dict:
    info = rr.environment_info()

    def sh(cmd):
        r = subprocess.run(cmd, capture_output=True, text=True)
        return (r.stdout + r.stderr).strip()
    info['tracked_inputs_modified'] = sh(
        ['git', '-C', REPO, 'status', '--porcelain', '--', 'src', 'experiments/sim',
         'experiments/se_mppi_utrap', 'experiments/barn', 'experiments/configs',
         'experiments/runner', 'experiments/campaign2', 'experiments/results_pilot_rerun'])
    info['git_describe'] = sh(['git', '-C', REPO, 'describe', '--tags', '--always', '--dirty'])
    info['controller_prefix'] = sh(['ros2', 'pkg', 'prefix', 'nav2_se_controller'])
    info['testbed_prefix'] = sh(['ros2', 'pkg', 'prefix', 'se_mppi_utrap'])
    return info


def run_trial(rr, config_mod, suite, base_tree, scenario, trial, out_root, env_info, probe,
              ident=None, prov=None):
    hostload = rr.hostload
    rl = rr.rl
    out = os.path.join(out_root, trial.trial_id)
    state = existing_state(out, ident)
    if state == 'foreign':
        raise SystemExit(f'{out} belongs to another attempt (freeze manifest or tag commit differ): '
                         'give a restart its own raw root, --attempt 2')
    if state == 'complete_valid':
        print(f'[campaign2] skip (complete): {trial.trial_id}', flush=True)
        return None
    if state == 'complete_invalid':
        os.rename(out, next_free(out, 'invalid'))
    elif state == 'incomplete':
        os.rename(out, next_free(out, 'aborted'))      # kept and reported, never deleted
    os.makedirs(out, exist_ok=True)

    params = build_params(rr, config_mod, suite, base_tree, scenario, trial.config)
    params_file = os.path.join(out, 'params.yaml')
    import yaml
    with open(params_file, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)

    tag = uuid.uuid4().hex
    print(f'[campaign2] {trial.phase} {trial.trial_id}: waiting for a quiet host', flush=True)
    gate = hostload.wait_for_quiet(log=lambda m: print(m, flush=True))
    sampler = hostload.Sampler(os.path.join(out, 'host_cpu.csv'), rr.TAG_VAR, tag)
    sampler.start()
    env = rl.launch_env()
    env.update({
        'ROS_DOMAIN_ID': rr.ROS_DOMAIN_ID,
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'GZ_PARTITION': f'se_c2_{tag[:12]}',
        rr.TAG_VAR: tag,
        'TURTLEBOT3_MODEL': 'waffle',
    })
    argv = rl.build_launch_argv(params_file, headless=True, use_rviz=False,
                                spec=rl.launch_spec_for(scenario), scenario=scenario)
    if scenario.world:
        argv.append(f'world:={scenario.world}')
    spec = plan_mod.CONFIGS[trial.config]
    meta = {
        'trial_id': trial.trial_id, 'campaign': 2, 'phase': trial.phase,
        'env_label': plan_mod.ENV_LABEL, 'scenario': scenario.name, 'config': trial.config,
        'ablation': spec['ablation'], 'bt': spec['tree'], 'movement_time_allowance_s': spec['allowance_s'],
        'rep': trial.rep, 'counted': trial.counted, 'launch_argv': argv,
        'params_yaml_sha256': plan_mod.sha256_file(plan_mod.PARAMS_YAML),
        'bt_sha256': plan_mod.sha256_file(os.path.join(plan_mod.BT_DIR, spec['tree'])),
        'world': scenario.world,
        'world_sha256': plan_mod.sha256_file(scenario.world) if scenario.world else None,
        'start': list(scenario.start), 'goal': list(scenario.goal),
        'watchdog_s': rr.WATCHDOG_S, 'setup_timeout_s': rr.SETUP_TIMEOUT_S,
        'ros_domain_id': rr.ROS_DOMAIN_ID, 'env': env_info,
        'wall_start': time.time(), 'complete': False, 'setup_attempts': 0,
        'ncpu': os.cpu_count(), 'active': False,
    }
    meta.update(ident or {})
    meta['provenance'] = prov if prov is not None else 'not verified'
    meta_path = os.path.join(out, 'meta.json')

    def save():
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
    meta.update(gate)
    save()

    launch = None
    extras = []
    ok = False
    try:
        for attempt in range(1, rr.SETUP_RETRIES + 2):
            meta['setup_attempts'] = attempt
            log_path = os.path.join(out, 'launch.log' if attempt == 1 else f'launch_attempt{attempt}.log')
            log_file = open(log_path, 'wb')
            launch = subprocess.Popen(argv, stdout=log_file, stderr=subprocess.STDOUT,
                                      env=env, start_new_session=True)
            deadline = time.time() + rr.SETUP_TIMEOUT_S
            active = False
            while time.time() < deadline and launch.poll() is None:
                with open(log_path, errors='replace') as f:
                    text = f.read()
                if 'Aborting bringup' in text:
                    break
                if rr.stack_is_active(text):
                    active = True
                    break
                time.sleep(1.0)
            if active:
                meta['active'] = True
                meta['launch_log'] = os.path.basename(log_path)
                break
            rr.stop_group(launch)
            log_file.close()
            rr.sweep_tag(tag)
            launch = None
        save()
        if not meta['active']:
            ok = True
            return meta

        bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             f'{rr.GT_GZ_TOPIC}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            stdout=open(os.path.join(out, 'gt_bridge.log'), 'wb'),
            stderr=subprocess.STDOUT, env=env, start_new_session=True)
        recorder = subprocess.Popen(
            ['python3', os.path.join(HERE, 'recorder.py'),
             os.path.join(out, 'telemetry.csv'), os.path.join(out, 'diagnostics.jsonl'),
             os.path.join(out, 'escape_path.jsonl')],
            stdout=open(os.path.join(out, 'recorder.log'), 'wb'),
            stderr=subprocess.STDOUT, env=env, start_new_session=True)
        extras = [recorder, bridge]
        time.sleep(3.0)
        if probe:
            # Shakedown only: a second publisher on the escape-path topic (see probe_escape_path.py).
            extras.append(subprocess.Popen(
                ['python3', os.path.join(HERE, 'probe_escape_path.py')],
                stdout=open(os.path.join(out, 'probe.log'), 'wb'),
                stderr=subprocess.STDOUT, env=env, start_new_session=True))

        denv = rl.drive_env(scenario, base_env=env, localizer='amcl', drive_timeout=rr.WATCHDOG_S)
        meta['drive_wall_start'] = time.time()
        with open(os.path.join(out, 'drive.log'), 'w') as dlog:
            drive = subprocess.Popen(['python3', rl.RosLauncher().smoke_drive],
                                     stdout=dlog, stderr=subprocess.STDOUT,
                                     env=denv, start_new_session=True)
            extras.append(drive)
            try:
                meta['drive_returncode'] = drive.wait(timeout=rr.WATCHDOG_S + 60.0)
            except subprocess.TimeoutExpired:
                meta['drive_returncode'] = None
                meta['drive_killed_by_driver'] = True
        meta['drive_wall_end'] = time.time()
        ok = True
    finally:
        for p in extras:
            rr.stop_group(p, grace=5.0)
        rr.stop_group(launch)
        meta['swept_pids'] = rr.sweep_tag(tag)
        meta['wall_end'] = time.time()
        sampler.stop()
        meta['top_after'] = hostload.top_consumers()
        if 'drive_wall_end' in meta:
            meta['host_load'] = hostload.verdict(
                hostload.read_rows(os.path.join(out, 'host_cpu.csv')),
                meta['drive_wall_start'], meta['drive_wall_end'])
        # Only the normal path marks the trial complete. A Ctrl-C or an exception leaves
        # 'interrupted': resume keeps the directory as .abortedN and runs the trial again.
        if ok:
            meta['complete'] = True
        else:
            meta['interrupted'] = True
        save()
    print(f'[campaign2] done: {trial.trial_id} active={meta["active"]} '
          f'drive_rc={meta.get("drive_returncode")} host_load={meta.get("host_load")}', flush=True)
    return meta


def check_stops(raw_root, trials_done_cells) -> list:
    import analyze as an
    recs = an.registered_records(raw_root)
    inval = {f'{s}/{c}': an.count_invalid_attempts(raw_root, (s, c)) for s, c in trials_done_cells}
    return an.evaluate_stops(recs, inval, an.superseded_records(raw_root))


def phase_gate_refusal(raw_root, trial_or_block) -> str | None:
    """Refusal text when ``trial_or_block`` (a Trial, or the block name 'B') would drive s3 or
    run block B before Phase A is complete and its gate decided. None when allowed."""
    import analyze as an
    if isinstance(trial_or_block, str):
        needs = trial_or_block == 'B'
    else:
        needs = trial_or_block.scenario == 's3_ctrap' or trial_or_block.phase == 'B'
    if not needs:
        return None
    return an.s3_refusal(an.registered_records(raw_root))


def trial_refusal(raw_root, out_root, trial, ident) -> str | None:
    """Why ``trial`` must not run now (s3 and block B wait for Phase A), or None. A trial that is
    already complete and valid is skipped, not refused."""
    refusal = phase_gate_refusal(raw_root, trial)
    if refusal and existing_state(os.path.join(out_root, trial.trial_id), ident) != 'complete_valid':
        return refusal
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--block', default='all', choices=('A', 'B', 'all', 'shakedown'))
    ap.add_argument('--raw', default=None, help='raw data root (default: that of --attempt)')
    ap.add_argument('--attempt', type=int, default=1,
                    help='attempt number; a restart after a fix is attempt 2 with its own raw root')
    ap.add_argument('--list', action='store_true', help='print the plan and exit')
    ap.add_argument('--allow-unfrozen', action='store_true',
                    help='run a counted block without a passing freeze manifest (not for the campaign)')
    ap.add_argument('--max-trials', type=int, default=None,
                    help='stop after this many trials have run in this session (resume later)')
    args = ap.parse_args()
    args.raw = args.raw or plan_mod.raw_root(args.attempt)

    trials = plan_mod.plan(args.block)
    if args.list:
        for i, t in enumerate(trials, 1):
            print(f'{i:2d} {t.phase:9s} {t.trial_id}')
        print(f'{len(trials)} trials')
        return 0

    import run_rerun as rr                      # campaign-1 helpers (process handling, gate)
    from experiments.runner import config as config_mod
    from experiments.runner import scenario as scenario_mod
    import yaml

    counted_block = args.block != 'shakedown'
    if counted_block and not args.allow_unfrozen:
        import freeze_manifest
        diffs = freeze_manifest.check()
        if diffs:
            print('[campaign2] REFUSED: the frozen files differ from FREEZE_MANIFEST.json '
                  '(PREREG.md, "Frozen files"):\n  ' + '\n  '.join(diffs[:20]), flush=True)
            return EXIT_REFUSED
    ident = (provenance.identity(REPO, os.path.join(HERE, 'FREEZE_MANIFEST.json'))
             if counted_block else None)
    if counted_block:
        refusal = phase_gate_refusal(args.raw, args.block)
        if refusal:
            print(f'[campaign2] REFUSED block {args.block}: {refusal}', flush=True)
            return EXIT_REFUSED
        foreign = foreign_trials(os.path.join(args.raw, 'trials'), ident)
        if foreign:
            print('[campaign2] REFUSED: trial directories of another attempt (freeze manifest or '
                  'tag commit differ) are in this raw root; a restart needs its own root '
                  f'(--attempt 2):\n  ' + '\n  '.join(foreign[:20]), flush=True)
            return EXIT_REFUSED
    if not rr.rl.ros_available():
        raise SystemExit('no ros2 on PATH: activate the environment and source the workspace')
    # Refuse every configuration whose parameter tree cannot let the escape act.
    with open(plan_mod.PARAMS_YAML) as f:
        base_tree = yaml.safe_load(f)
    suite = config_mod.load_ablations(os.path.join(REPO, 'experiments', 'configs', 'ablations.yaml'))
    scenarios = {n: scenario_mod.load_scenario(os.path.join(REPO, p))
                 for n, p in plan_mod.SCENARIO_FILES.items()}
    for c in plan_mod.CONFIG_ORDER:
        plan_mod.refuse_unsafe(config_mod.resolve_params(suite, plan_mod.CONFIGS[c]['ablation'],
                                                         base_tree=base_tree))
    info = environment_info(rr)
    print(f'[campaign2] environment: {json.dumps(info, indent=1)}', flush=True)
    if info['check_mppi_rollout_exit'] != 0:
        raise SystemExit('check_mppi_rollout does not report a correct nav2_mppi_controller build')
    if info['tracked_inputs_modified']:
        print('[campaign2] WARNING: tracked inputs are modified (recorded in meta.json):\n'
              + info['tracked_inputs_modified'], flush=True)

    sub = 'shakedown' if args.block == 'shakedown' else 'trials'
    out_root = os.path.join(args.raw, sub)
    os.makedirs(out_root, exist_ok=True)
    cells = sorted({t.cell for t in plan_mod.plan('all')})
    ran = 0
    for trial in trials:
        if sub == 'trials':
            stops = check_stops(args.raw, cells)
            if stops:
                for code, msg in stops:
                    print(f'[campaign2] STOP {code}: {msg}', flush=True)
                print('[campaign2] Stopping rule triggered: report to the lead; see PREREG.md '
                      '(stopping rules). Nothing was deleted.', flush=True)
                return EXIT_STOP
        if sub == 'trials':
            refusal = trial_refusal(args.raw, out_root, trial, ident)
            if refusal:
                print(f'[campaign2] REFUSED {trial.trial_id}: {refusal}', flush=True)
                return EXIT_REFUSED
        if args.max_trials is not None and ran >= args.max_trials:
            print(f'[campaign2] --max-trials {args.max_trials} reached; resume with the same command',
                  flush=True)
            return 0
        while True:
            prov = None
            if sub == 'trials' and not args.allow_unfrozen:
                import freeze_manifest
                with open(freeze_manifest.MANIFEST) as f:
                    manifest_files = json.load(f)['files']
                pdiffs, prov = provenance.verify_installed(
                    REPO, info['controller_prefix'], info['testbed_prefix'], manifest_files,
                    freeze_manifest.files())
                if pdiffs:
                    print('[campaign2] REFUSED: what is installed is not what is frozen '
                          '(PREREG.md, "Provenance"):\n  ' + '\n  '.join(pdiffs[:20]), flush=True)
                    return EXIT_REFUSED
            meta = run_trial(rr, config_mod, suite, base_tree, scenarios[trial.scenario], trial,
                             out_root, info, probe=(trial.phase == 'shakedown'),
                             ident=ident, prov=prov)
            if meta is None:                       # complete and valid: skipped
                break
            ran += 1
            time.sleep(5.0)
            load = meta.get('host_load')
            if load is None or load['valid']:
                break
            # Invalidated by foreign load: keep the attempt under the next free name,
            # report it, repeat it (amendment 8).
            tdir = os.path.join(out_root, meta['trial_id'])
            os.rename(tdir, next_free(tdir, 'invalid'))
            print(f'[campaign2] invalid by host load, repeating: {meta["trial_id"]} '
                  f'({load.get("reason")})', flush=True)
            if sub == 'trials':
                stops = check_stops(args.raw, cells)
                if stops:
                    break
    if sub == 'trials':
        stops = check_stops(args.raw, cells)
        for code, msg in stops:
            print(f'[campaign2] STOP {code}: {msg}', flush=True)
        if stops:
            return EXIT_STOP
    print('[campaign2] block finished', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
