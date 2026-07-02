# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Suite orchestrator (design §3): scenario × config × seed.

Loops the ablation matrix over a set of scenarios and seeds, runs each
combination as an isolated trial, writes a raw JSON per trial, and supports
``resume`` (skip already-completed trials) so a multi-thousand-trial sweep can
be interrupted and continued. The simulator backend is injected as a factory so
the loop is testable offline with ``FakeLauncher``; on hardware pass a
``RosLauncher`` factory.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

import yaml

from . import config as config_mod
from . import scenario as scenario_mod
from .scenario import Scenario, validate
from .trial import DriveResult, FakeLauncher, TrialConfig, run_trial


def trial_id(scenario: Scenario, config_name: str, seed: int) -> str:
    return f'{scenario.name}__{config_name}__seed{seed}'


def result_path(results_dir: str, scenario: Scenario, config_name: str,
                seed: int) -> str:
    return os.path.join(results_dir, scenario.tier, config_name,
                        f'{scenario.name}_seed{seed}.json')


@dataclass
class SuiteReport:
    n_total: int = 0
    n_run: int = 0
    n_skipped: int = 0
    n_setup_fail: int = 0
    records: list = field(default_factory=list)   # list of trial dicts (in-mem)
    outcomes: dict = field(default_factory=dict)  # outcome -> count


def run_suite(launcher_factory, suite: config_mod.AblationSuite,
              scenarios: list, seeds: list, results_dir: str, *,
              config_names: list | None = None, trial_cfg: TrialConfig | None = None,
              base_tree: dict | None = None, resume: bool = True,
              validate_scenarios: bool = True, do_cleanup: bool = True,
              logger=None) -> SuiteReport:
    """Run the full matrix; returns a :class:`SuiteReport`.

    ``launcher_factory`` is a zero-arg callable returning a fresh Launcher per
    trial (so each trial is isolated). ``base_tree`` may be supplied to avoid
    re-reading the base params yaml on every config resolution. ``do_cleanup``
    runs the per-trial process/SHM clean-restart (handoff §2-A); it MUST be off
    for the offline FakeLauncher path — the fake launcher starts nothing, so
    ``pkill``-ing ``nav2``/``gz``/``ruby`` would only kill a real sim on the host
    (the live ``RosLauncher`` path keeps it on).
    """
    cfg_names = config_names or config_mod.config_names(suite)
    report = SuiteReport()

    def log(msg):
        if logger:
            logger(msg)

    # Pre-resolve params per config once (base + overlay is pure).
    resolved = {name: config_mod.resolve_params(suite, name, base_tree=base_tree)
                for name in cfg_names}

    for scenario in scenarios:
        if validate_scenarios:
            ok, reason = validate(scenario, robot_radius=(
                trial_cfg.robot_radius if trial_cfg else 0.22))
            if not ok:
                log(f'skip scenario {scenario.name}: {reason}')
                continue
        for name in cfg_names:
            for seed in seeds:
                report.n_total += 1
                out_path = result_path(results_dir, scenario, name, seed)
                if resume and os.path.exists(out_path):
                    report.n_skipped += 1
                    with open(out_path) as f:
                        rec = json.load(f)
                    report.records.append(rec)
                    report.outcomes[rec['outcome']] = \
                        report.outcomes.get(rec['outcome'], 0) + 1
                    continue

                launcher = launcher_factory()
                rec = run_trial(launcher, scenario, name, resolved[name], seed,
                                cfg=trial_cfg, logger=logger,
                                do_cleanup=do_cleanup)
                d = rec.to_dict()
                _write_json(out_path, d)
                report.records.append(d)
                report.n_run += 1
                report.outcomes[rec.outcome] = \
                    report.outcomes.get(rec.outcome, 0) + 1
                if rec.outcome == 'SETUP_FAIL':
                    report.n_setup_fail += 1
                log(f'{trial_id(scenario, name, seed)} -> {rec.outcome}')

    return report


def _write_json(path: str, data: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)  # atomic: a crash mid-write never leaves a half file


# --------------------------------------------------------------------------- #
# Turnkey entrypoint (design §3, §8): suite.yaml -> one-command sweep
# --------------------------------------------------------------------------- #
@dataclass
class SuiteSettings:
    """Parsed ``suite.yaml``: everything the CLI needs to run the matrix."""
    ablations_path: str
    experiments_root: str               # dir holding the tier subdirs
    tiers: list
    seeds: list
    config_names: list | None
    results_dir: str
    resume: bool
    trial_cfg: TrialConfig
    scenario_names: list | None = None   # allowlist of scenario names (None=all)


def load_suite(path: str) -> SuiteSettings:
    """Load ``configs/suite.yaml`` into a :class:`SuiteSettings`.

    Relative paths (``ablations``, ``results_dir``) resolve next to suite.yaml,
    matching how ``config.load_ablations`` resolves its base params path.
    """
    with open(path) as f:
        doc = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(path))

    def resolve(p):
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(base, p))

    t = doc.get('trial', {}) or {}
    trial_cfg = TrialConfig(
        robot_radius=t.get('robot_radius', 0.22),
        goal_tol=t.get('goal_tol', 0.5),
        setup_timeout_s=t.get('setup_timeout_s', 60.0),
        watchdog_s=t.get('watchdog_s', 120.0),
        setup_retries=t.get('setup_retries', 1))
    # The tier dirs live one level above configs/ (experiments/{barn,...}).
    experiments_root = os.path.normpath(os.path.join(base, '..'))
    return SuiteSettings(
        ablations_path=resolve(doc['ablations']),
        experiments_root=experiments_root,
        tiers=doc.get('tiers') or list(scenario_mod.TIERS),
        seeds=list(doc.get('seeds') or []),
        config_names=doc.get('configs'),
        results_dir=resolve(doc.get('results_dir', '../results')),
        resume=bool(doc.get('resume', True)),
        trial_cfg=trial_cfg,
        scenario_names=doc.get('scenarios'))


def _fake_launcher_factory():
    """A FakeLauncher that drives straight start->goal (CI / dry-run path).

    Produces a reach-the-goal sample stream from each scenario so the full
    matrix executes offline without ROS — exercises the orchestration end to
    end and writes real result JSON.
    """
    def drive(scenario):
        sx, sy = scenario.start_xy
        gx, gy = scenario.goal_xy
        n = 21
        samples = [{'t': i * 0.5,
                    'x': sx + (gx - sx) * i / (n - 1),
                    'y': sy + (gy - sy) * i / (n - 1),
                    'v': 0.5, 'w': 0.0, 'loop_ms': 12.0} for i in range(n)]
        return DriveResult(samples=samples)
    return FakeLauncher(drive)


def make_launcher_factory(kind: str, settings: SuiteSettings, *, logger=None):
    """Return a zero-arg launcher factory for ``--launcher {fake,ros}``."""
    if kind == 'fake':
        return _fake_launcher_factory
    if kind == 'ros':
        from .ros_launcher import RosLauncher   # lazy: keeps offline import clean
        return lambda: RosLauncher(results_dir=settings.results_dir,
                                   logger=logger)
    raise ValueError(f'unknown launcher: {kind!r} (use fake|ros)')


def run_from_settings(settings: SuiteSettings, launcher_kind: str = 'fake',
                      *, logger=print) -> SuiteReport:
    """Glue load_suite -> scenarios -> run_suite for the CLI / programmatic use."""
    suite = config_mod.load_ablations(settings.ablations_path)
    scenarios = []
    for tier in settings.tiers:
        scenarios.extend(scenario_mod.discover(settings.experiments_root,
                                               tier=tier))
    if settings.scenario_names:
        allow = set(settings.scenario_names)
        scenarios = [s for s in scenarios if s.name in allow]
    factory = make_launcher_factory(launcher_kind, settings, logger=logger)
    # Clean-restart (process/SHM kill) only makes sense for the live launcher;
    # the fake launcher starts nothing, so cleanup would only kill a real sim.
    return run_suite(
        factory, suite, scenarios, settings.seeds, settings.results_dir,
        config_names=settings.config_names, trial_cfg=settings.trial_cfg,
        resume=settings.resume, do_cleanup=(launcher_kind == 'ros'),
        logger=logger)


def main(argv: list | None = None) -> int:
    import argparse
    p = argparse.ArgumentParser(
        description='Run the SE-MPPI evaluation suite (scenario × config × '
                    'seed). Default launcher is the offline FakeLauncher; pass '
                    '--launcher ros on a ROS2/Gazebo GPU workstation.')
    here = os.path.dirname(os.path.abspath(__file__))
    default_cfg = os.path.normpath(os.path.join(here, '..', 'configs',
                                                'suite.yaml'))
    p.add_argument('--config', default=default_cfg,
                   help='suite.yaml (default: experiments/configs/suite.yaml)')
    p.add_argument('--launcher', choices=('fake', 'ros'), default='fake',
                   help='fake = offline dry-run; ros = live ROS2/Gazebo')
    p.add_argument('--results-dir', default=None,
                   help='override results_dir from suite.yaml')
    p.add_argument('--tiers', nargs='*', default=None,
                   help='subset of tiers to run (default: from suite.yaml)')
    p.add_argument('--seeds', type=int, nargs='*', default=None,
                   help='override the seed list')
    p.add_argument('--scenarios', nargs='*', default=None,
                   help='only run scenarios with these names (default: all '
                        'discovered in the selected tiers)')
    p.add_argument('--configs', nargs='*', default=None,
                   help='only run these ablation configs (default: from '
                        'suite.yaml, else the full matrix)')
    p.add_argument('--no-resume', action='store_true',
                   help='re-run trials even if a result JSON already exists')
    args = p.parse_args(argv)

    settings = load_suite(args.config)
    if args.results_dir:
        settings.results_dir = os.path.abspath(args.results_dir)
    if args.tiers:
        settings.tiers = args.tiers
    if args.seeds is not None:
        settings.seeds = args.seeds
    if args.scenarios is not None:
        settings.scenario_names = args.scenarios
    if args.configs is not None:
        settings.config_names = args.configs
    if args.no_resume:
        settings.resume = False

    report = run_from_settings(settings, args.launcher)
    print(f'[run_suite] total={report.n_total} run={report.n_run} '
          f'skipped={report.n_skipped} setup_fail={report.n_setup_fail}')
    print(f'[run_suite] outcomes={report.outcomes}')
    print(f'[run_suite] results -> {settings.results_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
