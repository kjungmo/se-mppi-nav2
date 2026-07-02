# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Single-trial state machine (design §2).

    [CLEANUP] -> [LAUNCH] -> [WAIT_ACTIVE] -> [DRIVE] -> [CLASSIFY] -> [TEARDOWN]

The live simulator interaction is abstracted behind the :class:`Launcher`
protocol so this orchestration — including the SETUP_FAIL vs navigation-outcome
distinction and the one-shot setup retry (design §7) — is unit-tested offline
with a :class:`FakeLauncher`. The concrete ROS2/Gazebo launcher lives in
:class:`RosLauncher` (a documented stub that requires the GPU workstation).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol

from . import cleanup as cleanup_mod
from . import metrics as metrics_mod
from .scenario import Scenario


@dataclass
class DriveResult:
    """What a Launcher reports back from the drive phase."""
    samples: list                       # list of metric sample dicts
    timeout: bool = False               # watchdog tripped on wall/sim time
    stuck: bool = False                 # watchdog tripped on no-progress
    nav_status: str | None = None       # bt_navigator result, if available


class Launcher(Protocol):
    """Pluggable simulator backend. The runner owns lifecycle; the launcher
    owns the ROS/Gazebo plumbing."""

    def launch(self, params: dict, scenario: Scenario) -> object:
        """Bring up nav2 + sim + controller for ``params``; return a handle."""

    def wait_active(self, handle: object, timeout: float) -> bool:
        """Block until managed nodes are active and the controller is loaded."""

    def drive(self, handle: object, scenario: Scenario,
              watchdog_s: float) -> DriveResult:
        """Send the goal and record the sample stream until done/timeout."""

    def teardown(self, handle: object) -> None:
        """Stop everything started by ``launch``."""


@dataclass
class TrialConfig:
    robot_radius: float = 0.22
    goal_tol: float = 0.5
    setup_timeout_s: float = 60.0
    watchdog_s: float = 120.0
    setup_retries: int = 1              # design §7: one automatic setup retry


@dataclass
class TrialRecord:
    scenario: str
    tier: str
    config: str
    seed: int
    outcome: str
    metrics: dict = field(default_factory=dict)
    setup_attempts: int = 0
    error: str | None = None
    cleanup_log: list = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            'scenario': self.scenario, 'tier': self.tier, 'config': self.config,
            'seed': self.seed, 'outcome': self.outcome, 'metrics': self.metrics,
            'setup_attempts': self.setup_attempts, 'error': self.error,
        }


def run_trial(launcher: Launcher, scenario: Scenario, config_name: str,
              params: dict, seed: int, cfg: TrialConfig | None = None,
              *, logger=None, do_cleanup: bool = True) -> TrialRecord:
    """Run one isolated trial and return its :class:`TrialRecord`.

    SETUP_FAIL (launch never reached active state) is retried once and, if still
    failing, recorded distinctly so it does not pollute the success denominator.
    """
    cfg = cfg or TrialConfig()
    rec = TrialRecord(scenario=scenario.name, tier=scenario.tier,
                      config=config_name, seed=seed, outcome='')

    def log(msg):
        if logger:
            logger(msg)

    if do_cleanup:
        rec.cleanup_log = cleanup_mod.run_cleanup(logger=logger)

    drive: DriveResult | None = None
    handle = None
    for attempt in range(1, cfg.setup_retries + 2):
        rec.setup_attempts = attempt
        try:
            handle = launcher.launch(params, scenario)
            if not launcher.wait_active(handle, cfg.setup_timeout_s):
                log(f'attempt {attempt}: not active within {cfg.setup_timeout_s}s')
                launcher.teardown(handle)
                handle = None
                if do_cleanup:
                    cleanup_mod.run_cleanup(logger=logger)
                continue
            drive = launcher.drive(handle, scenario, cfg.watchdog_s)
            break
        except Exception as e:  # launch/activation crash -> retryable setup fail
            rec.error = f'{type(e).__name__}: {e}'
            log(f'attempt {attempt}: setup error {rec.error}')
            if handle is not None:
                try:
                    launcher.teardown(handle)
                except Exception:
                    pass
                handle = None
            if do_cleanup:
                cleanup_mod.run_cleanup(logger=logger)
            continue

    if drive is None:
        rec.outcome = metrics_mod.SETUP_FAIL
        return rec

    try:
        m = metrics_mod.compute_metrics(
            drive.samples, goal=scenario.goal_xy, goal_tol=cfg.goal_tol,
            robot_radius=cfg.robot_radius,
            optimal_length=scenario.optimal_length,
            optimal_time=scenario.optimal_time,
        )
        rec.metrics = m
        rec.outcome = metrics_mod.classify(
            m, timeout=drive.timeout, stuck=drive.stuck)
    finally:
        if handle is not None:
            try:
                launcher.teardown(handle)
            except Exception as e:
                log(f'teardown error: {e}')
    return rec


# --------------------------------------------------------------------------- #
# Backends
# --------------------------------------------------------------------------- #
class FakeLauncher:
    """Deterministic in-process launcher for testing the state machine.

    Configured with a scripted :class:`DriveResult` (or a callable producing one
    from the scenario/seed), plus optional ``fail_setup_times`` to exercise the
    retry path and ``raise_setup_times`` for the crash path.
    """

    def __init__(self, drive_result=None, *, fail_setup_times: int = 0,
                 raise_setup_times: int = 0):
        self._drive = drive_result
        self.fail_setup_times = fail_setup_times
        self.raise_setup_times = raise_setup_times
        self.launches = 0
        self.teardowns = 0

    def launch(self, params, scenario):
        self.launches += 1
        if self.raise_setup_times > 0:
            self.raise_setup_times -= 1
            raise RuntimeError('simulated launch crash')
        return {'params': params, 'scenario': scenario}

    def wait_active(self, handle, timeout):
        if self.fail_setup_times > 0:
            self.fail_setup_times -= 1
            return False
        return True

    def drive(self, handle, scenario, watchdog_s):
        if callable(self._drive):
            return self._drive(scenario)
        if self._drive is None:
            return DriveResult(samples=[])
        return self._drive

    def teardown(self, handle):
        self.teardowns += 1


def __getattr__(name):
    """Lazily expose :class:`RosLauncher` from the ``ros_launcher`` module.

    Kept lazy (PEP 562) so importing ``trial`` never pulls in the live launcher's
    yaml/subprocess machinery, and ``ros_launcher`` can import ``DriveResult``
    from here without a circular import at module load. The concrete launcher is
    a documented no-ROS-safe class (it fail-fasts in ``launch`` on a machine with
    no ``ros2``); see ``ros_launcher.py`` (design §9, H-4).
    """
    if name == 'RosLauncher':
        from .ros_launcher import RosLauncher
        return RosLauncher
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
