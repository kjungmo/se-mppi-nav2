# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Clean-restart between trials (design §7, live-run handoff §2-A).

The live runs showed that stale processes, a leftover ROS2 daemon, and
fastrtps shared-memory segments in ``/dev/shm`` cause TF/clock conflicts and
phantom "robot teleported" behaviour on the next launch. Each trial therefore
starts from a clean slate. These helpers return the exact teardown steps and
(optionally) execute them, tolerating individual failures so cleanup never
aborts the suite.
"""

from __future__ import annotations

import glob
import os
import shlex
import subprocess
from dataclasses import dataclass

# Process-name fragments to kill (SIGTERM then SIGKILL) before a launch.
_PROC_PATTERNS = (
    'nav2', 'controller_server', 'planner_server', 'bt_navigator',
    'lifecycle_manager', 'gzserver', 'gzclient', 'gz sim', 'ruby',
    'robot_state_publisher', 'amcl', 'map_server', 'component_container',
)


@dataclass
class CleanupPlan:
    """A described teardown: shell-style command strings + shm globs to remove."""
    commands: list           # list[list[str]] argv vectors
    shm_globs: list          # list[str] glob patterns under /dev/shm


def cleanup_plan() -> CleanupPlan:
    """Build the teardown plan without executing it (inspectable / testable)."""
    commands = [['ros2', 'daemon', 'stop']]
    for pat in _PROC_PATTERNS:
        commands.append(['pkill', '-TERM', '-f', pat])
    for pat in _PROC_PATTERNS:
        commands.append(['pkill', '-KILL', '-f', pat])
    shm_globs = ['/dev/shm/fastrtps_*', '/dev/shm/sem.fastrtps_*']
    return CleanupPlan(commands=commands, shm_globs=shm_globs)


def run_cleanup(plan: CleanupPlan | None = None, *, dry_run: bool = False,
                logger=None, timeout: float = 10.0) -> list:
    """Execute (or, if dry_run, just describe) the teardown.

    Returns a list of human-readable action strings (the audit trail written to
    the trial log). Failures of individual steps are swallowed — a missing
    ``pkill`` target is the normal case, not an error.
    """
    plan = plan or cleanup_plan()
    log = []

    def emit(msg):
        log.append(msg)
        if logger:
            logger(msg)

    for cmd in plan.commands:
        pretty = ' '.join(shlex.quote(c) for c in cmd)
        if dry_run:
            emit(f'[dry-run] {pretty}')
            continue
        try:
            subprocess.run(cmd, timeout=timeout, check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            emit(f'ran: {pretty}')
        except FileNotFoundError:
            emit(f'skip (no such tool): {pretty}')
        except subprocess.TimeoutExpired:
            emit(f'timeout: {pretty}')
        except Exception as e:  # never let cleanup abort the suite
            emit(f'error ({e}): {pretty}')

    for pattern in plan.shm_globs:
        if dry_run:
            emit(f'[dry-run] rm {pattern}')
            continue
        for path in glob.glob(pattern):
            try:
                os.remove(path)
                emit(f'removed: {path}')
            except OSError as e:
                emit(f'error removing {path}: {e}')
    return log
