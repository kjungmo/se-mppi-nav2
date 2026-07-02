# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for clean-restart teardown (design §7) — dry-run, no side effects."""

from experiments.runner import cleanup as C


def test_plan_has_daemon_stop_and_kills():
    plan = C.cleanup_plan()
    assert ['ros2', 'daemon', 'stop'] in plan.commands
    # Both SIGTERM and SIGKILL passes over the process patterns.
    term = [c for c in plan.commands if c[:2] == ['pkill', '-TERM']]
    kill = [c for c in plan.commands if c[:2] == ['pkill', '-KILL']]
    assert term and kill
    joined = ' '.join(c[-1] for c in term)
    assert 'controller_server' in joined and 'gzserver' in joined


def test_plan_targets_fastrtps_shm():
    plan = C.cleanup_plan()
    assert any('fastrtps' in g for g in plan.shm_globs)


def test_dry_run_executes_nothing_but_logs():
    log = C.run_cleanup(dry_run=True)
    assert any('daemon' in line for line in log)
    assert all(line.startswith('[dry-run]') for line in log)
