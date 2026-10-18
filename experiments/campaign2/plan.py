# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Campaign 2: configurations, trial plan and parameter overlays (PREREG.md).

Pure Python (standard library only), no ROS. The driver ``run_campaign2.py`` and
the analyzer ``analyze.py`` import this file, so the plan, the cell names and the
stopping thresholds exist once.
"""

from __future__ import annotations

import hashlib
import os
from typing import NamedTuple

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
PARAMS_YAML = os.path.join(REPO, 'experiments', 'sim', 'nav2_se_campaign2.yaml')
BT_DIR = os.path.join(REPO, 'experiments', 'se_mppi_utrap', 'behavior_trees')
RAW_ROOT = '/mnt/d/wsl/se-mppi/campaign2'


def raw_root(attempt: int = 1) -> str:
    """Raw-data root of an attempt. Attempt 1 uses ``RAW_ROOT``; a restart (a fix after trial 1,
    decision D-7) is attempt 2 and has its own root, so that no trial of attempt 1 is ever
    reused or mixed into it."""
    if attempt < 1:
        raise ValueError(attempt)
    return RAW_ROOT if attempt == 1 else f'{RAW_ROOT}/attempt{attempt}'

ENV_LABEL = 'correct'

FP = 'controller_server.ros__parameters.FollowPath'
BT_KEY = 'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml'
PROGRESS_TIME_KEY = 'controller_server.ros__parameters.progress_checker.movement_time_allowance'
BT_TEMPLATE = '$(find-pkg-share se_mppi_utrap)/behavior_trees/{}'
BT_DEFAULT = 'nav_to_pose_default_recovery.xml'
BT_REPLAN_ONLY = 'nav_to_pose_replanning_only.xml'

# Configuration id -> ablation name of experiments/configs/ablations.yaml,
# behaviour tree file, progress-checker movement_time_allowance in s.
# failure_tolerance 0.0 and the 0.5 m radius come from nav2_se_campaign2.yaml.
CONFIGS = {
    'C1': {'ablation': 'A_stock', 'tree': BT_DEFAULT, 'allowance_s': 10.0, 'se': False},
    'C2': {'ablation': 'F_se_full', 'tree': BT_DEFAULT, 'allowance_s': 10.0, 'se': True},
    'C3': {'ablation': 'A_stock', 'tree': BT_REPLAN_ONLY, 'allowance_s': 30.0, 'se': False},
    'C4': {'ablation': 'F_se_full', 'tree': BT_REPLAN_ONLY, 'allowance_s': 30.0, 'se': True},
}
CONFIG_ORDER = ('C1', 'C2', 'C3', 'C4')

SCENARIO_FILES = {
    's1_utrap': 'experiments/barn/s1_utrap.scenario.yaml',
    's3_ctrap': 'experiments/campaign2/s3_ctrap.scenario.yaml',
    's4_ltrap': 'experiments/campaign2/s4_ltrap.scenario.yaml',
    's0_open': 'experiments/campaign2/s0_open.scenario.yaml',
    's1_utrap_evidence': 'experiments/barn/s1_utrap_evidence.scenario.yaml',
}
SCENARIO_ORDER = ('s1_utrap', 's3_ctrap', 's4_ltrap', 's0_open')   # counted scenarios
# Configurations that run on a scenario. s4_ltrap (the pocket outside the development family,
# amendment 1 of PREREG.md) runs the two no-recovery configurations only: the clean transfer test
# is C4 against C3, so C1 and C2 are not run there. Every other counted scenario runs all four.
SCENARIO_CONFIGS = {'s4_ltrap': ('C3', 'C4')}
HELD_OUT_POCKETS = ('s3_ctrap', 's4_ltrap')


def configs_of(scenario: str) -> tuple:
    return SCENARIO_CONFIGS.get(scenario, CONFIG_ORDER)
EVIDENCE_SCENARIO = 's1_utrap_evidence'
EVIDENCE_CONFIGS = ('C2', 'C4')                           # SE configurations only
REPS = 6
EVIDENCE_REPS = 2
PHASE_A_C4_REPS = 6          # C4 x s1
PHASE_A_C3_REPS = 3          # C3 x s1

# Stopping rules (PREREG.md, "Stopping rules").
GATE_CELL = ('s1_utrap', 'C4')
GATE_MAX_SUCCESSES = 1       # Phase A: C4 x s1 <= 1 of 6 -> stop
STOCK_S0_MAX_SUCCESSES = 4   # stock MPPI <= 4 of 6 on s0 -> stack broken
# 4 of 6 or fewer is certain as soon as REPS - STOCK_S0_MAX_SUCCESSES = 2 valid trials of a
# stock cell on s0 have failed: the rule fires then, not after the sixth trial.
STOCK_S0_MIN_FAILURES = REPS - STOCK_S0_MAX_SUCCESSES
MAX_INVALID_PER_CELL = 6     # more invalid attempts than this on a cell -> incomplete


class Trial(NamedTuple):
    scenario: str
    config: str
    rep: int
    phase: str               # 'A', 'B' or 'shakedown'

    @property
    def counted(self) -> bool:
        return self.phase in ('A', 'B') and self.scenario != EVIDENCE_SCENARIO

    @property
    def cell(self) -> tuple:
        return (self.scenario, self.config)

    @property
    def trial_id(self) -> str:
        return trial_id(self.scenario, self.config, self.rep)


def trial_id(scenario: str, config: str, rep: int, env_label: str = ENV_LABEL) -> str:
    return f'{env_label}__{scenario}__{config}__rep{rep}'


def _rotate(cells: list, rep: int, step: int = 1) -> list:
    """Rotate the cell order with the repetition (``step`` places per repetition),
    so that no cell always runs first after a pause or always follows the same cell."""
    if not cells:
        return cells
    k = (step * rep) % len(cells)
    return cells[k:] + cells[:k]


def phase_a() -> list:
    out = []
    for rep in range(PHASE_A_C4_REPS):
        cells = [('s1_utrap', 'C4')]
        if rep < PHASE_A_C3_REPS:
            cells.append(('s1_utrap', 'C3'))
        out += [Trial(s, c, rep, 'A') for s, c in _rotate(cells, rep)]
    return out


def phase_b() -> list:
    canonical = [(s, c) for s in SCENARIO_ORDER for c in configs_of(s)]
    canonical += [(EVIDENCE_SCENARIO, c) for c in EVIDENCE_CONFIGS]
    out = []
    for rep in range(REPS):
        cells = []
        for s, c in canonical:
            if s == EVIDENCE_SCENARIO:
                if rep >= EVIDENCE_REPS:
                    continue
            elif (s, c) == GATE_CELL:
                continue                              # all six ran in Phase A
            elif (s, c) == ('s1_utrap', 'C3') and rep < PHASE_A_C3_REPS:
                continue                              # ran in Phase A
            cells.append((s, c))
        out += [Trial(s, c, rep, 'B') for s, c in _rotate(cells, rep, step=2)]
    return out


def plan(phase: str = 'all') -> list:
    if phase == 'A':
        return phase_a()
    if phase == 'B':
        return phase_b()
    if phase == 'all':
        return phase_a() + phase_b()
    if phase == 'shakedown':
        return [Trial('s0_open', 'C2', 0, 'shakedown')]
    raise ValueError(phase)


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def overlay(config: str) -> dict:
    """Dotted-path edits of ``nav2_se_campaign2.yaml`` that configuration ``config`` adds.

    Every key must already exist in the file (the driver checks).
    """
    spec = CONFIGS[config]
    return {
        BT_KEY: BT_TEMPLATE.format(spec['tree']),
        PROGRESS_TIME_KEY: spec['allowance_s'],
    }


def refuse_unsafe(params: dict) -> None:
    """Refuse a parameter tree in which the escape cannot act (DESIGN_ADVICE D1).

    ``se_enabled`` with ``vx_min > 0``: the robot cannot stop or reverse, as in
    campaign 1 set P. The stock configurations have ``se_enabled`` false and are
    not refused by this rule; the shipped file has vx_min -0.35 for all of them.
    """
    fp = params['controller_server']['ros__parameters']['FollowPath']
    if float(fp['vx_min']) > 0.0 and fp.get('se_enabled', True):
        raise SystemExit(
            f'refused: se_enabled with vx_min = {fp["vx_min"]} > 0; the robot cannot stop '
            'or reverse, so the escape cannot act (campaign 1, set P)')
