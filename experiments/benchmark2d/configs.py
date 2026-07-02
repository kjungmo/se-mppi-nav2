# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Ablation configs (paper §VI / experiments/README A–F) mapped to the 2D proto
control toggles used by ``rollout.rollout``.

Each config is the exact toggle dict the validated ``run_validation.run`` loop
consumes (``use_escape``, ``use_gap``, ``use_cbf``, ``use_coordination``):

  A_stock       — escape off, CBF off               (Nav2 stock MPPI analogue)
  C_escape      — escape detect-switch only, CBF off (isolates escape)
  D_cbf         — CBF only, escape off               (isolates the safety filter)
  E_indep       — escape + CBF, independent          (α_escape = α_base: no coord.)
  F_full        — escape + CBF, coordinated          (α 2→6 + TTC override)
  Fminus_nogap  — F with gap search off              (escape without gap subgoal)

The load-bearing contrast is **E_indep vs F_full**: identical mechanisms, the
only difference being whether the CBF gain is coordinated with the escape phase.
"""

from __future__ import annotations

CONFIGS = {
    'A_stock':      dict(use_escape=False, use_gap=False, use_cbf=False, use_coordination=False),
    'C_escape':     dict(use_escape=True,  use_gap=True,  use_cbf=False, use_coordination=False),
    'D_cbf':        dict(use_escape=False, use_gap=False, use_cbf=True,  use_coordination=False),
    'E_indep':      dict(use_escape=True,  use_gap=True,  use_cbf=True,  use_coordination=False),
    'F_full':       dict(use_escape=True,  use_gap=True,  use_cbf=True,  use_coordination=True),
    'Fminus_nogap': dict(use_escape=True,  use_gap=False, use_cbf=True,  use_coordination=True),
}

# Canonical ordering for tables/plots.
ORDER = ['A_stock', 'C_escape', 'D_cbf', 'E_indep', 'F_full', 'Fminus_nogap']

# The primary hypothesis contrast and the full set of F-vs-baseline contrasts.
KEY_CONTRAST = ('E_indep', 'F_full')
BASELINES_VS_F = ['A_stock', 'C_escape', 'D_cbf', 'E_indep']
