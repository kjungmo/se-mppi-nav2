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


# --------------------------------------------------------------------------- #
# SE-MPPI v2 (opt-in). Same six toggle sets, plus the v2 escape logic of
# ``prototype/se_mppi_v2.py``: anchor-path progress with 1 Hz simulated replans,
# horizon-ray opening selection, frozen-waypoint hopping, "all rollouts collide
# -> zero command", and the C++ parameter values (36 rays, 2.0 m, stall window
# 30). ``CONFIGS`` above is untouched: the paper-v1 results come from it.
# --------------------------------------------------------------------------- #
import os as _os  # noqa: E402
import sys as _sys  # noqa: E402

_PROTO = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))),
                       'prototype')
if _PROTO not in _sys.path:
    _sys.path.insert(0, _PROTO)
from se_mppi_v2 import v2_params  # noqa: E402


def make_v2_configs(**overrides):
    """The six configs with the v2 escape logic; ``overrides`` set v2 flags/values."""
    v2 = v2_params(**overrides)
    return {name: dict(cfg, v2=v2, stall=v2['stall_window'])
            for name, cfg in CONFIGS.items()}


CONFIGS_V2 = make_v2_configs()
# 'v2_advised': the same algorithm without the four additions (a) to (d) of PREREG.md, that is
# the design of DESIGN_ADVICE.md as advised (escape_extras=False). 'v2' is the shipped algorithm.
CONFIGS_V2_ADVISED = make_v2_configs(escape_extras=False)
VARIANTS = {'v1': CONFIGS, 'v2': CONFIGS_V2, 'v2_advised': CONFIGS_V2_ADVISED}
