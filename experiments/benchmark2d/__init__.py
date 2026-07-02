# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Randomized 2D quantitative benchmark for SE-MPPI (paper §VI-C).

A seed-deterministic scenario generator + batch runner + aggregator that drives
the *same* validated 2D controller math used by ``experiments/prototype`` (the
``se_mppi_proto`` primitives) across three randomized scenario families
(``utrap``, ``clutter``, ``dynamic``) and the A/C/D/E/F(+F⁻) ablation configs,
in a paired design (identical scenario + identical MPPI noise stream per config),
then computes McNemar / Mann–Whitney / Cliff's δ / Holm statistics.

The physics/formulas are *not* re-derived here — only the scenario geometry and
the batch harness are new. See ``rollout.py`` for the one documented harness
difference from ``run_validation.run`` (CBF scoped to dynamic obstacles, matching
the real controller of paper §IV-D / §VI-B).
"""
