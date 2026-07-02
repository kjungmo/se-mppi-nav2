# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""SE-MPPI evaluation harness (L11).

Reproducible benchmark runner that fills the SE-MPPI paper's *pending*
quantitative tables. See ``docs/architecture/2026-06_evaluation-harness-design.md``.

The pure-Python core (config resolution, metrics, classification, statistics,
aggregation) is unit-tested offline with no live simulator. The live launch is
isolated behind the ``Launcher`` protocol in :mod:`trial`, so the orchestration
is exercised with a ``FakeLauncher`` here and wired to ROS2/Gazebo on a GPU
workstation (design §9).
"""
