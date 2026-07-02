# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Scenario definitions and loading (design §4).

A scenario is the immutable description of one navigation task: which map/world,
where the robot starts, where it must go, and any dynamic agents. Scenarios are
stored as small YAML files under ``experiments/{barn,dynabarn,hunav}/`` and
loaded here. ``validate`` reuses the eroded-reachability check so an unreachable
goal is caught before launch (recorded as SETUP_FAIL, not a navigation failure).
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field

import yaml

from .gridmap import GridMap, reachable

TIERS = ('barn', 'dynabarn', 'hunav')


@dataclass
class Scenario:
    name: str
    tier: str
    map_yaml: str                       # absolute path to the occupancy map yaml
    start: tuple                        # (x, y, yaw)
    goal: tuple                         # (x, y, yaw)
    world: str | None = None            # gz/sdf world (None for loopback/static)
    agents: list = field(default_factory=list)   # dynamic-agent specs (DynaBARN/HuNav)
    optimal_length: float | None = None
    optimal_time: float | None = None
    meta: dict = field(default_factory=dict)

    @property
    def start_xy(self):
        return (self.start[0], self.start[1])

    @property
    def goal_xy(self):
        return (self.goal[0], self.goal[1])


def _as_pose(v):
    """Coerce a [x, y] or [x, y, yaw] list into an (x, y, yaw) tuple."""
    if v is None:
        raise ValueError('pose is required')
    seq = list(v)
    if len(seq) == 2:
        seq = seq + [0.0]
    if len(seq) != 3:
        raise ValueError(f'pose must be [x,y] or [x,y,yaw], got {v!r}')
    return (float(seq[0]), float(seq[1]), float(seq[2]))


def load_scenario(path: str, tier: str | None = None) -> Scenario:
    """Load one scenario YAML. Relative ``map``/``world`` resolve next to it."""
    with open(path) as f:
        doc = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(path))

    def resolve(p):
        if p is None:
            return None
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(base, p))

    fname = os.path.basename(path)
    if fname.endswith('.scenario.yaml'):
        default_name = fname[:-len('.scenario.yaml')]
    else:
        default_name = os.path.splitext(fname)[0]
    name = doc.get('name') or default_name
    tier = tier or doc.get('tier') or os.path.basename(os.path.dirname(path))
    return Scenario(
        name=name,
        tier=tier,
        map_yaml=resolve(doc['map']),
        start=_as_pose(doc['start']),
        goal=_as_pose(doc['goal']),
        world=resolve(doc.get('world')),
        agents=doc.get('agents', []) or [],
        optimal_length=doc.get('optimal_length'),
        optimal_time=doc.get('optimal_time'),
        meta=doc.get('meta', {}) or {},
    )


def loader_for_tier(tier: str):
    """Return the scenario loader for ``tier``.

    The dynamic tiers have richer schemas (HuNavSim pedestrians, DynaBARN moving
    obstacles), so they get dedicated loaders that normalise agents into the
    launch-ready field names; ``barn`` (static) uses the base loader. Imported
    lazily so ``scenario`` stays import-light and free of cycles.
    """
    if tier == 'hunav':
        from .hunav import load_hunav_scenario
        return load_hunav_scenario
    if tier == 'dynabarn':
        from .dynabarn import load_dynabarn_scenario
        return load_dynabarn_scenario
    return load_scenario


def discover(root: str, tier: str | None = None,
             *, tier_loaders: bool = True) -> list:
    """Find every ``*.scenario.yaml`` under a tier directory (or all tiers).

    With ``tier_loaders`` (default) the dynamic tiers use their dedicated loaders
    (:mod:`hunav` / :mod:`dynabarn`); set it False to force the base loader.
    """
    tiers = [tier] if tier else TIERS
    found = []
    for t in tiers:
        load = loader_for_tier(t) if tier_loaders else load_scenario
        pattern = os.path.join(root, t, '*.scenario.yaml')
        for p in sorted(glob.glob(pattern)):
            # Dedicated loaders set tier from the file/dir themselves.
            found.append(load(p) if tier_loaders and t in ('hunav', 'dynabarn')
                         else load(p, tier=t))
    return found


def validate(scenario: Scenario, robot_radius: float = 0.22) -> tuple:
    """Return (ok, reason). Checks the map exists and goal is reachable.

    A scenario that fails here should be skipped or recorded SETUP_FAIL rather
    than counted as a navigation failure.
    """
    if not scenario.map_yaml or not os.path.exists(scenario.map_yaml):
        return False, f'map not found: {scenario.map_yaml}'
    try:
        gm = GridMap.from_yaml(scenario.map_yaml)
    except Exception as e:  # malformed map / pgm
        return False, f'map load error: {e}'
    if not reachable(gm, scenario.start_xy, scenario.goal_xy, robot_radius):
        return False, 'goal not reachable from start (eroded free space)'
    return True, 'ok'
