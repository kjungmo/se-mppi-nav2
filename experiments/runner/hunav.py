# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""HuNavSim (T3 social) scenario loader (design §4, H-5; protocol §5).

HuNavSim is ROS2-native (`robotics-upo/hunav_sim`), which makes it the cleanest
first dynamic tier to wire — no ROS1 bridge, unlike BARN/DynaBARN. This loader
turns an SE-MPPI scenario YAML (the same compact schema ``scenario.py`` already
parses) into a :class:`Scenario` whose ``agents`` field carries pedestrian specs
in HuNavSim's verified ``hunav_loader`` field names, and emits the matching
``hunav_loader`` parameter block the workstation launch feeds to HuNavSim.

Input scenario YAML (superset of the base schema, all extras under ``agents``):

    name: hunav_crossing
    tier: hunav
    map: crossing.yaml            # nav2 occupancy map (relative to this file)
    world: crossing.sdf           # gz world HuNavSim populates
    start: [0.0, 0.0]
    goal:  [6.0, 0.0]
    agents:                       # list of pedestrians
      - {id: 1, behavior: 1, skin: 0, max_vel: 1.5, radius: 0.35,
         init_pose: {x: 3.0, y: 2.0, h: 0.0},
         goals: [{x: 3.0, y: -2.0, h: 0.0}, {x: 3.0, y: 2.0, h: 0.0}],
         goal_radius: 0.3, cyclic_goals: true}

The per-agent field names (``id, skin, behavior, group_id, max_vel, radius,
init_pose{x,y,z,h}, goal_radius, cyclic_goals, goals[{x,y,h}]``) match HuNavSim's
``hunav_agent_manager/config/agents.yaml`` (verified upstream, 2026-06). Defaults
are filled so a minimal agent spec still produces a launchable block; this is the
**assumed HuNavSim schema** the workstation step must conform to.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import yaml

from .scenario import Scenario, _as_pose

# Per-agent defaults (HuNavSim hunav_loader field names + sane SFM values).
_AGENT_DEFAULTS = {
    'skin': 0,
    'behavior': 1,            # 1 = regular SFM behaviour in HuNavSim
    'group_id': -1,
    'max_vel': 1.5,
    'radius': 0.35,
    'goal_radius': 0.3,
    'cyclic_goals': True,
}


@dataclass
class HuNavAgent:
    """One pedestrian, normalised to HuNavSim's ``hunav_loader`` fields."""
    id: int
    init_pose: dict                     # {x, y, z, h}
    goals: list                         # [{x, y, h}, ...]
    skin: int = 0
    behavior: int = 1
    group_id: int = -1
    max_vel: float = 1.5
    radius: float = 0.35
    goal_radius: float = 0.3
    cyclic_goals: bool = True

    def to_loader_dict(self) -> dict:
        """The per-agent dict HuNavSim's ``hunav_loader`` expects.

        Goals are named ``g0, g1, ...`` with a ``goals`` index list, matching
        the upstream agents.yaml layout.
        """
        goal_names = [f'g{i}' for i in range(len(self.goals))]
        d = {
            'id': self.id, 'skin': self.skin, 'behavior': self.behavior,
            'group_id': self.group_id, 'max_vel': self.max_vel,
            'radius': self.radius, 'init_pose': dict(self.init_pose),
            'goal_radius': self.goal_radius, 'cyclic_goals': self.cyclic_goals,
            'goals': goal_names,
        }
        for name, g in zip(goal_names, self.goals):
            d[name] = dict(g)
        return d


def _norm_pose3(v) -> dict:
    """Coerce an init/goal pose ({x,y[,z],h} dict or [x,y[,h]] list) to a dict."""
    if isinstance(v, dict):
        return {'x': float(v.get('x', 0.0)), 'y': float(v.get('y', 0.0)),
                'z': float(v.get('z', 0.0)), 'h': float(v.get('h', 0.0))}
    x, y, h = _as_pose(v)               # reuse the base [x,y,yaw] coercion
    return {'x': x, 'y': y, 'z': 0.0, 'h': h}


def _norm_goal(v) -> dict:
    p = _norm_pose3(v)
    return {'x': p['x'], 'y': p['y'], 'h': p['h']}


def parse_agent(raw: dict, idx: int) -> HuNavAgent:
    """Normalise one raw agent dict into a :class:`HuNavAgent` (defaults filled)."""
    d = dict(_AGENT_DEFAULTS)
    d.update({k: v for k, v in raw.items()
              if k not in ('init_pose', 'goals')})
    init_pose = _norm_pose3(raw.get('init_pose', [0.0, 0.0]))
    goals = [_norm_goal(g) for g in (raw.get('goals') or [])]
    if not goals:                        # a pedestrian needs at least one goal
        goals = [_norm_goal([init_pose['x'], init_pose['y']])]
    return HuNavAgent(
        id=int(raw.get('id', idx + 1)), init_pose=init_pose, goals=goals,
        skin=int(d['skin']), behavior=int(d['behavior']),
        group_id=int(d['group_id']), max_vel=float(d['max_vel']),
        radius=float(d['radius']), goal_radius=float(d['goal_radius']),
        cyclic_goals=bool(d['cyclic_goals']))


def load_hunav_scenario(path: str) -> Scenario:
    """Load a HuNavSim scenario YAML into a :class:`Scenario` (tier='hunav').

    Map/world relative paths resolve next to the file, exactly like
    ``scenario.load_scenario``. The normalised agents (HuNavSim field names) are
    stored on ``Scenario.agents``; ``meta['hunav']`` flags the ROS2-native source.
    """
    with open(path) as f:
        doc = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(path))

    def resolve(p):
        if not p:
            return None
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(base, p))

    fname = os.path.basename(path)
    if fname.endswith('.scenario.yaml'):
        default_name = fname[:-len('.scenario.yaml')]
    else:
        default_name = os.path.splitext(fname)[0]

    agents = [parse_agent(a, i)
              for i, a in enumerate(doc.get('agents', []) or [])]
    meta = dict(doc.get('meta', {}) or {})
    meta['hunav'] = {'ros2_native': True, 'n_agents': len(agents)}

    return Scenario(
        name=doc.get('name') or default_name,
        tier=doc.get('tier') or 'hunav',
        map_yaml=resolve(doc['map']),
        start=_as_pose(doc['start']),
        goal=_as_pose(doc['goal']),
        world=resolve(doc.get('world')),
        agents=[a.to_loader_dict() for a in agents],
        optimal_length=doc.get('optimal_length'),
        optimal_time=doc.get('optimal_time'),
        meta=meta,
    )


def hunav_loader_params(scenario: Scenario, *,
                        publish_people: bool = True) -> dict:
    """Build the ``hunav_loader`` ROS2 parameter block for a loaded scenario.

    Mirrors HuNavSim's ``agents.yaml``: a ``hunav_loader.ros__parameters`` block
    with ``map``, ``publish_people``, an ``agents`` index list (``agent1, ...``),
    and one sub-block per agent. The workstation launch passes this to HuNavSim.
    """
    names = [f'agent{i + 1}' for i in range(len(scenario.agents))]
    params: dict = {
        'map': os.path.basename(scenario.map_yaml) if scenario.map_yaml else '',
        'publish_people': publish_people,
        'agents': names,
    }
    for name, agent in zip(names, scenario.agents):
        params[name] = dict(agent)
    return {'hunav_loader': {'ros__parameters': params}}
