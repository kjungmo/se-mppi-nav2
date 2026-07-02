# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""DynaBARN (T2 dynamic, non-social) scenario loader (design §4, H-5; protocol §5).

DynaBARN extends the BARN procedural worlds (300 static congestion maps) with
moving obstacles. Unlike HuNavSim, **BARN/DynaBARN are ROS1-canonical** — the
worlds and the dynamic-obstacle controllers ship as ROS1 (Gazebo-classic)
assets. So this loader does the part that is well-defined offline — turn a
DynaBARN scenario YAML into a :class:`Scenario` with its moving-obstacle specs —
and **flags the ROS1→ROS2 bridge as a workstation-side dependency** via
``meta['requires_ros1_bridge']`` rather than pretending to run it here.

Input scenario YAML (base schema + a ``dynamic_obstacles`` list):

    name: dynabarn_world_312
    tier: dynabarn
    map: world_312.yaml
    world: world_312.world          # ROS1/Gazebo-classic world (bridged)
    start: [-2.0, 0.0]
    goal:  [ 2.0, 0.0]
    barn_index: 312                 # optional BARN/DynaBARN world id
    dynamic_obstacles:              # moving obstacles (CV motion)
      - {id: 0, shape: cylinder, radius: 0.3,
         init: [0.0, 1.0], velocity: [0.0, -0.5]}
      - {id: 1, shape: box, size: [0.4, 0.4],
         waypoints: [[1.0, 1.0], [1.0, -1.0]], speed: 0.6}

Each obstacle is normalised onto ``Scenario.agents`` (the runner's generic
dynamic-agent slot) with a ``kind: dynamic_obstacle`` tag so metrics/GT sampling
can tell them from HuNav pedestrians. The straight-CV ``velocity`` form matches
the constant-velocity prediction the controller assumes (protocol §3, C3).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import yaml

from .scenario import Scenario, _as_pose

# Marker the workstation reads to know it must bridge ROS1 assets.
ROS1_BRIDGE_NOTE = (
    'BARN/DynaBARN worlds are ROS1/Gazebo-classic canonical; the workstation '
    'must provide a ros1_bridge (or a ROS2 re-export of the world + the moving '
    'obstacle controllers) before this scenario can launch. design §9 / '
    'protocol §7.')


@dataclass
class DynamicObstacle:
    """One moving obstacle, normalised for the runner / GT collision sampling."""
    id: int
    init: tuple                          # (x, y)
    shape: str = 'cylinder'
    radius: float = 0.3
    size: list = field(default_factory=list)     # box [w, l] if shape=box
    velocity: tuple | None = None        # (vx, vy) constant-velocity form
    waypoints: list = field(default_factory=list)  # [[x,y], ...] patrol form
    speed: float | None = None           # along-waypoint speed (patrol form)

    def effective_radius(self) -> float:
        """A single radius for clearance math (box → half its diagonal)."""
        if self.shape == 'box' and self.size:
            w = float(self.size[0])
            l = float(self.size[1]) if len(self.size) > 1 else w
            return 0.5 * (w * w + l * l) ** 0.5
        return float(self.radius)

    def to_agent_dict(self) -> dict:
        d = {'kind': 'dynamic_obstacle', 'id': self.id, 'shape': self.shape,
             'init': list(self.init), 'radius': self.effective_radius()}
        if self.size:
            d['size'] = list(self.size)
        if self.velocity is not None:
            d['velocity'] = list(self.velocity)
        if self.waypoints:
            d['waypoints'] = [list(w) for w in self.waypoints]
        if self.speed is not None:
            d['speed'] = self.speed
        return d


def _xy(v) -> tuple:
    x, y, _ = _as_pose(v)
    return (x, y)


def parse_obstacle(raw: dict, idx: int) -> DynamicObstacle:
    """Normalise one raw moving-obstacle dict into a :class:`DynamicObstacle`."""
    shape = raw.get('shape', 'cylinder')
    init = _xy(raw.get('init', raw.get('init_pose', [0.0, 0.0])))
    velocity = tuple(raw['velocity']) if raw.get('velocity') is not None else None
    waypoints = [list(w) for w in (raw.get('waypoints') or [])]
    return DynamicObstacle(
        id=int(raw.get('id', idx)), init=init, shape=shape,
        radius=float(raw.get('radius', 0.3)),
        size=list(raw.get('size', []) or []),
        velocity=velocity, waypoints=waypoints,
        speed=(float(raw['speed']) if raw.get('speed') is not None else None))


def load_dynabarn_scenario(path: str) -> Scenario:
    """Load a DynaBARN scenario YAML into a :class:`Scenario` (tier='dynabarn').

    The dynamic obstacles land on ``Scenario.agents``; ``meta`` records the BARN
    world index (if any) and the ROS1-bridge requirement so a workstation run
    fails loudly-but-clearly rather than silently launching an empty world.
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

    obstacles = [parse_obstacle(o, i)
                 for i, o in enumerate(doc.get('dynamic_obstacles', []) or [])]
    meta = dict(doc.get('meta', {}) or {})
    meta['requires_ros1_bridge'] = True
    meta['ros1_bridge_note'] = ROS1_BRIDGE_NOTE
    if doc.get('barn_index') is not None:
        meta['barn_index'] = int(doc['barn_index'])
    meta['dynabarn'] = {'n_dynamic_obstacles': len(obstacles)}

    return Scenario(
        name=doc.get('name') or default_name,
        tier=doc.get('tier') or 'dynabarn',
        map_yaml=resolve(doc['map']),
        start=_as_pose(doc['start']),
        goal=_as_pose(doc['goal']),
        world=resolve(doc.get('world')),
        agents=[o.to_agent_dict() for o in obstacles],
        optimal_length=doc.get('optimal_length'),
        optimal_time=doc.get('optimal_time'),
        meta=meta,
    )
