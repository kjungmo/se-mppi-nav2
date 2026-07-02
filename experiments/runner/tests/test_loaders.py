# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Offline tests for the H-5 dynamic-tier scenario loaders (DynaBARN + HuNavSim).

Pure YAML → :class:`Scenario` parsing; no ROS. Confirms each loader produces the
fields the runner consumes and the launch-time param blocks use the verified
upstream field names.
"""

import yaml

from experiments.runner import dynabarn, hunav
from experiments.runner import scenario as sc


# --------------------------------------------------------------------------- #
# HuNavSim
# --------------------------------------------------------------------------- #
def _write(path, doc):
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f)
    return str(path)


def test_hunav_loads_agents_with_defaults(tmp_path):
    p = _write(tmp_path / 'h.scenario.yaml', {
        'name': 'crossing', 'tier': 'hunav', 'map': 'm.yaml',
        'world': 'w.sdf', 'start': [0.0, 0.0], 'goal': [6.0, 0.0],
        'agents': [
            {'id': 1, 'init_pose': {'x': 3.0, 'y': 2.0, 'h': 0.0},
             'goals': [{'x': 3.0, 'y': -2.0, 'h': 0.0}]},
            {'init_pose': [1.0, 1.0]},   # minimal: defaults + id auto-filled
        ]})
    scen = hunav.load_hunav_scenario(p)
    assert scen.tier == 'hunav'
    assert scen.goal == (6.0, 0.0, 0.0)
    assert len(scen.agents) == 2
    a0, a1 = scen.agents
    # verified hunav_loader field names present
    assert a0['id'] == 1 and a0['behavior'] == 1 and a0['max_vel'] == 1.5
    assert a0['init_pose'] == {'x': 3.0, 'y': 2.0, 'z': 0.0, 'h': 0.0}
    assert a0['goals'] == ['g0'] and a0['g0'] == {'x': 3.0, 'y': -2.0, 'h': 0.0}
    # minimal agent: id auto = idx+1, fallback goal at its own init pose
    assert a1['id'] == 2 and a1['goals'] == ['g0']
    assert a1['g0'] == {'x': 1.0, 'y': 1.0, 'h': 0.0}
    assert scen.meta['hunav']['n_agents'] == 2


def test_hunav_loader_params_block(tmp_path):
    p = _write(tmp_path / 'h.scenario.yaml', {
        'tier': 'hunav', 'map': 'crossing.yaml', 'start': [0, 0],
        'goal': [5, 0],
        'agents': [{'id': 7, 'init_pose': [2.0, 0.0],
                    'goals': [[2.0, 3.0], [2.0, -3.0]]}]})
    scen = hunav.load_hunav_scenario(p)
    block = hunav.hunav_loader_params(scen)
    params = block['hunav_loader']['ros__parameters']
    assert params['map'] == 'crossing.yaml'
    assert params['publish_people'] is True
    assert params['agents'] == ['agent1']
    a = params['agent1']
    assert a['id'] == 7
    assert a['goals'] == ['g0', 'g1']
    assert a['g0'] == {'x': 2.0, 'y': 3.0, 'h': 0.0}


# --------------------------------------------------------------------------- #
# DynaBARN
# --------------------------------------------------------------------------- #
def test_dynabarn_loads_obstacles_and_flags_bridge(tmp_path):
    p = _write(tmp_path / 'd.scenario.yaml', {
        'name': 'world_312', 'tier': 'dynabarn', 'map': 'w.yaml',
        'world': 'w.world', 'start': [-2.0, 0.0], 'goal': [2.0, 0.0],
        'barn_index': 312,
        'dynamic_obstacles': [
            {'id': 0, 'shape': 'cylinder', 'radius': 0.3,
             'init': [0.0, 1.0], 'velocity': [0.0, -0.5]},
            {'id': 1, 'shape': 'box', 'size': [0.4, 0.4],
             'waypoints': [[1.0, 1.0], [1.0, -1.0]], 'speed': 0.6},
        ]})
    scen = dynabarn.load_dynabarn_scenario(p)
    assert scen.tier == 'dynabarn'
    assert scen.meta['requires_ros1_bridge'] is True
    assert 'ros1_bridge' in scen.meta['ros1_bridge_note']
    assert scen.meta['barn_index'] == 312
    assert len(scen.agents) == 2
    o0, o1 = scen.agents
    assert o0['kind'] == 'dynamic_obstacle'
    assert o0['velocity'] == [0.0, -0.5]
    # box effective radius = half-diagonal of 0.4x0.4
    assert abs(o1['radius'] - 0.2828427) < 1e-4
    assert o1['waypoints'] == [[1.0, 1.0], [1.0, -1.0]] and o1['speed'] == 0.6


def test_dynabarn_box_effective_radius():
    o = dynabarn.DynamicObstacle(id=0, init=(0, 0), shape='box', size=[0.6, 0.8])
    assert abs(o.effective_radius() - 0.5) < 1e-9   # half-diag of 0.6x0.8 = 0.5


# --------------------------------------------------------------------------- #
# discover() dispatches to the right loader per tier
# --------------------------------------------------------------------------- #
def test_discover_dispatches_tier_loaders(tmp_path):
    (tmp_path / 'hunav').mkdir()
    (tmp_path / 'dynabarn').mkdir()
    _write(tmp_path / 'hunav' / 'a.scenario.yaml', {
        'map': 'm.yaml', 'start': [0, 0], 'goal': [3, 0],
        'agents': [{'id': 1, 'init_pose': [1, 1], 'goals': [[1, 2]]}]})
    _write(tmp_path / 'dynabarn' / 'b.scenario.yaml', {
        'map': 'm.yaml', 'start': [0, 0], 'goal': [3, 0],
        'dynamic_obstacles': [{'id': 0, 'init': [1, 1], 'velocity': [0, 0.2]}]})
    found = {s.name: s for s in sc.discover(str(tmp_path))}
    # hunav agent normalised to hunav_loader fields (has init_pose dict)
    assert isinstance(found['a'].agents[0]['init_pose'], dict)
    # dynabarn obstacle tagged + bridge flagged
    assert found['b'].agents[0]['kind'] == 'dynamic_obstacle'
    assert found['b'].meta['requires_ros1_bridge'] is True
