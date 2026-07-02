# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Tests for scenario loading + reachability validation (design §4, §7)."""

import os

import yaml

from experiments.runner import scenario as sc
from experiments.runner.gridmap import GridMap, reachable


def _write_pgm(path, w, h, walls):
    """Write a P5 PGM: free=254, wall=0; ``walls`` is a set of (r, c)."""
    pix = bytearray([254] * (w * h))
    for (r, c) in walls:
        pix[r * w + c] = 0
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(bytes(pix))


def _write_map_yaml(path, image, res=0.1):
    with open(path, 'w') as f:
        yaml.safe_dump({'image': image, 'resolution': res,
                        'origin': [0.0, 0.0, 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.25}, f)


def test_open_map_goal_reachable(tmp_path):
    w = h = 40
    _write_pgm(tmp_path / 'open.pgm', w, h, walls=set())
    _write_map_yaml(tmp_path / 'open.yaml', 'open.pgm')
    gm = GridMap.from_yaml(str(tmp_path / 'open.yaml'))
    assert reachable(gm, (1.0, 2.0), (3.0, 2.0), robot_radius=0.22)


def test_wall_blocks_goal(tmp_path):
    w = h = 40
    wall = {(r, 20) for r in range(h)}      # solid column splits the map
    _write_pgm(tmp_path / 'split.pgm', w, h, walls=wall)
    _write_map_yaml(tmp_path / 'split.yaml', 'split.pgm')
    gm = GridMap.from_yaml(str(tmp_path / 'split.yaml'))
    # Start left of wall, goal right of wall -> unreachable.
    assert not reachable(gm, (1.0, 2.0), (3.0, 2.0), robot_radius=0.22)


def test_load_scenario_and_validate(tmp_path):
    w = h = 40
    _write_pgm(tmp_path / 'open.pgm', w, h, walls=set())
    _write_map_yaml(tmp_path / 'open.yaml', 'open.pgm')
    scen_path = tmp_path / 'demo.scenario.yaml'
    with open(scen_path, 'w') as f:
        yaml.safe_dump({'name': 'demo', 'map': 'open.yaml',
                        'start': [1.0, 2.0], 'goal': [3.0, 2.0, 1.57]}, f)
    scenario = sc.load_scenario(str(scen_path), tier='barn')
    assert scenario.name == 'demo'
    assert scenario.tier == 'barn'
    assert scenario.start == (1.0, 2.0, 0.0)
    assert scenario.goal == (3.0, 2.0, 1.57)
    ok, reason = sc.validate(scenario)
    assert ok, reason


def test_validate_missing_map(tmp_path):
    scen = sc.Scenario(name='x', tier='barn',
                       map_yaml=str(tmp_path / 'nope.yaml'),
                       start=(0.0, 0.0, 0.0), goal=(1.0, 0.0, 0.0))
    ok, reason = sc.validate(scen)
    assert not ok and 'not found' in reason


def test_discover_finds_scenarios(tmp_path):
    tier_dir = tmp_path / 'barn'
    tier_dir.mkdir()
    _write_pgm(tier_dir / 'open.pgm', 40, 40, walls=set())
    _write_map_yaml(tier_dir / 'open.yaml', 'open.pgm')
    for nm in ('s1', 's2'):
        with open(tier_dir / f'{nm}.scenario.yaml', 'w') as f:
            yaml.safe_dump({'map': 'open.yaml', 'start': [1.0, 2.0],
                            'goal': [3.0, 2.0]}, f)
    found = sc.discover(str(tmp_path), tier='barn')
    assert {s.name for s in found} == {'s1', 's2'}
