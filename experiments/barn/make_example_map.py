#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Generate a small synthetic BARN-like map + scenario as a runnable fixture.

This gives the harness a self-contained scenario that ``scenario.validate``
passes offline (no ROS, no nav2_bringup share), so ``discover`` → ``validate``
→ ``run_suite`` can be exercised end-to-end with the FakeLauncher. The layout is
a narrow corridor with a U-shaped dead-end pocket (a local-minimum trap, the
EscapeCritic's reason for being); the goal sits past the pocket and is reachable
by going around it.

Run: ``python3 experiments/barn/make_example_map.py``
"""

import os

import yaml

RES = 0.1
W = H = 50              # 5 m × 5 m
FREE, WALL = 254, 0


def build():
    grid = [[FREE] * W for _ in range(H)]

    def wall_rect(r0, r1, c0, c1):
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                grid[r][c] = WALL

    # Outer border.
    wall_rect(0, H - 1, 0, 1)
    wall_rect(0, H - 1, W - 2, W - 1)
    wall_rect(0, 1, 0, W - 1)
    wall_rect(H - 2, H - 1, 0, W - 1)

    # U-trap pocket in the lower-middle, opening upward (toward +y / lower rows).
    # Three walls of a box; the open side faces the start so a greedy planner
    # can drive in and stall — the escape behaviour must back out and go around.
    wall_rect(28, 30, 18, 32)     # bottom of the U
    wall_rect(20, 30, 18, 20)     # left wall of the U
    wall_rect(20, 30, 30, 32)     # right wall of the U
    return grid


def write_pgm(grid, path):
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        flat = bytearray()
        for row in grid:
            flat.extend(row)
        f.write(bytes(flat))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    grid = build()
    write_pgm(grid, os.path.join(here, 'example_utrap.pgm'))
    with open(os.path.join(here, 'example_utrap.yaml'), 'w') as f:
        yaml.safe_dump({
            'image': 'example_utrap.pgm', 'resolution': RES,
            'origin': [0.0, 0.0, 0.0], 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.25}, f)
    # Start above the pocket opening; goal below/around it, both in free space.
    with open(os.path.join(here, 'example_utrap.scenario.yaml'), 'w') as f:
        yaml.safe_dump({
            'name': 'example_utrap', 'tier': 'barn',
            'map': 'example_utrap.yaml',
            'start': [2.5, 3.5], 'goal': [2.5, 0.7],
            'meta': {'kind': 'u-trap',
                     'note': 'local-minimum pocket; escape must back out'}}, f)
    print('wrote example_utrap.{pgm,yaml,scenario.yaml}')


if __name__ == '__main__':
    main()
