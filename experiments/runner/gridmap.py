# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Occupancy-grid loading and reachability — codifies the start/goal sanity
check from ``experiments/sim/pick_goal.py`` so the runner never launches a
scenario whose goal sits in/behind a wall (a setup-failure that would otherwise
pollute the success denominator; design §7).

Pure stdlib (no ROS, no numpy) so it runs in CI and offline tests.
"""

from __future__ import annotations

import collections
import math
import os
from dataclasses import dataclass

import yaml


def parse_map_yaml(path: str) -> dict:
    """Parse a Nav2 map yaml (image/resolution/origin/thresholds).

    Uses PyYAML so both flow (``origin: [0,0,0]``) and block list styles load.
    """
    with open(path) as f:
        return yaml.safe_load(f)


def read_pgm(path: str):
    """Read a binary (P5) PGM; returns (width, height, bytes)."""
    with open(path, 'rb') as f:
        data = f.read()
    if data[:2] != b'P5':
        raise ValueError('only binary PGM (P5) supported')
    idx = 2
    toks = []
    while len(toks) < 3:
        while idx < len(data) and data[idx] in b' \t\r\n':
            idx += 1
        if data[idx:idx + 1] == b'#':
            while idx < len(data) and data[idx] not in b'\r\n':
                idx += 1
            continue
        start = idx
        while idx < len(data) and data[idx] not in b' \t\r\n':
            idx += 1
        toks.append(data[start:idx])
    width, height, _maxval = (int(t) for t in toks)
    idx += 1
    pix = data[idx:idx + width * height]
    return width, height, pix


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    free: list           # free[r][c] -> bool (traversable)

    @classmethod
    def from_yaml(cls, map_yaml: str) -> 'GridMap':
        meta = parse_map_yaml(map_yaml)
        res = meta['resolution']
        ox, oy = meta['origin'][0], meta['origin'][1]
        negate = int(meta.get('negate', 0))
        free_th = meta.get('free_thresh', 0.25)
        pgm = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta['image'])
        w, h, pix = read_pgm(pgm)

        def is_free(px: int) -> bool:
            p = (255 - px) / 255.0 if negate == 0 else px / 255.0
            return p < free_th

        free = [[is_free(pix[r * w + c]) for c in range(w)] for r in range(h)]
        return cls(w, h, res, ox, oy, free)

    def world_to_cell(self, x: float, y: float):
        c = int((x - self.origin_x) / self.resolution)
        r = self.height - 1 - int((y - self.origin_y) / self.resolution)
        return r, c

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width

    def erode(self, robot_radius: float) -> list:
        """Boolean grid of cells whose disk of ``robot_radius`` stays free."""
        er = max(1, int(math.ceil(robot_radius / self.resolution)))
        safe = [[False] * self.width for _ in range(self.height)]
        for r in range(self.height):
            for c in range(self.width):
                if not self.free[r][c]:
                    continue
                ok = True
                for dr in range(-er, er + 1):
                    for dc in range(-er, er + 1):
                        if dr * dr + dc * dc > er * er:
                            continue
                        rr, cc = r + dr, c + dc
                        if not self.in_bounds(rr, cc) or not self.free[rr][cc]:
                            ok = False
                            break
                    if not ok:
                        break
                safe[r][c] = ok
        return safe


def reachable(gm: GridMap, start_xy, goal_xy, robot_radius: float) -> bool:
    """True if goal is in the same eroded free component as start (4-conn flood)."""
    safe = gm.erode(robot_radius)
    sr, sc = gm.world_to_cell(*start_xy)
    grr, gcc = gm.world_to_cell(*goal_xy)
    if not (gm.in_bounds(sr, sc) and gm.in_bounds(grr, gcc)):
        return False

    # Snap start to nearest safe cell if pinned (robot can nudge into it).
    seed = (sr, sc)
    if not safe[sr][sc]:
        best = None
        for r in range(gm.height):
            for c in range(gm.width):
                if safe[r][c]:
                    d = (r - sr) ** 2 + (c - sc) ** 2
                    if best is None or d < best[0]:
                        best = (d, r, c)
        if best is None:
            return False
        seed = (best[1], best[2])

    if not safe[grr][gcc]:
        return False

    reach = [[False] * gm.width for _ in range(gm.height)]
    q = collections.deque([seed])
    reach[seed[0]][seed[1]] = True
    while q:
        r, c = q.popleft()
        if (r, c) == (grr, gcc):
            return True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if gm.in_bounds(rr, cc) and safe[rr][cc] and not reach[rr][cc]:
                reach[rr][cc] = True
                q.append((rr, cc))
    return reach[grr][gcc]
