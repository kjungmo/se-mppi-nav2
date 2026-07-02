#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang
# Licensed under the Apache License, Version 2.0.
#
# Offline goal finder for the tb3_sandbox smoke test. Loads the static map
# (pgm + yaml), erodes the free space by the robot radius, flood-fills the
# reachable component from the true spawn, and reports goals that are provably
# reachable and clear of walls — so a smoke drive does not fail on a goal that
# sits in/behind a wall or a start pinned against one. Pure stdlib (no ROS).
#
# Usage:
#   python3 experiments/sim/pick_goal.py [map_yaml] [start_x] [start_y]
# Defaults: nav2_bringup tb3_sandbox map, spawn (-2.0, -0.5).

import collections
import math
import os
import subprocess
import sys


def find_map_yaml():
    try:
        share = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'nav2_bringup'], text=True).strip()
        cand = os.path.join(share, 'share', 'nav2_bringup', 'maps', 'tb3_sandbox.yaml')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    return None


def parse_yaml(path):
    # Minimal YAML for the fields a map server uses.
    d = {}
    with open(path) as f:
        for line in f:
            line = line.split('#', 1)[0].rstrip()
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            k, v = k.strip(), v.strip()
            if not v:
                continue
            if v.startswith('['):
                v = [float(x) for x in v.strip('[]').split(',')]
            else:
                try:
                    v = float(v)
                except ValueError:
                    pass
            d[k] = v
    return d


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    assert data[:2] == b'P5', 'only binary PGM (P5) supported'
    # Parse header tokens skipping comments.
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
    idx += 1  # single whitespace after maxval
    pix = data[idx:idx + width * height]
    return width, height, pix


def main():
    map_yaml = sys.argv[1] if len(sys.argv) > 1 else find_map_yaml()
    start_x = float(sys.argv[2]) if len(sys.argv) > 2 else -2.0
    start_y = float(sys.argv[3]) if len(sys.argv) > 3 else -0.5
    if not map_yaml or not os.path.exists(map_yaml):
        print('MAP_YAML not found; pass it as the first argument.')
        sys.exit(2)

    meta = parse_yaml(map_yaml)
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    negate = int(meta.get('negate', 0))
    occ_th = meta.get('occupied_thresh', 0.65)
    free_th = meta.get('free_thresh', 0.25)
    pgm = os.path.join(os.path.dirname(map_yaml), meta['image'])
    w, h, pix = read_pgm(pgm)
    print(f'map={os.path.basename(pgm)} {w}x{h} res={res} origin=({ox},{oy})')

    # free[row][col] True if traversable (free), with map row 0 = top (max y).
    def is_free(px):
        p = (255 - px) / 255.0 if negate == 0 else px / 255.0
        return p < free_th  # strictly free; unknown/occupied excluded

    free = [[is_free(pix[r * w + c]) for c in range(w)] for r in range(h)]

    # Erode by the robot radius so a goal/path keeps the footprint clear.
    robot_radius = 0.22
    er = max(1, int(math.ceil(robot_radius / res)))
    safe = [[False] * w for _ in range(h)]
    for r in range(h):
        for c in range(w):
            if not free[r][c]:
                continue
            ok = True
            for dr in range(-er, er + 1):
                for dc in range(-er, er + 1):
                    if dr * dr + dc * dc > er * er:
                        continue
                    rr, cc = r + dr, c + dc
                    if rr < 0 or rr >= h or cc < 0 or cc >= w or not free[rr][cc]:
                        ok = False
                        break
                if not ok:
                    break
            safe[r][c] = ok

    def world_to_cell(x, y):
        c = int((x - ox) / res)
        r = h - 1 - int((y - oy) / res)
        return r, c

    def cell_to_world(r, c):
        x = ox + (c + 0.5) * res
        y = oy + (h - 1 - r + 0.5) * res
        return x, y

    sr, sc = world_to_cell(start_x, start_y)
    print(f'start ({start_x},{start_y}) -> cell (r={sr},c={sc}) '
          f'free={free[sr][sc]} safe={safe[sr][sc]}')

    # If the exact start is not safe (pinned near a wall), snap to the nearest
    # safe cell so the flood has a seed — the robot can still nudge into it.
    seed = (sr, sc)
    if not safe[sr][sc]:
        best = None
        for r in range(h):
            for c in range(w):
                if safe[r][c]:
                    d = (r - sr) ** 2 + (c - sc) ** 2
                    if best is None or d < best[0]:
                        best = (d, r, c)
        if best:
            seed = (best[1], best[2])
            sx, sy = cell_to_world(*seed)
            print(f'start not safe; nearest safe seed cell -> world ({sx:.2f},{sy:.2f})')

    # Flood-fill reachable safe component from the seed (4-connectivity).
    reach = [[False] * w for _ in range(h)]
    q = collections.deque([seed])
    reach[seed[0]][seed[1]] = True
    cells = []
    while q:
        r, c = q.popleft()
        cells.append((r, c))
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and safe[rr][cc] and not reach[rr][cc]:
                reach[rr][cc] = True
                q.append((rr, cc))

    if not cells:
        print('NO reachable safe cells from start — start is boxed in.')
        sys.exit(1)

    xs = [cell_to_world(r, c) for r, c in cells]
    bx = [p[0] for p in xs]
    by = [p[1] for p in xs]
    print(f'reachable safe cells={len(cells)}  '
          f'bbox x[{min(bx):.2f},{max(bx):.2f}] y[{min(by):.2f},{max(by):.2f}]')

    # Check the current default goal.
    gx, gy = 0.9, -2.25
    gr, gc = world_to_cell(gx, gy)
    in_bounds = 0 <= gr < h and 0 <= gc < w
    print(f'current goal ({gx},{gy}) -> '
          f'free={free[gr][gc] if in_bounds else "OOB"} '
          f'safe={safe[gr][gc] if in_bounds else "OOB"} '
          f'reachable={reach[gr][gc] if in_bounds else "OOB"}')

    # Suggest goals: farthest reachable safe cell overall, and the farthest one
    # that is forward (+x) of the start so the robot need not reverse to begin.
    def dist(rc):
        x, y = cell_to_world(*rc)
        return math.hypot(x - start_x, y - start_y)

    farthest = max(cells, key=dist)
    fx, fy = cell_to_world(*farthest)
    print(f'SUGGEST_FARTHEST x={fx:.2f} y={fy:.2f}  (dist {dist(farthest):.2f} m)')

    forward = [rc for rc in cells if cell_to_world(*rc)[0] > start_x + 0.3]
    if forward:
        ff = max(forward, key=dist)
        ffx, ffy = cell_to_world(*ff)
        print(f'SUGGEST_FORWARD  x={ffx:.2f} y={ffy:.2f}  (dist {dist(ff):.2f} m)')
    # A mid-range forward goal (~60% out) is the most robust first smoke target.
    if forward:
        forward.sort(key=dist)
        mid = forward[int(len(forward) * 0.6)]
        mx, my = cell_to_world(*mid)
        print(f'SUGGEST_MID      x={mx:.2f} y={my:.2f}  (dist {dist(mid):.2f} m)')


if __name__ == '__main__':
    main()
