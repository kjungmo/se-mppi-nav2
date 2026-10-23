"""Verify s4_ltrap.sdf geometry without Gazebo: parse SDF, plot, flood fill."""
import math
import pathlib
import re
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

here = pathlib.Path(__file__).parent
root = ET.parse(here / 'worlds' / 's4_ltrap.sdf').getroot()
boxes = {}
for m in root.iter('model'):
    n = m.get('name')
    if n == 'ground_plane':
        continue
    pose = [float(v) for v in m.find('pose').text.split()]
    sz = [float(v) for v in m.find('.//collision//size').text.split()]
    boxes[n] = (pose[0], pose[1], pose[5], sz[0], sz[1])

def corners(b):
    x, y, th, L, T = b
    c, s = math.cos(th), math.sin(th)
    return [(x + c * a - s * d, y + s * a + c * d)
            for a, d in ((-L/2, -T/2), (L/2, -T/2), (L/2, T/2), (-L/2, T/2))]

def centreline(b):
    x, y, th, L, T = b
    return ((x - L/2*math.cos(th), y - L/2*math.sin(th)),
            (x + L/2*math.cos(th), y + L/2*math.sin(th)))

# occupancy grid, 5 cm, over [-4.2, 4.2]^2 (perimeter included)
res = 0.05
x0 = -4.2
N = int(round(8.4 / res))
xs = x0 + (np.arange(N) + 0.5) * res
X, Y = np.meshgrid(xs, xs)
occ = np.zeros((N, N), bool)
for b in boxes.values():
    x, y, th, L, T = b
    c, s = math.cos(th), math.sin(th)
    u = (X - x) * c + (Y - y) * s
    v = -(X - x) * s + (Y - y) * c
    occ |= (abs(u) <= L / 2) & (abs(v) <= T / 2)

def idx(p):
    return int((p[1] - x0) / res), int((p[0] - x0) / res)

def flood(seed, blocked):
    seen = np.zeros_like(occ)
    q = deque([seed]); seen[seed] = True
    while q:
        r, c = q.popleft()
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < N and 0 <= cc < N and not seen[rr, cc] and not blocked[rr, cc]:
                seen[rr, cc] = True; q.append((rr, cc))
    return seen

start, goal = (-3.0, 0.0), (3.0, 0.0)
YAW = math.radians(-15.0); MC = (-0.6, 0.0)
def to_pocket(p):
    c, s = math.cos(-YAW), math.sin(-YAW)
    dx, dy = p[0] - MC[0], p[1] - MC[1]
    return (c * dx - s * dy, s * dx + c * dy)

# 1) full-world reachability start -> goal around the structure (sanity)
full = flood(idx(start), occ)
print('goal reachable around structure (full world):', bool(full[idx(goal)]))

# 2) pocket enclosure: seal the mouth with a line, flood from inside the pocket
pm = lambda q: (MC[0] + math.cos(YAW)*q[0] - math.sin(YAW)*q[1],
                MC[1] + math.sin(YAW)*q[0] + math.cos(YAW)*q[1])
sealed = occ.copy()
for t in np.linspace(-0.8, 0.8, 400):
    r, c = idx(pm((0.0, t)))
    sealed[max(r-1,0):r+2, max(c-1,0):c+2] = True   # 3x3 cell dilation of the seal line
# seed deep in corridor and in the leg
seeds = {'corridor': pm((1.0, 0.0)), 'leg': pm((1.8, 1.4))}
for name, p in seeds.items():
    seen = flood(idx(p), sealed)
    area = seen.sum() * res * res
    print(f'sealed-mouth flood from {name} seed: area={area:.3f} m^2, reaches goal={bool(seen[idx(goal)])}, '
          f'reaches start={bool(seen[idx(start)])}, touches perimeter cells={bool(seen[:2].any() or seen[-2:].any() or seen[:, :2].any() or seen[:, -2:].any())}')
    if name == 'corridor':
        print('  leg seed also enclosed with corridor:', bool(seen[idx(seeds['leg'])]))
print('Expected pocket interior area (1.3x2.2 corridor + leg approx):',
      round(1.3*2.05 + 0.9*1.2, 3), 'order of magnitude check only')

# 3) straight line start->goal vs structure
line = np.array([[start[0] + t*(goal[0]-start[0]), start[1]] for t in np.linspace(0, 1, 6001)])
hit = [i for i, p in enumerate(line) if occ[idx(p)]]
# line in pocket frame
def inside_pocket_x(p):
    return to_pocket(p)[0]
print('line y=0 first occupied point:', tuple(np.round(line[hit[0]], 3)) if hit else None)
# entry: where the line crosses the mouth plane (pocket x = 0)
for t in np.linspace(0, 1, 600001):
    p = (start[0] + t*(goal[0]-start[0]), 0.0)
    if to_pocket(p)[0] >= 0:
        q = to_pocket(p)
        print('line crosses mouth plane at world', np.round(p, 4), 'pocket-frame y =', round(q[1], 4))
        enter = p; break
# end of corridor depth along the line: back wall inner face is pocket x = 2.2 - T/2
# solve line y=0 with pocket x = 2.05
# pocket x = cos(th)*(x-MC0) - sin(th)*(0-0)... compute directly
# to_pocket x = cos(-YAW)*dx - sin(-YAW)*dy ; dy=0 -> x_p = cos(YAW)*(x+0.6)
xe_back = -0.6 + 2.05 / math.cos(YAW)
xe_mouth = -0.6
print('line depth in corridor along the line (mouth plane to back-wall inner face):',
      round(xe_back - xe_mouth, 4), 'm (pocket depth 2.05 m inner, 2.2 m centreline)')
# exact: pocket y of world point (xe_back,0): -sin(-YAW)*dx
print('pocket y at back wall inner face:', round(-math.sin(-YAW)*(xe_back+0.6), 4),
      '(corridor inner half width 0.65)')

# 4) mouth gap
a = pm((0.0, -0.8)); b = pm((0.0, 0.8))
print('mouth centreline gap:', round(math.dist(a, b), 4), 'm; clear gap between wall inner faces:', round(math.dist(a, b) - 0.3, 4), 'm')
# grid-measured clear gap at mouth plane (free cells in pocket x in [0,0.05])
col_mask = np.zeros_like(occ)
cnt = 0
for t in np.arange(-0.9, 0.9, 0.001):
    p = pm((0.02, t))
    if not occ[idx(p)]:
        cnt += 1
print('grid-measured free width at pocket x=0.02:', round(cnt*0.001, 3), 'm')

# plot
fig, ax = plt.subplots(figsize=(7, 7))
ax.imshow(occ, origin='lower', extent=[x0, x0 + N*res]*2, cmap='Greys', alpha=0.4)
for n, b in boxes.items():
    pc = corners(b); pc.append(pc[0])
    ax.plot(*zip(*pc), 'k-', lw=0.8)
    if n.startswith('l_'):
        ca, cb = centreline(b)
        ax.plot([ca[0], cb[0]], [ca[1], cb[1]], 'r--', lw=0.8)
        ax.text(b[0], b[1], n, fontsize=7, color='r', ha='center')
ax.plot([start[0], goal[0]], [start[1], goal[1]], 'b-', lw=1.5, label='start-goal line')
ax.plot(*start, 'go', label='start'); ax.plot(*goal, 'm*', ms=12, label='goal')
ax.plot(*MC, 'cx', label='mouth centre')
ax.set_aspect('equal'); ax.set_xlim(-4.2, 4.2); ax.set_ylim(-4.2, 4.2)
ax.grid(alpha=0.3); ax.legend(loc='lower right', fontsize=8)
ax.set_title('s4_ltrap: wall footprints (black), centrelines (red dashed)')
fig.savefig(here / 's4_ltrap_geometry.png', dpi=130, bbox_inches='tight')
for n, b in boxes.items():
    if n.startswith('l_'):
        print(n, 'centre=(%.4f,%.4f) yaw=%.4f len=%.4f' % (b[0], b[1], b[2], b[3]))
