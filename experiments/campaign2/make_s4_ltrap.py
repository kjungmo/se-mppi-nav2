"""Generate worlds/s4_ltrap.sdf and s4_ltrap.scenario.yaml (L-shaped dead end)."""
import math
import pathlib

here = pathlib.Path(__file__).parent
tpl = (here / '../se_mppi_utrap/worlds/testbed.sdf').resolve().read_text().splitlines()
# keep everything before the first u_ model; the u_* pocket is replaced
i = next(k for k, l in enumerate(tpl) if 'name="u_back"' in l)
head = tpl[:i]
T, H = 0.3, 1.0
YAW = math.radians(-15.0)
MOUTH = (-0.6, 0.0)
# pocket-frame centreline segments: name: (p0, p1, pad_at_p0, pad_at_p1)
# pad = extend by T/2 so corners close; the mouth ends are not padded.
SEGS = {
    'l_bot':   ((0.0, -0.8), (2.2, -0.8), False, True),
    'l_top':   ((0.0,  0.8), (1.4,  0.8), False, True),
    'l_inner': ((1.4,  0.8), (1.4,  2.0), True,  True),
    'l_cap':   ((1.4,  2.0), (2.2,  2.0), True,  True),
    'l_back':  ((2.2, -0.8), (2.2,  2.0), True,  True),
}


def to_world(p):
    c, s = math.cos(YAW), math.sin(YAW)
    return (MOUTH[0] + c * p[0] - s * p[1], MOUTH[1] + s * p[0] + c * p[1])


def models():
    out = []
    for name, (a, b, pa, pb) in SEGS.items():
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        a2 = (a[0] - ux * T / 2 * pa, a[1] - uy * T / 2 * pa)
        b2 = (b[0] + ux * T / 2 * pb, b[1] + uy * T / 2 * pb)
        cen = to_world(((a2[0] + b2[0]) / 2, (a2[1] + b2[1]) / 2))
        yaw = math.atan2(dy, dx) + YAW
        out.append((name, cen, yaw, L + T / 2 * (pa + pb)))
    return out


def sdf_model(name, cen, yaw, L):
    box = f'<size>{L:.6f} {T:.6f} {H:.6f}</size>'
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{cen[0]:.6f} {cen[1]:.6f} {H / 2:.6f} 0.000000 0.000000 {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              {box}
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              {box}
            </box>
          </geometry>
          <material>
            <ambient>0.750 0.500 0.300 1</ambient>
            <diffuse>0.750 0.500 0.300 1</diffuse>
            <specular>0.850 0.600 0.400 1</specular>
          </material>
        </visual>
      </link>
    </model>'''


SCEN = '''# S4 - L-shaped dead end. Pocket mouth faces -x; corridor 1.6 m wide, 2.2 m deep;
# the top wall turns into a closed 1.2 m leg along +y at the far end (dead end,
# not a passage). Structure translated so the mouth centre is at (-0.6, 0) and
# rotated -15 deg about the mouth centre. Walls are physical in the gz world
# but absent from the localization map (empty box), as in s1.
name: s4_ltrap
tier: campaign2
world: worlds/s4_ltrap.sdf
map: ../se_mppi_utrap/maps/utrap_loc.yaml
start: [-3.0, 0.0, 0.0]
goal:  [3.0, 0.0, 0.0]
meta:
  kind: l-trap
  note: physical L-shaped dead end absent from planner map; local-minimum escape test
  launch:
    pkg: se_mppi_utrap
    file: utrap_bench.launch.py
    pass_map: true
    pass_spawn: true
    use_sim_time: true
'''

if __name__ == '__main__':
    (here / 'worlds').mkdir(exist_ok=True)
    body = '\n'.join(head + [sdf_model(*m) for m in models()] + ['  </world>', '</sdf>']) + '\n'
    (here / 'worlds' / 's4_ltrap.sdf').write_text(body)
    (here / 's4_ltrap.scenario.yaml').write_text(SCEN)
