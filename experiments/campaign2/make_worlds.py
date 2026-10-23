#!/usr/bin/env python3
"""Generate campaign-2 worlds from the U-trap testbed. s0_open: no u_* models.
s3_ctrap: asymmetric pocket (pocket frame, open toward -x), rotated +25 deg about (0.5,0)."""
import math, re, pathlib
HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / "se_mppi_utrap/worlds/testbed.sdf"
T, H = 0.3, 1.0           # thickness, height (template)
XB = 1.3                  # back-wall x (pocket frame)
BACK_L, TOP_L, BOT_L = 2.4, 1.6, 1.1
PIVOT, ROT = (0.5, 0.0), math.radians(25.0)


def pocket_walls():
    """(name, cx, cy, yaw, length) in pocket frame; wall centrelines.
    Side walls sit on the back-wall ends, as in the template (y=+-1 for a 2.0 back wall)."""
    yt, yb = BACK_L / 2, -BACK_L / 2
    return [("c_back", XB, 0.0, math.pi / 2, BACK_L),
            ("c_top", XB - TOP_L / 2, yt, math.pi, TOP_L),
            ("c_bot", XB - BOT_L / 2, yb, math.pi, BOT_L)]


def to_world(cx, cy, yaw):
    dx, dy = cx - PIVOT[0], cy - PIVOT[1]
    c, s = math.cos(ROT), math.sin(ROT)
    return PIVOT[0] + c * dx - s * dy, PIVOT[1] + s * dx + c * dy, yaw + ROT


def model(name, x, y, yaw, L):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} {H/2:.6f} 0.000000 0.000000 {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{L:.6f} {T:.6f} {H:.6f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{L:.6f} {T:.6f} {H:.6f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.750 0.500 0.300 1</ambient>
            <diffuse>0.750 0.500 0.300 1</diffuse>
            <specular>0.850 0.600 0.400 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def main():
    txt = SRC.read_text()
    pat = re.compile(r'    <model name="u_(?:back|top|bot)">.*?\n    </model>\n', re.S)
    assert len(pat.findall(txt)) == 3
    base = pat.sub("", txt)
    out = HERE / "worlds"
    out.mkdir(exist_ok=True)
    (out / "s0_open.sdf").write_text(base)
    new = "".join(model(n, *to_world(cx, cy, yaw), L) for n, cx, cy, yaw, L in pocket_walls())
    assert "  </world>" in base
    (out / "s3_ctrap.sdf").write_text(base.replace("  </world>", new + "  </world>"))


if __name__ == "__main__":
    main()
