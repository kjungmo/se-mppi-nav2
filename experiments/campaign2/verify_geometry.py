#!/usr/bin/env python3
"""Read wall models back from the SDF, compute world-frame segments, plot, report line/pocket numbers."""
import math, pathlib, re, sys
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = pathlib.Path(__file__).resolve().parent
START, GOAL = np.array([-3.0, 0.0]), np.array([3.0, 0.0])


def walls(sdf, prefix):
    out = {}
    for m in ET.parse(sdf).getroot().iter("model"):
        n = m.get("name")
        if not n.startswith(prefix):
            continue
        p = [float(v) for v in m.find("pose").text.split()]
        sz = [float(v) for v in m.find(".//collision//size").text.split()]
        out[n] = (p[0], p[1], p[5], sz[0], sz[1])  # x, y, yaw, length, thickness
    return out


def frame(w):
    x, y, yaw, L, T = w
    u = np.array([math.cos(yaw), math.sin(yaw)]); v = np.array([-u[1], u[0]])
    c = np.array([x, y])
    return c, u, v, L, T


def seg(w):
    c, u, v, L, T = frame(w)
    return c - u * L / 2, c + u * L / 2


def rect(w):
    c, u, v, L, T = frame(w)
    return [c + a * u * L / 2 + b * v * T / 2 for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def line_poly_hits(poly):
    """x-range where the line y=0 is inside the convex polygon."""
    xs = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (a[1] - 0) * (b[1] - 0) <= 0 and a[1] != b[1]:
            t = (0 - a[1]) / (b[1] - a[1])
            xs.append(a[0] + t * (b[0] - a[0]))
    return (min(xs), max(xs)) if xs else None


def seg_dist(p1, p2, q1, q2):
    def pt(p, a, b):
        d = b - a; t = np.clip(np.dot(p - a, d) / np.dot(d, d), 0, 1); return np.linalg.norm(p - (a + t * d))
    return min(pt(p1, q1, q2), pt(p2, q1, q2), pt(q1, p1, p2), pt(q2, p1, p2))


def poly_dist(P, Q):
    best = 1e9
    for i in range(4):
        for j in range(4):
            best = min(best, seg_dist(P[i], P[(i + 1) % 4], Q[j], Q[(j + 1) % 4]))
    return best


def main():
    sdf = HERE / "worlds/s3_ctrap.sdf"
    W = walls(sdf, "c_")
    assert len(W) == 3
    fig, ax = plt.subplots(figsize=(8, 6))
    for name, w in W.items():
        P = np.array(rect(w)); ax.fill(P[:, 0], P[:, 1], alpha=0.4, label=name)
        a, b = seg(w); ax.plot([a[0], b[0]], [a[1], b[1]], "k-", lw=1)
        print(f"{name}: centreline ({a[0]:.4f},{a[1]:.4f}) -> ({b[0]:.4f},{b[1]:.4f}); pose xy=({w[0]:.4f},{w[1]:.4f}) yaw={math.degrees(w[2]):.2f}deg L={w[3]}")
    ax.plot([START[0], GOAL[0]], [START[1], GOAL[1]], "r--", label="start-goal line")
    ax.plot(*START, "go"); ax.plot(*GOAL, "bs")
    # per-wall line interaction with thick walls
    back = W["c_back"]; top = W["c_top"]; bot = W["c_bot"]
    hit = line_poly_hits(rect(back))
    print(f"line y=0 inside back wall slab: x in [{hit[0]:.4f}, {hit[1]:.4f}] (near face {hit[0]:.4f})")
    for n in ("c_top", "c_bot"):
        print(f"{n} slab intersects y=0:", line_poly_hits(rect(W[n])))
    # mouth: line joining the free (-x) ends of the side walls
    free = {}
    for n in ("c_top", "c_bot"):
        a, b = seg(W[n]); free[n] = a if a[0] < b[0] else b  # pocket opens toward -x (rotated: use the end further from back wall)
        back_c = frame(back)[0]
        free[n] = a if np.linalg.norm(a - back_c) > np.linalg.norm(b - back_c) else b
    ft, fb = free["c_top"], free["c_bot"]
    d = fb - ft
    t = (0 - ft[1]) / d[1]
    mouth_pt = ft + t * d
    print(f"mouth free-end centrelines: top ({ft[0]:.4f},{ft[1]:.4f}) bottom ({fb[0]:.4f},{fb[1]:.4f}); line meets mouth chord at x={mouth_pt[0]:.4f}, y=0")
    # inner-face (pocket-interior) corners of the free ends
    def inner_corner(n):
        c, u, v, L, T = frame(W[n])
        back_c = frame(back)[0]
        end = free[n]
        cands = [end + s * v * T / 2 for s in (-1, 1)]
        return min(cands, key=lambda q: np.linalg.norm(q - back_c))  # inner side = closer to back wall's axis? use y-toward-axis
    # inner face = the face looking at the pocket axis (closer to the line y=0 after considering axis through back centre)
    def inner_face_corner(n):
        c, u, v, L, T = frame(W[n]); end = free[n]
        cands = [end + s * v * T / 2 for s in (-1, 1)]
        axis_pt = frame(back)[0]
        return min(cands, key=lambda q: np.linalg.norm(q - axis_pt))
    it, ib = inner_face_corner("c_top"), inner_face_corner("c_bot")
    print(f"inner-face mouth corners: top ({it[0]:.4f},{it[1]:.4f}) bottom ({ib[0]:.4f},{ib[1]:.4f}); corner-to-corner gap = {np.linalg.norm(it-ib):.4f} m")
    dt = poly_dist(rect(top), rect(bot))
    print(f"min distance between the two side-wall slabs (true minimum gap at mouth) = {dt:.4f} m")
    # width of mouth measured perpendicular to pocket axis (rotated x-axis, 25 deg)
    ax_dir = np.array([math.cos(math.radians(25)), math.sin(math.radians(25))])
    perp = np.array([-ax_dir[1], ax_dir[0]])
    print(f"mouth corner separation projected on pocket-normal = {abs(np.dot(it-ib, perp)):.4f} m; along pocket axis = {abs(np.dot(it-ib, ax_dir)):.4f} m")
    # depth along line: mouth chord crossing (centreline) to back wall near face and centreline
    back_c = frame(back)[0]
    cl = None
    a, b = seg(back)
    dd = b - a; tt = (0 - a[1]) / dd[1]; cl = a + tt * dd
    print(f"line meets back-wall centreline at x={cl[0]:.4f}; near (robot-facing) face at x={hit[0]:.4f}")
    print(f"depth along line: mouth chord -> back-wall centreline = {cl[0]-mouth_pt[0]:.4f} m; mouth chord -> near face = {hit[0]-mouth_pt[0]:.4f} m")
    # entry from inner-face view: where the line crosses the top-wall free-end line extended? also report which side walls the line passes
    ax.plot(*mouth_pt, "m*", ms=12, label="line enters mouth chord")
    ax.plot([ft[0], fb[0]], [ft[1], fb[1]], "m:", label="mouth chord")
    ax.plot(hit[0], 0, "kx", ms=10, label="meets back wall (near face)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=8)
    ax.set_xlim(-4, 4); ax.set_ylim(-3, 3); ax.set_title("s3_ctrap: world-frame walls from SDF")
    fig.savefig(HERE / "s3_ctrap_geometry.png", dpi=130)


if __name__ == "__main__":
    main()
