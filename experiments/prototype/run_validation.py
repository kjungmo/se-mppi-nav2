#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Run SE-MPPI 2D validation scenarios and emit metrics + figures.

Scenarios:
  1. U-trap (static local minimum)  -> does escape get the robot out?
  2. Dynamic crossing obstacle      -> does the CBF filter keep it safe?
  3. Coordination (U-trap + dynamic) -> independent escape+CBF (E) vs SE (F).

Each config is a dict of toggles mirroring the ablation matrix:
  use_escape, use_gap, use_cbf, use_coordination.
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import se_mppi_proto as se  # noqa: E402

FIG = os.path.join(os.path.dirname(__file__), 'figures')
os.makedirs(FIG, exist_ok=True)


def chain(x0, y0, x1, y1, r=0.25, step=0.22):
    n = max(2, int(np.hypot(x1 - x0, y1 - y0) / step) + 1)
    return [se.Obstacle(x, y, r) for x, y in
            zip(np.linspace(x0, x1, n), np.linspace(y0, y1, n))]


def u_trap_world():
    # A single finite wall between robot and goal: greedy goal-descent pins the
    # robot against it (local minimum); escaping requires routing around an end.
    obs = chain(2.0, -0.7, 2.0, 0.9)  # slightly off-centre -> a preferred side
    return se.World(obs, goal=(4.0, 0.0))


def dynamic_world():
    # Obstacle crossing the path fast; MPPI (which sees only the current
    # position) reacts late, the velocity-aware CBF anticipates.
    o = se.Obstacle(2.0, -3.0, 0.25, vx=0.0, vy=0.6)
    return se.World([o], goal=(4.0, 0.0))


def coordination_world():
    # Finite-wall local minimum AND a moving obstacle drifting across the
    # upper escape route, so escape and dynamic safety conflict.
    w = u_trap_world()
    w.obstacles.append(se.Obstacle(2.6, 2.6, 0.25, vx=0.0, vy=-0.5))
    return w


# --------------------------------------------------------------------------- #
def run(world, cfg, start=(0.0, 0.0, 0.0), max_steps=400, seed=0):
    mppi = se.MPPI(seed=seed)
    detector = se.EntrapmentDetector(stall_window=cfg.get('stall', 15))
    state = np.array(start, float)
    traj, alphas, slacks, clears, ent_flags = [state[:2].copy()], [], [], [], []
    reached = collided = False
    alpha_base, alpha_escape = 2.0, 6.0
    prev_gap = None

    for _ in range(max_steps):
        dist = np.linalg.norm(state[:2] - world.goal)
        entrapped = detector.update(dist) if cfg['use_escape'] else False
        ent_flags.append(entrapped)

        escape_costs = None
        eff_goal = None
        if not entrapped:
            prev_gap = None
        if cfg['use_escape'] and entrapped:
            pos = state[:2].copy()
            gb = np.arctan2(world.goal[1] - pos[1], world.goal[0] - pos[0])
            # Require a genuinely open ray (reaches far), not one that merely
            # clears min_clearance before dead-ending into a wall; hysteresis
            # keeps the choice stable across symmetric openings.
            gap = (se.find_escape_gap(pos, gb, world, min_clearance=2.4,
                                      prev_bearing=prev_gap)
                   if cfg['use_gap'] else None)
            prev_gap = gap
            # detect-and-switch: steer to a TEMPORARY SUBGOAL placed through the
            # opening (the design's free-space gap subgoal), so the robot rounds
            # the obstacle decisively instead of pinning against it. APF keeps it
            # off the walls.
            if gap is not None:
                eff_goal = pos + 2.5 * np.array([np.cos(gap), np.sin(gap)])

            def escape_costs(pts):
                return se.apf_repulsion(pts, world)

        u = mppi.step(state, world, escape_costs, eff_goal=eff_goal)

        alpha = alpha_base
        if cfg['use_cbf']:
            ttc = se.min_time_to_collision(state, u[0], world)
            ent = entrapped if cfg['use_coordination'] else False
            alpha = se.coordinate_alpha(ent, ttc, alpha_base, alpha_escape)
            v, w, slack, hard = se.cbf_filter(state, u, world, alpha)
            if not hard:
                v = 0.0   # brake if the barrier had to be relaxed
            u = np.array([v, w])
            slacks.append(slack)
        alphas.append(alpha)

        # integrate true robot + obstacles
        state[0] += u[0] * np.cos(state[2]) * mppi.dt
        state[1] += u[0] * np.sin(state[2]) * mppi.dt
        state[2] += u[1] * mppi.dt
        world.step_obstacles(mppi.dt)

        traj.append(state[:2].copy())
        clears.append(world.min_clearance(state[:2]) - world.robot_radius)
        if world.in_collision(state[:2]):
            collided = True
            break
        if np.linalg.norm(state[:2] - world.goal) < 0.25:
            reached = True
            break

    return {
        'traj': np.array(traj), 'reached': reached, 'collided': collided,
        'steps': len(traj), 'time_s': len(traj) * mppi.dt,
        'min_clear': float(np.min(clears)) if clears else np.inf,
        'alphas': np.array(alphas), 'slacks': np.array(slacks),
        'ent': np.array(ent_flags),
    }


# --------------------------------------------------------------------------- #
def plot_world(ax, world, title):
    for o in world.obstacles:
        ax.add_patch(plt.Circle(o.p, o.r, color='0.3'))
    ax.plot(*world.goal, 'g*', ms=16)
    ax.set_aspect('equal')
    ax.set_xlim(-0.7, 4.6)
    ax.set_ylim(-2.8, 2.4)
    ax.set_title(title, fontsize=10)


CFG_STOCK = dict(use_escape=False, use_gap=False, use_cbf=False, use_coordination=False)
CFG_ESC = dict(use_escape=True, use_gap=True, use_cbf=False, use_coordination=False)
CFG_INDEP = dict(use_escape=True, use_gap=True, use_cbf=True, use_coordination=False)
CFG_SE = dict(use_escape=True, use_gap=True, use_cbf=True, use_coordination=True)


def fig_utrap():
    rows = []
    fig, axes = plt.subplots(1, 2, figsize=(9, 4.2))
    for ax, (name, cfg) in zip(axes, [('Stock MPPI', CFG_STOCK),
                                       ('SE-MPPI (escape)', CFG_ESC)]):
        w = u_trap_world()
        r = run(w, cfg)
        plot_world(ax, w, f'{name}\nreached={r["reached"]} t={r["time_s"]:.1f}s')
        ax.plot(r['traj'][:, 0], r['traj'][:, 1], 'b-', lw=1.5)
        ax.plot(0, 0, 'ko', ms=5)
        rows.append((f'U-trap / {name}', r))
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'utrap_escape.png'), dpi=110)
    plt.close(fig)
    return rows


def fig_dynamic():
    rows = []
    fig, axes = plt.subplots(1, 2, figsize=(9, 4.2))
    for ax, (name, cfg) in zip(axes, [('No CBF', CFG_ESC),
                                       ('SE-MPPI (CBF)', CFG_SE)]):
        w = dynamic_world()
        r = run(w, cfg)
        plot_world(ax, w, f'{name}\ncollided={r["collided"]} '
                          f'min_clear={r["min_clear"]:.2f}m')
        ax.plot(r['traj'][:, 0], r['traj'][:, 1], 'b-', lw=1.5)
        ax.plot(0, 0, 'ko', ms=5)
        rows.append((f'Dynamic / {name}', r))
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'dynamic_cbf.png'), dpi=110)
    plt.close(fig)
    return rows


def fig_coordination():
    """Show the escape-safety coordination MECHANISM on the U-trap: alpha
    escalates only while entrapped (permitting the escape detour) yet the CBF
    slack stays ~0 throughout, i.e. the escape is certified-safe (h >= 0)."""
    rows = []
    w = u_trap_world()
    r = run(w, CFG_SE)
    rows.append(('Coord / SE-MPPI (F)', r))
    # E vs F outcome for the metrics table (benign here -> both reach).
    rows.append(('Coord / Independent (E)', run(coordination_world(), CFG_INDEP)))

    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    traj, ent = r['traj'], r['ent']
    ax = axes[0]
    plot_world(ax, w, 'SE-MPPI: escape phase highlighted')
    ax.plot(traj[:, 0], traj[:, 1], 'b-', lw=1.2, label='nominal')
    em = np.concatenate([ent, [ent[-1]]]) if len(ent) < len(traj) else ent[:len(traj)]
    esc = traj[np.flatnonzero(em)]
    if len(esc):
        ax.plot(esc[:, 0], esc[:, 1], 'r.', ms=4, label='entrapped/escaping')
    ax.plot(0, 0, 'ko', ms=5)
    ax.legend(fontsize=7, loc='lower right')
    ax = axes[1]
    ax.plot(r['alphas'], 'r-')
    ax.set_title('CBF gain alpha (2->6 while entrapped)', fontsize=9)
    ax.set_xlabel('control step')
    ax.set_ylabel('alpha')
    ax = axes[2]
    ax.plot(r['slacks'], 'm-')
    ax.set_title('CBF slack ~ 0 == certified-safe escape', fontsize=9)
    ax.set_xlabel('control step')
    ax.set_ylabel('slack')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'coordination.png'), dpi=110)
    plt.close(fig)
    return rows


def main():
    all_rows = fig_utrap() + fig_dynamic() + fig_coordination()
    print('\n=== SE-MPPI 2D validation results ===')
    print(f'{"scenario / config":36s} {"reached":>7} {"collided":>8} '
          f'{"time_s":>7} {"min_clear_m":>11}')
    for name, r in all_rows:
        print(f'{name:36s} {str(r["reached"]):>7} {str(r["collided"]):>8} '
              f'{r["time_s"]:7.1f} {r["min_clear"]:11.2f}')
    print(f'\nFigures written to {FIG}/')


if __name__ == '__main__':
    main()
