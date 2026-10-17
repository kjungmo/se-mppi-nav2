# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""One-trial rollout for the 2D benchmark.

This is a faithful mirror of ``experiments/prototype/run_validation.run`` — the
validated control loop — reusing the *same* ``se_mppi_proto`` primitives (MPPI,
EntrapmentDetector, APF, gap search, DCBF-QP, α coordination, TTC). The MPPI
sampling, APF cost, gap raycast, entrapment logic, integration, and collision
test are byte-for-byte the same calls.

The single, intentional, documented difference from ``run_validation.run``:

  * the DCBF-QP and the TTC estimate see only the **dynamic** obstacles (those
    with nonzero velocity), not the static walls.

This matches the *real* Nav2 controller (paper §IV-D and the §VI-B deployment
lesson: "feeding static walls into the CBF makes the barrier infeasible
everywhere and freezes the robot — fixed by scoping the CBF to genuinely dynamic
obstacles"). Static geometry is handled, as in Nav2, by the sampling/obstacle
cost and the escape layer. ``run_validation.py`` itself is left untouched so the
§VI-A mechanism numbers remain exactly reproducible.

The trial metric conventions follow ``experiments/runner/metrics``: a run is a
SUCCESS iff it reaches the goal without a footprint collision; COLLISION if the
footprint overlaps an obstacle; TIMEOUT otherwise (ran out of the step budget).
"""

from __future__ import annotations

import os
import sys

import numpy as np

_PROTO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'prototype')
if _PROTO not in sys.path:
    sys.path.insert(0, _PROTO)
import se_mppi_proto as se  # noqa: E402
import se_mppi_v2 as v2m  # noqa: E402

ALPHA_BASE = 2.0
ALPHA_ESCAPE = 6.0


class _SubWorld:
    """A thin obstacle view exposing the attributes the CBF / TTC read."""

    def __init__(self, obstacles, robot_radius):
        self.obstacles = obstacles
        self.robot_radius = robot_radius


def _dynamic_view(world):
    dyn = [o for o in world.obstacles if float(np.hypot(o.v[0], o.v[1])) > 1e-9]
    return _SubWorld(dyn, world.robot_radius)


def rollout(world, cfg, start, *, goal_tol=0.25, max_steps=400, seed=0):
    """Run one closed-loop trial; return metrics + diagnostic time series.

    Mirrors ``run_validation.run`` exactly except the CBF/TTC obstacle scope
    (see module docstring).

    SE-MPPI v2 (opt-in): a config that carries a ``'v2'`` dict (see
    ``configs.CONFIGS_V2`` and ``se_mppi_v2.V2_PARAMS``) switches four things
    independently; a config without it runs the paper-v1 loop unchanged.
    """
    mppi = se.MPPI(seed=seed)
    detector = se.EntrapmentDetector(stall_window=cfg.get('stall', 15))
    state = np.array(start, float)
    dyn = _dynamic_view(world)
    v2 = cfg.get('v2')
    tracker = hopper = None
    zero_cmd_steps = 0
    if v2 is not None and cfg['use_escape']:
        if v2['progress'] == 'anchor':
            tracker = v2m.AnchorProgress(state[:2], world.goal,
                                         v2['progress_min_advance'],
                                         v2['progress_adopt_advance'])
            replan_every = max(1, int(round(v2['replan_period_s'] / mppi.dt)))
        hopper = v2m.WaypointHopper(v2, mppi.dt)

    traj = [state[:2].copy()]
    alphas, slacks, clears, ent_flags = [], [], [], []
    reached = collided = False
    prev_gap = None

    for step_i in range(max_steps):
        dist = np.linalg.norm(state[:2] - world.goal)
        if tracker is not None:
            # v2: progress on the anchor path; the BT replans a straight path
            # from the robot pose every second (the mirror's planner is blind
            # to the pocket, like the planner of the live runs).
            if step_i > 0 and step_i % replan_every == 0:
                tracker.replan(state[:2], world.goal)
            entrapped = detector.update(-float(tracker.update(state[:2])))
        else:
            entrapped = detector.update(dist) if cfg['use_escape'] else False
        ent_flags.append(entrapped)

        escape_costs = None
        eff_goal = None
        if not entrapped:
            prev_gap = None
            if hopper is not None:
                hopper.release()
        if cfg['use_escape'] and entrapped:
            pos = state[:2].copy()
            gb = np.arctan2(world.goal[1] - pos[1], world.goal[0] - pos[0])
            if hopper is not None:
                eff_goal = (hopper.update(step_i, pos, gb, world)
                            if cfg['use_gap'] else None)
            else:
                gap = (se.find_escape_gap(pos, gb, world, min_clearance=2.4,
                                          prev_bearing=prev_gap)
                       if cfg['use_gap'] else None)
                prev_gap = gap
                if gap is not None:
                    eff_goal = pos + 2.5 * np.array([np.cos(gap), np.sin(gap)])

            def escape_costs(pts):
                return se.apf_repulsion(pts, world)

        if v2 is not None and v2['zero_on_all_collide']:
            u = mppi.step(state, world, escape_costs, eff_goal=eff_goal,
                          zero_on_all_collide=True)
            zero_cmd_steps += int(mppi.last_all_collide)
        else:
            u = mppi.step(state, world, escape_costs, eff_goal=eff_goal)

        alpha = ALPHA_BASE
        if cfg['use_cbf']:
            ttc = se.min_time_to_collision(state, u[0], dyn)
            ent = entrapped if cfg['use_coordination'] else False
            alpha = se.coordinate_alpha(ent, ttc, ALPHA_BASE, ALPHA_ESCAPE)
            v, w, slack, hard = se.cbf_filter(state, u, dyn, alpha)
            if not hard:
                v = 0.0
            u = np.array([v, w])
            slacks.append(slack)
        alphas.append(alpha)

        state[0] += u[0] * np.cos(state[2]) * mppi.dt
        state[1] += u[0] * np.sin(state[2]) * mppi.dt
        state[2] += u[1] * mppi.dt
        world.step_obstacles(mppi.dt)

        traj.append(state[:2].copy())
        clears.append(world.min_clearance(state[:2]) - world.robot_radius)
        if world.in_collision(state[:2]):
            collided = True
            break
        if np.linalg.norm(state[:2] - world.goal) < goal_tol:
            reached = True
            break

    traj = np.array(traj)
    path_length = float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1))) \
        if len(traj) > 1 else 0.0
    steps = len(traj)
    time_s = steps * mppi.dt
    success = bool(reached and not collided)
    if collided:
        outcome = 'COLLISION'
    elif reached:
        outcome = 'SUCCESS'
    else:
        outcome = 'TIMEOUT'

    out = {
        'success': success,
        'collided': collided,
        'reached': reached,
        'outcome': outcome,
        'time_to_goal': time_s,
        'path_length': path_length,
        'min_clearance': float(np.min(clears)) if clears else float('inf'),
        'steps': steps,
        'alpha_max': float(np.max(alphas)) if alphas else ALPHA_BASE,
        'alpha_escape_frac': (float(np.mean(np.array(alphas) > ALPHA_BASE + 1e-9))
                              if alphas else 0.0),
        'slack_max': float(np.max(slacks)) if slacks else 0.0,
        'slack_mean': float(np.mean(slacks)) if slacks else 0.0,
        'cbf_active_steps': len(slacks),
        'entrapped_frac': (float(np.mean(ent_flags)) if ent_flags else 0.0),
        # kept out of the CSV but handy for the trajectory montage:
        '_traj': traj,
    }
    if v2 is not None:
        out.update({
            'v2_zero_cmd_steps': zero_cmd_steps,
            'v2_progress_events': tracker.events if tracker else 0,
            'v2_adoptions': tracker.adoptions if tracker else 0,
            'v2_hops': hopper.hops if hopper else 0,
        })
    return out
