#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Multi-SE-MPPI standalone 2D validation (L9 milestone N1).

Two (or more) SE-MPPI robots share a narrow corridor. Each robot runs its own
MPPI + entrapment detector + CBF filter and perceives the others as dynamic
obstacles (perception-first: no communication beyond an ID convention).
This prototype contrasts two robot-robot safety modes:

  - INDEPENDENT (baseline, M1/M3 failure): each robot applies its single-robot
    CBF to the other robot with FULL responsibility and the measured closing
    velocity — both brake for the whole conflict, the corridor face-off
    freezes symmetrically, and single-robot escape cannot break the tie
    (the gap toward the goal IS the other robot).

  - COORDINATED (Multi-SE-MPPI): the pairwise barrier is split by
    responsibility lambda_ij + lambda_ji = 1 (each enforces
    2 d . G_i u_i + lambda_ij * alpha * h >= 0, Egerstedt-style certificates,
    no velocity exchange needed), and a mutual-deadlock detection assigns
    deterministic priority: the PASS robot escapes with alpha_escape and a
    right-biased lateral subgoal, the YIELD robot takes a larger
    responsibility share, pulls right, and holds back until the passer clears.

Walls stay in the MPPI cost (the costmap's job, matching the SE-MPPI scope);
only robot-robot pairs enter the CBF — the same division of labour as the
single-robot controller. An emergency override re-enables the full-velocity
constraint when a pair gets critically close, mirroring the TTC override.

The math reuses se_mppi_proto (MPPI, EntrapmentDetector, Obstacle, World).
"""

import numpy as np
import osqp
from scipy import sparse

from se_mppi_proto import (
    MPPI, EntrapmentDetector, Obstacle, World, _wrap, find_escape_gap,
)

ROBOT_RADIUS = 0.22


# --------------------------------------------------------------------------- #
# Responsibility-allocated robot-robot CBF (QP over [v, w, delta])
# --------------------------------------------------------------------------- #
def multi_cbf_filter(state, u_nom, neighbors, alpha, margin=0.06,
                     lookahead=0.2, slack_weight=1e3, v_lim=(-0.35, 0.5),
                     w_lim=1.9, emergency_dist=0.15):
    """Project u_nom onto the responsibility-allocated robot-robot safe set.

    neighbors: list of (position(2,), velocity(2,), radius, lam_ij) — one entry
    per other robot, with this robot's responsibility share lam_ij for the
    pair. lam_ij = 1 with the velocity term reproduces the single-robot
    (independent) constraint; lam_ij = 0.5 without it is the reciprocal share.

    Emergency override: if a pair's surface gap is below emergency_dist the
    constraint reverts to full responsibility + measured closing velocity —
    the multi-robot analogue of the TTC override (safety beats protocol).
    """
    x, y, th = state
    c, s = np.cos(th), np.sin(th)
    p_l = np.array([x + lookahead * c, y + lookahead * s])
    G = np.array([[c, -lookahead * s], [s, lookahead * c]])

    rows, lows = [], []
    h_min = np.inf
    for (p_j, v_j, r_j, lam_ij) in neighbors:
        d = p_l - p_j
        eff_r = ROBOT_RADIUS + r_j + margin
        h = d.dot(d) - eff_r ** 2
        h_min = min(h_min, h)
        gap = np.linalg.norm(d) - eff_r
        if gap < emergency_dist:
            lam_ij, vel_term = 1.0, 2.0 * d.dot(v_j)   # full + reactive
        elif lam_ij >= 1.0 - 1e-9:
            vel_term = 2.0 * d.dot(v_j)                # independent baseline
        else:
            vel_term = 0.0                             # reciprocal share
        a = 2.0 * d @ G
        b = -lam_ij * alpha * h + vel_term
        rows.append([a[0], a[1], 1.0])
        lows.append(b)

    if not rows:
        return (np.clip(u_nom[0], *v_lim), np.clip(u_nom[1], -w_lim, w_lim),
                0.0, True, h_min)

    P = sparse.diags([1.0, 1.0, slack_weight]).tocsc()
    q = np.array([-u_nom[0], -u_nom[1], 0.0])
    A = sparse.csc_matrix(
        np.array(rows + [[1, 0, 0], [0, 1, 0], [0, 0, 1]], float))
    lo = np.array(lows + [v_lim[0], -w_lim, 0.0])
    hi = np.array([np.inf] * len(rows) + [v_lim[1], w_lim, np.inf])

    prob = osqp.OSQP()
    prob.setup(P, q, A, lo, hi, verbose=False, eps_abs=1e-6, eps_rel=1e-6)
    res = prob.solve()
    if res.info.status_val not in (1, 2):
        return 0.0, np.clip(u_nom[1], -w_lim, w_lim), 0.0, False, h_min
    z = res.x
    slack = max(0.0, z[2])
    return (np.clip(z[0], *v_lim), np.clip(z[1], -w_lim, w_lim), slack,
            slack <= 1e-3, h_min)


# --------------------------------------------------------------------------- #
# One SE-MPPI agent (perception-first view of the shared world)
# --------------------------------------------------------------------------- #
class Agent:
    def __init__(self, rid, start, goal, walls, seed):
        self.id = rid
        self.state = np.array(start, float)          # x, y, theta
        self.goal = np.array(goal, float)
        self.walls = walls
        self.mppi = MPPI(seed=seed, prefer_forward=5.0)
        self.detector = EntrapmentDetector(stall_window=12)
        self.cmd = np.zeros(2)                       # last applied (v, w)
        self.trace = [self.state.copy()]
        self.reached = False
        self.role = 'none'                           # latched priority role
        self.prev_gap = None                         # gap-search hysteresis
        self.gap_age = 99                            # cycles since last gap raycast
        self.yield_spot = None                       # parking pose when yielding
        self.block_id = None                         # the robot being passed/yielded to

    def center_velocity(self):
        return self.cmd[0] * np.array(
            [np.cos(self.state[2]), np.sin(self.state[2])])

    def dist_to_goal(self):
        return np.linalg.norm(self.state[:2] - self.goal)

    def step_dynamics(self, v, w, dt):
        self.cmd = np.array([v, w])
        self.state[0] += v * np.cos(self.state[2]) * dt
        self.state[1] += v * np.sin(self.state[2]) * dt
        self.state[2] = _wrap(self.state[2] + w * dt)
        self.trace.append(self.state.copy())


def right_biased_subgoal(agent, lateral=0.45, forward=0.9):
    """A temporary subgoal pulled to the robot's RIGHT of its goal bearing —
    the deterministic, perception-only symmetry breaker (right-hand rule)."""
    to_goal = agent.goal - agent.state[:2]
    bearing = np.arctan2(to_goal[1], to_goal[0])
    right = bearing - np.pi / 2.0
    return (agent.state[:2] +
            forward * np.array([np.cos(bearing), np.sin(bearing)]) +
            lateral * np.array([np.cos(right), np.sin(right)]))


def pass_lane_subgoal(agent, blocker_pos, lateral=0.55, beyond=0.9):
    """Pass target anchored to the BLOCKER: a point beyond it, offset to the
    passer's right by the required pair separation. Anchoring to the blocker
    (not to self) keeps the lane geometrically valid however the passer's own
    pose has drifted during the face-off."""
    to_goal = agent.goal - agent.state[:2]
    bearing = np.arctan2(to_goal[1], to_goal[0])
    fwd = np.array([np.cos(bearing), np.sin(bearing)])
    right = np.array([np.cos(bearing - np.pi / 2),
                      np.sin(bearing - np.pi / 2)])
    return np.asarray(blocker_pos, float) + beyond * fwd + lateral * right


def yield_park_spot(agent, lateral=0.32, retreat=0.30):
    """Where a yielder pulls over: to its RIGHT and slightly back, opening the
    corridor for the passer. Computed once at role assignment."""
    to_goal = agent.goal - agent.state[:2]
    bearing = np.arctan2(to_goal[1], to_goal[0])
    fwd = np.array([np.cos(bearing), np.sin(bearing)])
    right = np.array([np.cos(bearing - np.pi / 2),
                      np.sin(bearing - np.pi / 2)])
    return agent.state[:2] + lateral * right - retreat * fwd


def yield_hold_control(agent, spot, v_max=0.20, k_w=1.8):
    """Reactive pull-over-and-hold: drive to the parking spot (reversing when
    it lies behind), then stop. The explicit 'hold/back-off' yield primitive
    from the problem statement — still CBF-filtered downstream."""
    rel = spot - agent.state[:2]
    dist = np.linalg.norm(rel)
    if dist < 0.08:
        return 0.0, 0.0
    desired = np.arctan2(rel[1], rel[0])
    err = _wrap(desired - agent.state[2])
    if abs(err) > np.pi / 2:           # spot behind: back up toward it
        err = _wrap(err + np.pi)
        v = -v_max * min(1.0, dist / 0.4)
    else:
        v = v_max * min(1.0, dist / 0.4)
    return v * np.cos(err), k_w * err


def conflict_cleared(agent, others, clear_range=1.6):
    """Role release: every other robot is either behind me (w.r.t. my goal
    bearing) or out of conflict range — the face-off is over."""
    to_goal = agent.goal - agent.state[:2]
    n = np.linalg.norm(to_goal)
    if n < 1e-6:
        return True
    fwd = to_goal / n
    for b in others:
        rel = b.state[:2] - agent.state[:2]
        if np.linalg.norm(rel) < clear_range and rel.dot(fwd) > 0.0:
            return False
    return True


# --------------------------------------------------------------------------- #
# Simulation
# --------------------------------------------------------------------------- #
def simulate(agents, mode, dt=0.1, t_max=45.0, goal_tol=0.30,
             alpha_base=2.0, alpha_escape=6.0, deadlock_speed=0.12,
             deadlock_range=1.6):
    """Run the shared world; returns a per-run report dict.

    mode: 'independent' or 'coordinated'.
    """
    n = len(agents)
    steps = int(t_max / dt)
    h_history = []
    clearance_history = []

    for step in range(steps):
        if all(a.reached for a in agents):
            break

        # ---- per-agent decision (synchronous; each uses last-step percepts) -
        new_cmds = {}
        for a in agents:
            if a.reached:
                new_cmds[a.id] = (0.0, 0.0)
                continue
            others = [b for b in agents if b.id != a.id and not b.reached]

            # Perceived world for MPPI: walls + other robots as obstacles.
            # While PASSING (role latched last cycle), shrink the blocker's
            # soft-avoidance zone: the responsibility CBF certifies the close
            # pass, so the sampling cost must not re-block the lane.
            percepts = []
            for b in others:
                ob = Obstacle(b.state[0], b.state[1], ROBOT_RADIUS,
                              *b.center_velocity())
                if a.role == 'pass' and b.id == a.block_id:
                    ob.soft = 0.10
                percepts.append(ob)
            world = World(a.walls + percepts, a.goal,
                          robot_radius=ROBOT_RADIUS)

            entrapped = a.detector.update(a.dist_to_goal())

            # ---- mutual-deadlock detection + deterministic priority --------
            # Roles LATCH until the conflict clears: an un-latched role would
            # flicker off the moment the sidestep itself counts as "progress",
            # reproducing exactly the oscillation it is meant to remove.
            if mode == 'coordinated':
                if a.role == 'none' and entrapped:
                    blockers = [
                        b for b in others
                        if np.linalg.norm(b.state[:2] - a.state[:2]) <
                        deadlock_range
                        and np.linalg.norm(b.center_velocity()) <
                        deadlock_speed
                        and abs(a.cmd[0]) < deadlock_speed
                    ]
                    if blockers:
                        # Lower ID passes (any shared deterministic convention
                        # works; ID stands in for a priority broadcast/norm).
                        nearest = min(
                            blockers, key=lambda b: np.linalg.norm(
                                b.state[:2] - a.state[:2]))
                        a.block_id = nearest.id
                        a.role = ('pass' if a.id < min(b.id for b in blockers)
                                  else 'yield')
                        if a.role == 'yield':
                            a.yield_spot = yield_park_spot(a)
                elif a.role != 'none' and conflict_cleared(a, others):
                    a.role = 'none'
                    a.yield_spot = None
                    a.block_id = None
                    a.detector.reset()
            role = a.role

            # ---- escape subgoal -------------------------------------------
            eff_goal = None
            if mode == 'coordinated' and role == 'pass':
                blocker = next(
                    (b for b in others if b.id == a.block_id), None)
                if blocker is not None:
                    eff_goal = pass_lane_subgoal(a, blocker.state[:2])
            elif mode == 'independent' and entrapped:
                # The UNMODIFIED single-robot escape: gap search over the
                # perceived world (walls + the other robot). It has no notion
                # of the other agent yielding — in a face-off both sidestep
                # against each other and re-stall (M1). Re-raycast at a
                # realistic replan rate (every 5 cycles), not every cycle.
                a.gap_age += 1
                if a.gap_age >= 5 or a.prev_gap is None:
                    to_goal = a.goal - a.state[:2]
                    gb = np.arctan2(to_goal[1], to_goal[0])
                    gap = find_escape_gap(a.state[:2], gb, world,
                                          prev_bearing=a.prev_gap)
                    if gap is not None:
                        a.prev_gap = gap
                    a.gap_age = 0
                if a.prev_gap is not None:
                    eff_goal = a.state[:2] + 1.0 * np.array(
                        [np.cos(a.prev_gap), np.sin(a.prev_gap)])

            if mode == 'coordinated' and role == 'yield':
                # Explicit pull-over-and-hold primitive (problem statement:
                # 'hold/back-off'); MPPI would fight the parking intent.
                u_nom = np.array(yield_hold_control(a, a.yield_spot))
            else:
                u_nom = a.mppi.step(np.copy(a.state), world, eff_goal=eff_goal)

            # ---- responsibility-allocated CBF (robots only) -----------------
            lam = {'pass': 0.3, 'yield': 0.7, 'none': 0.5}[role] \
                if mode == 'coordinated' else 1.0
            neighbors = [
                (b.state[:2].copy(), b.center_velocity(), ROBOT_RADIUS, lam)
                for b in others]

            alpha = alpha_escape if (role == 'pass' or
                                     (mode == 'independent' and entrapped)) \
                else alpha_base
            v, w, slack, hard, h_min = multi_cbf_filter(
                a.state, u_nom, neighbors, alpha)
            if not hard:
                v = min(v, 0.0)
            new_cmds[a.id] = (v, w)
            h_history.append(h_min)

        # ---- apply synchronously ----------------------------------------- -
        for a in agents:
            v, w = new_cmds[a.id]
            if not a.reached:
                a.step_dynamics(v, w, dt)
                if a.dist_to_goal() < goal_tol:
                    a.reached = True

        # ---- shared-world bookkeeping ---------------------------------------
        for i in range(n):
            for j in range(i + 1, n):
                clearance_history.append(
                    np.linalg.norm(agents[i].state[:2] - agents[j].state[:2]) -
                    2 * ROBOT_RADIUS)

    return {
        'mode': mode,
        'all_reached': all(a.reached for a in agents),
        'reached': {a.id: a.reached for a in agents},
        'time': (step + 1) * dt,
        'min_pair_clearance': float(min(clearance_history))
        if clearance_history else np.inf,
        'min_h': float(min(h_history)) if h_history else np.inf,
        'traces': {a.id: np.array(a.trace) for a in agents},
    }


# --------------------------------------------------------------------------- #
# Scenarios
# --------------------------------------------------------------------------- #
def corridor_walls(x0=-3.0, x1=3.0, half_width=0.7, r=0.12, pitch=0.2):
    """Two circle-chain walls bounding a corridor along the x axis."""
    walls = []
    xs = np.arange(x0, x1 + 1e-9, pitch)
    for x in xs:
        walls.append(Obstacle(x, +half_width + r, r))
        walls.append(Obstacle(x, -half_width - r, r))
    return walls


def make_corridor_agents(seed=0):
    """Head-on encounter (M1): two robots swap ends of a narrow corridor."""
    walls = corridor_walls()
    a = Agent(0, start=(-2.6, 0.0, 0.0), goal=(2.6, 0.0), walls=walls,
              seed=seed)
    b = Agent(1, start=(2.6, 0.0, np.pi), goal=(-2.6, 0.0), walls=walls,
              seed=seed + 100)
    return [a, b]


def make_intersection_agents(seed=0):
    """Four robots crossing an open intersection (antipodal swap)."""
    walls = []
    starts = [(-2.2, 0.0, 0.0), (2.2, 0.0, np.pi),
              (0.0, -2.2, np.pi / 2), (0.0, 2.2, -np.pi / 2)]
    goals = [(2.2, 0.0), (-2.2, 0.0), (0.0, 2.2), (0.0, -2.2)]
    return [Agent(i, starts[i], goals[i], walls, seed=seed + 37 * i)
            for i in range(4)]
