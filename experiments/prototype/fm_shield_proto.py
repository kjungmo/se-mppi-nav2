#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""FM-Shielded SE-MPPI standalone 2D validation (L10 milestone N1).

A navigation foundation model (FM) PROPOSES — a semantic subgoal and a
boldness hint — and SE-MPPI's CBF layer GUARANTEES: every command still passes
the CBF projection, so a proposal can redirect the objective but can never
relax the safety constraint. This file validates that *structure* with the FM
replaced by oracles (design §3 / milestone N1: "FM=오라클/규칙로 대체"):

  - OracleFM      : map-aware detour proposals (the upside: semantic guidance
                    arrives BEFORE the entrapment heuristic would fire).
  - AdversarialFM : actively proposes driving AT a moving obstacle with max
                    boldness (the worst hallucination). The CBF veto must keep
                    the robot collision-free regardless.
  - SilentFM      : never proposes (or goes quiet) — the controller must
                    gracefully degrade to plain SE-MPPI.

The proposer runs at a LOW rate (period >> control dt), outside the control
loop, and the controller consumes the latest proposal — matching the design's
asynchronous-node architecture (no learned inference in the control cycle).
"""

from dataclasses import dataclass

import numpy as np

import se_mppi_proto as se


@dataclass
class Proposal:
    """What an FM hands the controller: WHERE to go and HOW assertively.

    The contract (design §3.3): proposals shape the MPPI objective and the
    coordination hint only. They never enter the CBF constraint — that is what
    makes safety independent of proposal quality.
    """
    subgoal: np.ndarray | None      # temporary objective for MPPI (None = goal)
    boldness: str = 'normal'        # 'bold' | 'normal' | 'cautious'
    stamp: float = 0.0


class OracleFM:
    """Map-aware proposer for the U-trap: routes around the wall's lower end
    immediately — semantic foresight the stall-detection heuristic lacks."""

    def __init__(self, wall_x=2.0, wall_low_y=-0.7, clearance=0.9):
        self.detour = np.array([wall_x + 0.3, wall_low_y - clearance])

    def propose(self, t, state, world):
        if state[0] < self.detour[0]:           # wall still ahead: detour
            return Proposal(self.detour.copy(), 'bold', t)
        return Proposal(None, 'normal', t)      # past the wall: real goal


class AdversarialFM:
    """Worst-case hallucination: lead-pursuit subgoal ON the moving obstacle,
    with maximum boldness. If the architecture is sound this degrades progress
    at most — never safety."""

    def propose(self, t, state, world):
        if not world.obstacles:
            return Proposal(None, 'bold', t)
        o = world.obstacles[0]
        return Proposal(o.p + o.v * 1.0, 'bold', t)   # aim where it will be


class SilentFM:
    """No proposals (e.g. model down / late) — the degrade path."""

    def propose(self, t, state, world):
        return None


def run_shielded(world, fm, cfg, start=(0.0, 0.0, 0.0), max_steps=400,
                 seed=0, fm_period_s=0.5):
    """SE-MPPI control loop + asynchronous FM proposals + CBF veto.

    Mirrors run_validation.run() and keeps its escape/coordination machinery;
    the FM adds (a) an objective override via its subgoal and (b) a boldness
    hint mapped to the alpha schedule. The TTC override and the CBF projection
    are UNCONDITIONAL — the veto the safety claim rests on.
    """
    mppi = se.MPPI(seed=seed)
    detector = se.EntrapmentDetector(stall_window=cfg.get('stall', 15))
    state = np.array(start, float)
    traj = [state[:2].copy()]
    clears = []
    reached = collided = False
    alpha_base, alpha_escape = 2.0, 6.0
    prev_gap = None
    proposal = None
    fm_every = max(1, int(fm_period_s / mppi.dt))

    for step in range(max_steps):
        t = step * mppi.dt
        # ---- low-rate proposer (outside the control loop in the design) ----
        if fm is not None and step % fm_every == 0:
            p = fm.propose(t, state, world)
            if p is not None:
                proposal = p

        dist = np.linalg.norm(state[:2] - world.goal)
        entrapped = detector.update(dist) if cfg['use_escape'] else False

        # ---- objective: FM subgoal > heuristic gap subgoal > goal ----------
        escape_costs = None
        eff_goal = None
        if proposal is not None and proposal.subgoal is not None:
            eff_goal = proposal.subgoal
        elif cfg['use_escape'] and entrapped:
            pos = state[:2].copy()
            gb = np.arctan2(world.goal[1] - pos[1], world.goal[0] - pos[0])
            gap = se.find_escape_gap(pos, gb, world, min_clearance=2.4,
                                     prev_bearing=prev_gap)
            prev_gap = gap
            if gap is not None:
                eff_goal = pos + 2.5 * np.array([np.cos(gap), np.sin(gap)])

            def escape_costs(pts):
                return se.apf_repulsion(pts, world)
        if not entrapped and (proposal is None or proposal.subgoal is None):
            prev_gap = None

        u = mppi.step(state, world, escape_costs, eff_goal=eff_goal)

        # ---- alpha: boldness hint, but TTC override is unconditional -------
        bold = (proposal is not None and proposal.boldness == 'bold') \
            or entrapped
        ttc = se.min_time_to_collision(state, u[0], world)
        alpha = se.coordinate_alpha(bold, ttc, alpha_base, alpha_escape)

        # ---- the veto: every command goes through the CBF projection -------
        v, w, slack, hard = se.cbf_filter(state, u, world, alpha)
        if not hard:
            v = 0.0
        u = np.array([v, w])

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
        'time_s': len(traj) * mppi.dt,
        'min_clear': float(np.min(clears)) if clears else np.inf,
    }
