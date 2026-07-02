#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Multi-SE-MPPI validation runs (L9 milestone N1).

Two A/B scenarios, each contrasting INDEPENDENT (each robot runs unmodified
single-robot SE-MPPI against the other as a dynamic obstacle) with COORDINATED
(responsibility-allocated CBF + deterministic-priority escape):

  1. corridor      — head-on encounter in a narrow corridor (problem M1):
                     independent face-off freezes; coordination passes.
  2. intersection  — four robots swapping antipodal positions (M2/M3 stress):
                     responsibility split removes the double-braking standoff.

Success criteria (printed and asserted in the summary):
  - coordinated: ALL robots reach their goals, min pairwise surface clearance
    > 0 (no robot-robot collision) throughout.
  - independent corridor: NOT all robots reach within the budget (the
    documented baseline failure that motivates L9).

Outputs figures/multirobot_{corridor,intersection}.png and a metrics table.
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from multi_se_proto import (
    ROBOT_RADIUS, make_corridor_agents, make_intersection_agents, simulate,
)

HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, 'figures')


def run_scenario(name, factory, t_max):
    results = {}
    for mode in ('independent', 'coordinated'):
        agents = factory(seed=7)
        results[mode] = simulate(agents, mode, t_max=t_max)
    plot_scenario(name, factory, results)
    return results


def plot_scenario(name, factory, results):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True, sharey=True)
    colors = ['tab:blue', 'tab:red', 'tab:green', 'tab:orange']
    for ax, mode in zip(axes, ('independent', 'coordinated')):
        res = results[mode]
        walls = factory(seed=0)[0].walls
        for wobs in walls:
            ax.add_patch(plt.Circle(wobs.p, wobs.r, color='0.6', zorder=1))
        for rid, trace in sorted(res['traces'].items()):
            ax.plot(trace[:, 0], trace[:, 1], color=colors[rid % 4],
                    lw=1.6, zorder=3, label=f'robot {rid}')
            ax.plot(trace[0, 0], trace[0, 1], 'o', color=colors[rid % 4],
                    ms=7, zorder=4)
            ax.plot(trace[-1, 0], trace[-1, 1], 's', color=colors[rid % 4],
                    ms=7, zorder=4)
            ax.add_patch(plt.Circle(trace[-1, :2], ROBOT_RADIUS,
                                    color=colors[rid % 4], alpha=0.25,
                                    zorder=2))
        ok = 'ALL REACHED' if res['all_reached'] else 'DEADLOCK / TIMEOUT'
        ax.set_title(f"{mode}\n{ok} | t={res['time']:.1f}s | "
                     f"min clear={res['min_pair_clearance']:.2f} m")
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
    fig.suptitle(f'Multi-SE-MPPI N1: {name} '
                 '(o = start, ■ = final position)')
    fig.tight_layout()
    os.makedirs(FIGS, exist_ok=True)
    out = os.path.join(FIGS, f'multirobot_{name}.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f'  wrote {os.path.relpath(out, HERE)}')


def main():
    print('=== L9/N1 multi-robot validation ===')
    table = []
    for name, factory, t_max in (
        ('corridor', make_corridor_agents, 60.0),
        ('intersection', make_intersection_agents, 45.0),
    ):
        print(f'\n--- scenario: {name} ---')
        results = run_scenario(name, factory, t_max)
        for mode, res in results.items():
            table.append((name, mode, res['all_reached'], res['time'],
                          res['min_pair_clearance'], res['min_h']))
            print(f"  {mode:12s} all_reached={res['all_reached']} "
                  f"time={res['time']:5.1f}s "
                  f"min_clear={res['min_pair_clearance']:+.3f} m "
                  f"min_h={res['min_h']:+.3f}")

    print('\n=== summary table ===')
    print(f"{'scenario':14s} {'mode':12s} {'all_reached':12s} "
          f"{'time[s]':>8s} {'min_clear[m]':>13s} {'min_h':>8s}")
    for row in table:
        print(f'{row[0]:14s} {row[1]:12s} {str(row[2]):12s} '
              f'{row[3]:8.1f} {row[4]:13.3f} {row[5]:8.3f}')

    # ---- the N1 claims, asserted -----------------------------------------
    res = {(r[0], r[1]): r for r in table}
    failures = []
    for name in ('corridor', 'intersection'):
        coord = res[(name, 'coordinated')]
        if not coord[2]:
            failures.append(f'{name}: coordinated did not all reach')
        if coord[4] <= 0.0:
            failures.append(f'{name}: coordinated robot-robot collision')
    if res[('corridor', 'independent')][2]:
        failures.append('corridor: independent unexpectedly resolved the '
                        'face-off (baseline failure not reproduced)')
    if failures:
        print('\nVALIDATION FAILED:')
        for f in failures:
            print(f'  - {f}')
        raise SystemExit(1)
    print('\nVALIDATION OK: coordination resolves the deadlock the '
          'independent baseline cannot, with no robot-robot collision.')


if __name__ == '__main__':
    main()
