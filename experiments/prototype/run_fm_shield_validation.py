#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""FM-Shielded SE-MPPI validation runs (L10 milestone N1).

Three runs, one per oracle, against the existing single-robot worlds:

  1. U-trap + OracleFM      — semantic detour proposal; should reach (and not
                              be slower than the heuristic baseline's order).
  2. Dynamic + AdversarialFM — proposals aim AT the moving obstacle with max
                              boldness; the CBF veto must keep min clearance
                              >= 0 (progress may suffer; safety may not).
  3. U-trap + SilentFM      — no proposals; must degrade to plain SE-MPPI
                              (reaches like the baseline).

Together these validate the load-bearing claim (design §3.3): proposals shape
only the objective, the CBF owns the constraint, so forward-invariance holds
REGARDLESS of proposal quality.

Outputs figures/fm_shield.png + a metrics table; exits non-zero on any gate.
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import run_validation as rv  # noqa: E402  (worlds + baseline config/run)
from fm_shield_proto import (  # noqa: E402
    AdversarialFM, OracleFM, SilentFM, run_shielded,
)

HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, 'figures')


def main():
    print('=== L10/N1 FM-shield validation ===')
    cfg = rv.CFG_SE

    # Baseline for reference: heuristic SE-MPPI on the U-trap.
    base = rv.run(rv.u_trap_world(), cfg)

    runs = [
        ('U-trap + OracleFM', rv.u_trap_world(), OracleFM()),
        ('Dynamic + AdversarialFM', rv.dynamic_world(), AdversarialFM()),
        ('U-trap + SilentFM (degrade)', rv.u_trap_world(), SilentFM()),
    ]
    results = []
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.4))
    for ax, (name, world, fm) in zip(axes, runs):
        r = run_shielded(world, fm, cfg)
        results.append((name, r))
        rv.plot_world(ax, world, f'{name}\nreached={r["reached"]} '
                                 f'collided={r["collided"]} '
                                 f'min_clear={r["min_clear"]:+.2f} m')
        ax.plot(r['traj'][:, 0], r['traj'][:, 1], 'b-', lw=1.5)
        ax.plot(0, 0, 'ko', ms=5)
    fig.suptitle('FM-Shielded SE-MPPI (L10/N1): proposals steer, the CBF '
                 'vetoes — safety is independent of proposal quality')
    fig.tight_layout()
    os.makedirs(FIGS, exist_ok=True)
    out = os.path.join(FIGS, 'fm_shield.png')
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f'  wrote {os.path.relpath(out, HERE)}')

    print(f'\n{"run":34s} {"reached":>7s} {"collided":>8s} '
          f'{"time_s":>7s} {"min_clear":>10s}')
    print(f'{"(baseline) U-trap heuristic SE":34s} {str(base["reached"]):>7s} '
          f'{str(base["collided"]):>8s} {base["time_s"]:7.1f} '
          f'{base["min_clear"]:10.2f}')
    for name, r in results:
        print(f'{name:34s} {str(r["reached"]):>7s} {str(r["collided"]):>8s} '
              f'{r["time_s"]:7.1f} {r["min_clear"]:10.2f}')

    # ---- gates --------------------------------------------------------------
    oracle, adversarial, silent = (r for _, r in results)
    failures = []
    if not (oracle['reached'] and not oracle['collided']):
        failures.append('OracleFM: did not reach safely')
    if adversarial['collided'] or adversarial['min_clear'] < 0.0:
        failures.append('AdversarialFM: the CBF veto FAILED — collision')
    if not (silent['reached'] == base['reached'] and not silent['collided']):
        failures.append('SilentFM: degrade path diverged from the baseline')
    if failures:
        print('\nVALIDATION FAILED:')
        for f in failures:
            print(f'  - {f}')
        raise SystemExit(1)
    print('\nVALIDATION OK: oracle guidance helps, adversarial proposals '
          'cannot cause a collision, and a silent FM degrades to plain '
          'SE-MPPI.')


if __name__ == '__main__':
    main()
