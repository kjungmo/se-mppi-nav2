# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Does the 0.6 m threshold on the longest-ray fallback matter in the advised arm?

The mirror flag ``escape_extras=False`` (arm ``v2_advised``) removes the 0.6 m clearance
threshold from the opening selection altogether: from the horizon candidates (addition (d))
and from the longest-ray fallback. The advice had no threshold on the fallback. This
script runs the F configuration of the ``v2_advised`` arm on the U-trap family twice, once as
the arm is, once with the threshold restored for both parts of the selection (all other
additions stay off), and compares the outcome per seed.

    python3 -m experiments.benchmark2d.advised_fallback_threshold -n 50 \
        > experiments/results_2d_v2_advised/FALLBACK_THRESHOLD.md

Single process, deterministic. Same budget as the committed arm (``--max-steps 400``).
"""

from __future__ import annotations

import argparse

from . import scenarios as sc
from .configs import make_v2_configs
from .rollout import rollout

import se_mppi_v2 as v2m  # noqa: E402  (configs puts experiments/prototype on sys.path)


def _select_opening_with_threshold(pos, goal_bearing, world, p, prev_bearing=None, avoid=()):
    """``select_opening`` of the mirror with the clearance threshold on, the avoid list off."""
    bearings, free, reached = v2m.scan_rays(pos, world, p['gap_num_rays'], p['gap_max_range'],
                                            p['window_half'])
    return v2m.select_horizon_gap(bearings, free, reached, goal_bearing, p['gap_min_clearance'],
                                  prev_bearing, p['hysteresis'], ())


def run(family, n, max_steps, config='F_full'):
    """{seed: outcome} for the arm as it is and with the threshold restored."""
    cfg = make_v2_configs(escape_extras=False)[config]
    original = v2m.select_opening
    out = {'as_is': {}, 'threshold': {}}
    try:
        for label, fn in (('as_is', original), ('threshold', _select_opening_with_threshold)):
            v2m.select_opening = fn
            for seed in range(n):
                scn = sc.make(family, seed)
                r = rollout(scn.world, cfg, scn.start, goal_tol=0.25, max_steps=max_steps,
                            seed=seed)
                out[label][seed] = r['outcome']
    finally:
        v2m.select_opening = original
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', type=int, default=50)
    ap.add_argument('--family', default='utrap')
    ap.add_argument('--max-steps', type=int, default=400)
    a = ap.parse_args()
    res = run(a.family, a.n, a.max_steps)
    a_ok = sum(o == 'SUCCESS' for o in res['as_is'].values())
    b_ok = sum(o == 'SUCCESS' for o in res['threshold'].values())
    differ = [s for s in range(a.n) if res['as_is'][s] != res['threshold'][s]]
    print('# Clearance threshold on the fallback ray, advised arm\n')
    print(f'Family `{a.family}`, config F, seeds 0 to {a.n - 1}, `--max-steps {a.max_steps}`, '
          'additions (a) to (c) off in both columns.\n')
    print('| Selection | Successes | Seeds |')
    print('|---|---|---|')
    print(f'| as in the advised arm (no threshold) | {a_ok} | {a.n} |')
    print(f'| 0.6 m threshold restored | {b_ok} | {a.n} |')
    print(f'\nSeeds whose outcome differs between the two columns: {len(differ)}'
          + (f' ({differ})' if differ else '') + '.')


if __name__ == '__main__':
    main()
