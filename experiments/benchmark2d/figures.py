# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Figures for the 2D benchmark (paper §VI-C).

Three figures, all headless (Agg):

  * ``success_bars.png``  — per-family grouped bars of success / collision /
    timeout across the A–F configs, success carrying Wilson-95% error bars;
  * ``e_vs_f.png``        — the coordination contrast: E_indep vs F_full success
    (with CI) + collision per family, annotated with the McNemar p-value;
  * ``traj_montage.png``  — representative trajectories (stock A vs coordinated F)
    over each family's world, regenerated live from the seed so what is drawn is
    exactly what was measured.

``summary`` and ``stats`` are the objects returned by ``aggregate.summarize`` /
``aggregate.compute_stats``.
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

from . import scenarios as sc     # noqa: E402
from .configs import ORDER, KEY_CONTRAST, CONFIGS  # noqa: E402
from .rollout import rollout      # noqa: E402

_COLORS = {'success': '#22aa77', 'collision': '#cc3333', 'timeout': '#ffaa00'}

# Short config codes for axis ticks (Tables III–IV notation in the paper).
_SHORT = {'A_stock': 'A', 'C_escape': 'C', 'D_cbf': 'D', 'E_indep': 'E',
          'F_full': 'F', 'Fminus_nogap': 'F$^{-}$'}


def _families(summary):
    return sorted({e['family'] for e in summary})


def _idx(summary, family):
    return {e['config']: e for e in summary if e['family'] == family}


def plot_success_bars(summary, out_path):
    fams = _families(summary)
    fig, axes = plt.subplots(1, len(fams), figsize=(5.2 * len(fams), 4.2),
                             squeeze=False)
    for ax, fam in zip(axes[0], fams):
        idx = _idx(summary, fam)
        cfgs = [c for c in ORDER if c in idx]
        succ = [idx[c]['success_rate'] for c in cfgs]
        coll = [idx[c]['collision_rate'] for c in cfgs]
        tout = [idx[c]['timeout_rate'] for c in cfgs]
        lo = [max(0.0, idx[c]['success_rate'] - idx[c]['success_ci_low']) for c in cfgs]
        hi = [max(0.0, idx[c]['success_ci_high'] - idx[c]['success_rate']) for c in cfgs]
        x = np.arange(len(cfgs))
        w = 0.27
        ax.bar(x - w, succ, w, label='success', color=_COLORS['success'],
               yerr=[lo, hi], capsize=3)
        ax.bar(x, coll, w, label='collision', color=_COLORS['collision'])
        ax.bar(x + w, tout, w, label='timeout', color=_COLORS['timeout'])
        ax.set_xticks(x)
        ax.set_xticklabels([_SHORT.get(c, c) for c in cfgs], fontsize=13)
        ax.set_ylim(0, 1.0)
        ax.set_ylabel('rate')
        n = idx[cfgs[0]]['n_trials'] if cfgs else 0
        ax.set_title(f'{fam}  (N={n}/config, success CI = Wilson 95%)', fontsize=10)
        ax.legend(fontsize=8)
    fig.tight_layout()
    _save(fig, out_path)
    return out_path


def plot_e_vs_f(summary, stats, out_path):
    fams = _families(summary)
    e_cfg, f_cfg = KEY_CONTRAST
    fig, axes = plt.subplots(1, len(fams), figsize=(4.3 * len(fams), 4.2),
                             squeeze=False)
    for ax, fam in zip(axes[0], fams):
        idx = _idx(summary, fam)
        e, f = idx.get(e_cfg), idx.get(f_cfg)
        if not e or not f:
            continue
        succ = [e['success_rate'], f['success_rate']]
        coll = [e['collision_rate'], f['collision_rate']]
        elo = max(0.0, e['success_rate'] - e['success_ci_low'])
        ehi = max(0.0, e['success_ci_high'] - e['success_rate'])
        flo = max(0.0, f['success_rate'] - f['success_ci_low'])
        fhi = max(0.0, f['success_ci_high'] - f['success_rate'])
        x = np.array([0, 1])
        w = 0.35
        ax.bar(x - w / 2, succ, w, label='success', color=_COLORS['success'],
               yerr=[[elo, flo], [ehi, fhi]], capsize=4)
        ax.bar(x + w / 2, coll, w, label='collision', color=_COLORS['collision'])
        ax.set_xticks(x)
        ax.set_xticklabels(['E: escape+CBF\n(independent)',
                            'F: SE-MPPI\n(coordinated)'], fontsize=9)
        ax.set_ylim(0, 1.0)
        ax.set_ylabel('rate')
        ax.set_title(f'E vs F — {fam}', fontsize=10)
        ax.legend(fontsize=8, loc='upper left')
        # McNemar annotation from stats
        comp = next((c for c in stats.get(fam, {}).get('comparisons', [])
                     if c['baseline'] == e_cfg), None)
        if comp:
            ax.annotate(f"McNemar p = {comp['mcnemar_p']:.3g}\n"
                        f"(b={comp['mcnemar_b']}, c={comp['mcnemar_c']})",
                        xy=(0.5, 0.97), xycoords='axes fraction', ha='center',
                        va='top', fontsize=8, color='#333333')
    fig.tight_layout()
    _save(fig, out_path)
    return out_path


def _plot_world(ax, world, start, goal):
    for o in world.obstacles:
        dyn = float(np.hypot(o.v[0], o.v[1])) > 1e-9
        ax.add_patch(plt.Circle(o.p, o.r, color='#cc7722' if dyn else '0.4',
                                alpha=0.9))
    ax.plot(goal[0], goal[1], 'g*', ms=15)
    ax.plot(start[0], start[1], 'ko', ms=5)
    ax.set_aspect('equal')


def plot_traj_montage(out_path, montage=None, max_steps=350):
    """Regenerate representative scenarios and draw stock (A) vs coordinated (F).

    ``montage`` is a list of (family, seed) pairs; defaults to one illustrative
    seed per family. Trajectories are recomputed live so the drawing matches the
    measured run exactly.
    """
    montage = montage or [('utrap', 0), ('clutter', 3), ('dynamic', 1)]
    fig, axes = plt.subplots(2, len(montage), figsize=(4.6 * len(montage), 8),
                             squeeze=False)
    for col, (fam, seed) in enumerate(montage):
        for row, cfg_name in enumerate(['A_stock', 'F_full']):
            ax = axes[row][col]
            scn = sc.make(fam, seed)
            r = rollout(scn.world, CONFIGS[cfg_name], scn.start,
                        max_steps=max_steps, seed=seed)
            # world.obstacles have advanced (movers); redraw at final state is
            # fine for static families; for dynamic we draw initial positions by
            # regenerating a fresh scenario for the backdrop.
            backdrop = sc.make(fam, seed)
            _plot_world(ax, backdrop.world, scn.start, scn.goal)
            traj = r['_traj']
            ax.plot(traj[:, 0], traj[:, 1], 'b-', lw=1.4)
            ax.set_title(f'{fam} s{seed} / {cfg_name}\n{r["outcome"]} '
                         f't={r["time_to_goal"]:.1f}s', fontsize=9)
            ax.set_xlim(-1.0, 7.0)
            ax.set_ylim(-4.0, 4.0)
    fig.tight_layout()
    _save(fig, out_path)
    return out_path


def _save(fig, out_path):
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_all(summary, stats, out_dir, *, montage=None):
    os.makedirs(out_dir, exist_ok=True)
    paths = [
        plot_success_bars(summary, os.path.join(out_dir, 'success_bars.png')),
        plot_e_vs_f(summary, stats, os.path.join(out_dir, 'e_vs_f.png')),
        plot_traj_montage(os.path.join(out_dir, 'traj_montage.png'),
                          montage=montage),
    ]
    return paths
