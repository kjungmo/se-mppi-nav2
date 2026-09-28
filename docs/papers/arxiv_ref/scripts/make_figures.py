#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Regenerate every data figure of the arXiv package as vector PDF.

Sources (all committed in this repository):
  * experiments/results_2d/summary.csv, stats.json -> success_bars.pdf, e_vs_f.pdf
  * experiments/prototype/run_validation.py (seed-fixed, deterministic)
        -> mechanism.pdf and data/mechanism_validation.json

The mechanism scenarios are re-run through the unmodified
``run_validation.run`` loop, so the metrics written to
``data/mechanism_validation.json`` are exactly the values that script prints.
Nothing is written under experiments/.

Needs numpy, matplotlib, scipy and osqp (e.g. the RoboStack ``ros2`` env):
    micromamba run -n ros2 python3 docs/papers/arxiv_ref/scripts/make_figures.py
"""

from __future__ import annotations

import csv
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(PKG, '..', '..', '..'))
RES = os.path.join(ROOT, 'experiments', 'results_2d')
FIG = os.path.join(PKG, 'figures')
DATA = os.path.join(PKG, 'data')

plt.rcParams.update({
    'pdf.fonttype': 42, 'ps.fonttype': 42,
    'font.family': 'serif', 'font.serif': ['DejaVu Serif'],
    'mathtext.fontset': 'dejavuserif',
    'font.size': 8, 'axes.titlesize': 8, 'axes.labelsize': 8,
    'xtick.labelsize': 7, 'ytick.labelsize': 7, 'legend.fontsize': 6.5,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.linewidth': 0.6, 'lines.linewidth': 1.2,
})

# Okabe-Ito colour-blind-safe palette.
C_SUCC, C_COLL, C_TOUT = '#009E73', '#D55E00', '#E69F00'
C_BLUE, C_GREY, C_PURP = '#0072B2', '#7F7F7F', '#CC79A7'

ORDER = ['A_stock', 'C_escape', 'D_cbf', 'E_indep', 'F_full', 'Fminus_nogap']
SHORT = {'A_stock': 'A', 'C_escape': 'C', 'D_cbf': 'D', 'E_indep': 'E',
         'F_full': 'F', 'Fminus_nogap': r'F$^{-}$'}
FAMILIES = ['utrap', 'clutter', 'dynamic', 'narrowdyn']
FAM_TITLE = {'utrap': 'U-trap', 'clutter': 'Clutter', 'dynamic': 'Dynamic',
             'narrowdyn': 'Narrow-dynamic'}


def load_summary():
    with open(os.path.join(RES, 'summary.csv')) as fh:
        rows = list(csv.DictReader(fh))
    out = {}
    for r in rows:
        out[(r['family'], r['config'])] = {
            k: float(r[k]) for k in ('success_rate', 'success_ci_low',
                                     'success_ci_high', 'collision_rate',
                                     'timeout_rate', 'n_trials')}
    return out


def load_stats():
    with open(os.path.join(RES, 'stats.json')) as fh:
        return json.load(fh)


def success_bars(summary):
    fig, axes = plt.subplots(1, 4, figsize=(7.0, 2.1), sharey=True)
    x = np.arange(len(ORDER))
    w = 0.27
    for i, (ax, fam) in enumerate(zip(axes, FAMILIES)):
        s = [summary[(fam, c)] for c in ORDER]
        succ = np.array([e['success_rate'] for e in s])
        lo = succ - np.array([e['success_ci_low'] for e in s])
        hi = np.array([e['success_ci_high'] for e in s]) - succ
        ax.bar(x - w, succ, w, color=C_SUCC, label='success',
               yerr=[np.maximum(lo, 0), np.maximum(hi, 0)],
               error_kw=dict(lw=0.6, capsize=1.5))
        ax.bar(x, [e['collision_rate'] for e in s], w, color=C_COLL,
               label='collision')
        ax.bar(x + w, [e['timeout_rate'] for e in s], w, color=C_TOUT,
               label='timeout')
        ax.set_xticks(x)
        ax.set_xticklabels([SHORT[c] for c in ORDER])
        ax.set_ylim(0, 1.05)
        ax.set_title(FAM_TITLE[fam])
        if i == 0:
            ax.set_ylabel('rate ($N=50$ per config)')
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc='upper center', ncol=3, frameon=False,
               bbox_to_anchor=(0.5, 1.0), handlelength=1.0)
    fig.tight_layout(pad=0.3, w_pad=0.6, rect=(0, 0, 1, 0.9))
    fig.savefig(os.path.join(FIG, 'success_bars.pdf'))
    plt.close(fig)


def e_vs_f(summary, stats):
    fig, axes = plt.subplots(1, 4, figsize=(7.0, 1.9), sharey=True)
    for i, (ax, fam) in enumerate(zip(axes, FAMILIES)):
        e, f = summary[(fam, 'E_indep')], summary[(fam, 'F_full')]
        x = np.array([0, 1])
        w = 0.36
        succ = np.array([e['success_rate'], f['success_rate']])
        lo = succ - np.array([e['success_ci_low'], f['success_ci_low']])
        hi = np.array([e['success_ci_high'], f['success_ci_high']]) - succ
        ax.bar(x - w / 2, succ, w, color=C_SUCC, label='success',
               yerr=[np.maximum(lo, 0), np.maximum(hi, 0)],
               error_kw=dict(lw=0.6, capsize=1.5))
        ax.bar(x + w / 2, [e['collision_rate'], f['collision_rate']], w,
               color=C_COLL, label='collision')
        ax.set_xticks(x)
        ax.set_xticklabels(['E (indep.)', 'F (coord.)'])
        ax.set_ylim(0, 1.18)
        comp = next(c for c in stats[fam]['comparisons']
                    if c['baseline'] == 'E_indep')
        ax.set_title(f"{FAM_TITLE[fam]}\nMcNemar $b/c$={comp['mcnemar_b']}/"
                     f"{comp['mcnemar_c']}, $p$={comp['mcnemar_p']:.3g}")
        if i == 0:
            ax.set_ylabel('rate')
    axes[0].legend(loc='upper left', frameon=False, ncol=2, handlelength=1.0)
    fig.tight_layout(pad=0.3, w_pad=0.6)
    fig.savefig(os.path.join(FIG, 'e_vs_f.pdf'))
    plt.close(fig)


# --------------------------------------------------------------------------- #
def mechanism():
    proto = os.path.join(ROOT, 'experiments', 'prototype')
    sys.path.insert(0, proto)
    import run_validation as rv  # noqa: E402  (writes nothing on import but mkdir)

    runs = {
        'utrap_stock': (rv.u_trap_world, rv.CFG_STOCK),
        'utrap_escape': (rv.u_trap_world, rv.CFG_ESC),
        'dynamic_nocbf': (rv.dynamic_world, rv.CFG_ESC),
        'dynamic_cbf': (rv.dynamic_world, rv.CFG_SE),
        'coord_F_utrap': (rv.u_trap_world, rv.CFG_SE),
        'coord_E_coordworld': (rv.coordination_world, rv.CFG_INDEP),
    }
    res = {k: rv.run(w(), c) for k, (w, c) in runs.items()}

    record = {}
    for k, r in res.items():
        record[k] = {
            'reached': bool(r['reached']), 'collided': bool(r['collided']),
            'steps': int(r['steps']), 'time_s': round(float(r['time_s']), 2),
            'min_clear_m': round(float(r['min_clear']), 4),
            'alpha_max': float(np.max(r['alphas'])) if len(r['alphas']) else None,
            'slack_max': (float(np.max(r['slacks'])) if len(r['slacks'])
                          else None),
            'entrapped_steps': int(np.sum(r['ent'])),
        }
    record['_source'] = ('experiments/prototype/run_validation.py run() with '
                         'seed=0, max_steps=400; regenerated by '
                         'docs/papers/arxiv_ref/scripts/make_figures.py')
    os.makedirs(DATA, exist_ok=True)
    with open(os.path.join(DATA, 'mechanism_validation.json'), 'w') as fh:
        json.dump(record, fh, indent=2, sort_keys=True)

    def draw_world(ax, world, moving=False):
        for o in world.obstacles:
            dyn = float(np.hypot(*o.v)) > 1e-9
            ax.add_patch(plt.Circle(o.p, o.r, color=C_TOUT if dyn else '0.55',
                                    lw=0))
            if dyn and moving:
                ax.plot([o.p[0], o.p[0]], [o.p[1], 2.4], ls=':', lw=0.8,
                        color=C_TOUT)
                ax.annotate('mover path', xy=(o.p[0] + 0.1, -2.5), fontsize=6,
                            color=C_TOUT)
        ax.plot(*world.goal, marker='*', color=C_SUCC, ms=8, ls='none')
        ax.plot(0, 0, 'ko', ms=3)
        ax.set_aspect('equal')
        ax.set_xlim(-0.5, 4.5)
        ax.set_ylim(-2.8, 2.4)
        ax.set_xlabel('$x$ (m)')

    fig, axes = plt.subplots(1, 4, figsize=(7.0, 1.95),
                             gridspec_kw=dict(width_ratios=[1, 1, 1.05, 1.05]))
    ax = axes[0]
    draw_world(ax, rv.u_trap_world())
    t = res['utrap_escape']['traj']
    ax.plot(t[:, 0], t[:, 1], color=C_BLUE, label='escape (reaches)')
    t = res['utrap_stock']['traj']
    ax.plot(t[:, 0], t[:, 1], color=C_COLL, lw=2.2, label='stock (stalls)')
    ax.plot(*t[-1], 's', color=C_COLL, ms=3.5)
    ax.set_ylabel('$y$ (m)')
    ax.set_title('(a) U-trap escape')
    ax.legend(loc='lower left', frameon=False, handlelength=1.4)

    ax = axes[1]
    draw_world(ax, rv.dynamic_world(), moving=True)
    t = res['dynamic_nocbf']['traj']
    ax.plot(t[:, 0], t[:, 1], color=C_COLL, lw=2.2, label='no CBF (collides)')
    ax.plot(*t[-1], 'x', color=C_COLL, ms=5)
    t = res['dynamic_cbf']['traj']
    ax.plot(t[:, 0], t[:, 1], color=C_BLUE, label='CBF (safe)')
    ax.set_title('(b) Crossing mover')
    ax.legend(loc='upper left', frameon=False, handlelength=1.4)

    r = res['coord_F_utrap']
    dt = 0.1
    ts = np.arange(len(r['alphas'])) * dt
    ent = r['ent'][:len(ts)].astype(bool)
    ax = axes[2]
    ax.fill_between(ts, 0, 7, where=ent, color=C_PURP, alpha=0.18, lw=0,
                    step='post', label='entrapped')
    ax.step(ts, r['alphas'], where='post', color=C_BLUE, label=r'$\alpha_t$')
    ax.set_ylim(0, 7)
    ax.set_xlabel('time (s)')
    ax.set_ylabel(r'CBF gain $\alpha$')
    ax.set_title(r'(c) Gain schedule, $\alpha{:}\,2\to6$')
    ax.legend(loc='upper right', frameon=False, handlelength=1.2)

    ax = axes[3]
    ts2 = np.arange(len(r['slacks'])) * dt
    ax.fill_between(ts2, 0, 1, where=ent[:len(ts2)], color=C_PURP, alpha=0.18,
                    lw=0, step='post', transform=ax.get_xaxis_transform())
    ax.plot(ts2, r['slacks'] * 1e6, color=C_BLUE)
    ax.set_xlabel('time (s)')
    ax.set_ylabel(r'QP slack $\delta$ ($\times10^{-6}$)')
    ax.set_title(r'(d) QP slack stays $\approx 0$')
    fig.tight_layout(pad=0.3, w_pad=0.5)
    fig.savefig(os.path.join(FIG, 'mechanism.pdf'))
    plt.close(fig)
    return record


def main():
    os.makedirs(FIG, exist_ok=True)
    summary, stats = load_summary(), load_stats()
    success_bars(summary)
    e_vs_f(summary, stats)
    rec = mechanism()
    for k, v in sorted(rec.items()):
        print(k, v)


if __name__ == '__main__':
    main()
