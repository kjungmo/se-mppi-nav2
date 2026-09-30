#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Generate every data table of the arXiv package from committed artifacts.

  experiments/results_2d/summary.csv  -> tables/bench_success.tex, bench_full.tex
  experiments/results_2d/stats.json   -> tables/bench_contrasts.tex, slack.tex
  data/mechanism_validation.json      -> tables/mechanism.tex

Standard library only.  ``python3 docs/papers/arxiv_ref/scripts/make_tables.py``
rewrites the files; ``--check`` exits non-zero if any committed table differs
from what the artifacts produce (used by scripts/check_paper_numbers.py).
"""

from __future__ import annotations

import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.abspath(os.path.join(PKG, '..', '..', '..'))
RES = os.path.join(ROOT, 'experiments', 'results_2d')
TAB = os.path.join(PKG, 'tables')

ORDER = ['A_stock', 'C_escape', 'D_cbf', 'E_indep', 'F_full', 'Fminus_nogap']
LABEL = {'A_stock': 'A\\,stock MPPI', 'C_escape': 'C\\,escape only',
         'D_cbf': 'D\\,CBF only', 'E_indep': 'E\\,escape+CBF (indep.)',
         'F_full': '\\textbf{F\\,\\sname{} (coord.)}',
         'Fminus_nogap': 'F$^{-}$\\,F without gap search'}
FAMILIES = ['utrap', 'clutter', 'dynamic', 'narrowdyn']
FAM = {'utrap': 'U-trap', 'clutter': 'Clutter', 'dynamic': 'Dynamic',
       'narrowdyn': 'Narrow-dyn.'}


def summary():
    with open(os.path.join(RES, 'summary.csv')) as fh:
        return {(r['family'], r['config']): r for r in csv.DictReader(fh)}


def stats():
    with open(os.path.join(RES, 'stats.json')) as fh:
        return json.load(fh)


def pct(x):
    return f'{round(100 * float(x)):d}'


def pfmt(p):
    """p-value as it is quoted in the paper: 1 / 0.109 / 1.4\\times10^{-9}."""
    if p >= 0.9995:
        return '1'
    if p >= 1e-3:
        return f'{p:.3f}'
    m, e = f'{p:.1e}'.split('e')
    return f'${m}\\!\\times\\!10^{{{int(e)}}}$'


def comp(st, fam, base):
    return next(c for c in st[fam]['comparisons'] if c['baseline'] == base)


def t_success(sm):
    lines = [
        '\\begin{table}[t]', '\\centering',
        '\\caption{\\textbf{Main result: success rate} in \\% with Wilson 95\\% '
        'confidence interval, $N=50$ paired scenarios per cell '
        '($1{,}200$ trials of the Python 2D mirror, not the Nav2 controller). '
        'Best per family in bold. On the two trap families '
        'the escape configurations (C, E, F) separate from stock (A), CBF only '
        '(D) and the no-gap ablation (F$^{-}$), which all stay at $0\\%$. '
        'Source: \\texttt{experiments/results\\_2d/summary.csv}.}',
        '\\label{tab:main}', '\\small',
        '\\begin{tabular}{lcccc}', '\\toprule',
        'Configuration & ' + ' & '.join(FAM[f] for f in FAMILIES) + ' \\\\',
        '\\midrule']
    best = {f: max(float(sm[(f, c)]['success_rate']) for c in ORDER)
            for f in FAMILIES}
    for c in ORDER:
        cells = []
        for f in FAMILIES:
            r = sm[(f, c)]
            v = pct(r['success_rate'])
            if float(r['success_rate']) == best[f]:
                v = f'\\textbf{{{v}}}'
            cells.append(f'{v}\\,{{\\scriptsize[{pct(r["success_ci_low"])},'
                         f'{pct(r["success_ci_high"])}]}}')
        lines.append(f'{LABEL[c]} & ' + ' & '.join(cells) + ' \\\\')
    lines += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    return '\n'.join(lines) + '\n'


def t_contrasts(st):
    lines = [
        '\\begin{table}[t]', '\\centering',
        '\\caption{\\textbf{Paired contrasts against F} (McNemar on success, '
        'Holm-adjusted within each family of 16 tests), Python 2D mirror. '
        '$b$ = trials the '
        'baseline solves and F does not; $c$ = the reverse. $^{*}$ marks '
        '$p_{\\mathrm{adj}}<0.05$. The escape contrast (A$\\to$F) is decisive '
        'on both trap families; the coordination contrast (E$\\to$F) is null '
        'everywhere. Source: \\texttt{experiments/results\\_2d/stats.json}.}',
        '\\label{tab:contrasts}', '\\small',
        '\\begin{tabular}{lcccccc}', '\\toprule',
        ' & \\multicolumn{2}{c}{Escape (A$\\to$F)} & '
        '\\multicolumn{2}{c}{CBF cost (C$\\to$F)} & '
        '\\multicolumn{2}{c}{Coordination (E$\\to$F)} \\\\',
        '\\cmidrule(lr){2-3}\\cmidrule(lr){4-5}\\cmidrule(lr){6-7}',
        'Family & $b/c$ & $p_{\\mathrm{adj}}$ & $b/c$ & $p_{\\mathrm{adj}}$ & '
        '$b/c$ & $p_{\\mathrm{adj}}$ \\\\', '\\midrule']
    for f in FAMILIES:
        cells = []
        for base in ('A_stock', 'C_escape', 'E_indep'):
            c = comp(st, f, base)
            p = pfmt(c['mcnemar_p_adj'])
            if c['mcnemar_reject']:
                p += '$^{*}$'
            cells += [f"{c['mcnemar_b']}/{c['mcnemar_c']}", p]
        lines.append(f'{FAM[f]} & ' + ' & '.join(cells) + ' \\\\')
    lines += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    return '\n'.join(lines) + '\n'


def num(x, nd=2):
    return '--' if x in ('', None) else f'{float(x):.{nd}f}'


def t_full(sm):
    lines = [
        '\\begin{table}[!htbp]', '\\centering',
        '\\caption{\\textbf{Full per-family results} of the Python 2D-mirror benchmark '
        '($N=50$ per cell). Rates in \\%; time-to-goal and path length are '
        'means over successful trials only (\\,$\\pm$ 95\\% CI half-width); '
        'minimum clearance is over all trials. Source: '
        '\\texttt{experiments/results\\_2d/summary.csv}.}',
        '\\label{tab:full}', '\\footnotesize', '\\setlength{\\tabcolsep}{4pt}',
        '\\begin{tabular}{llcccccc}', '\\toprule',
        'Family & Config & Succ. & Coll. & Timeout & Time-to-goal (s) & '
        'Path (m) & Min.\\ clear.\\ (m) \\\\', '\\midrule']
    for i, f in enumerate(FAMILIES):
        if i:
            lines.append('\\midrule')
        for j, c in enumerate(ORDER):
            r = sm[(f, c)]
            fam = f'\\multirow{{6}}{{*}}{{{FAM[f]}}}' if j == 0 else ''
            short = c.split('_')[0].replace('Fminus', 'F$^{-}$')
            ttg = ('--' if r['time_to_goal_mean'] == '' else
                   f"{num(r['time_to_goal_mean'])}\\stdv{{{num(r['time_to_goal_ci'])}}}")
            pl = ('--' if r['path_length_mean'] == '' else
                  f"{num(r['path_length_mean'])}\\stdv{{{num(r['path_length_ci'])}}}")
            mc = f"{num(r['min_clearance_mean'])}\\stdv{{{num(r['min_clearance_ci'])}}}"
            lines.append(f"{fam} & {short} & {pct(r['success_rate'])} & "
                         f"{pct(r['collision_rate'])} & {pct(r['timeout_rate'])} & "
                         f'{ttg} & {pl} & {mc} \\\\')
    lines += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    return '\n'.join(lines) + '\n'


def t_slack(st):
    lines = [
        '\\begin{table}[!htbp]', '\\centering',
        '\\caption{\\textbf{CBF slack usage} in the Python 2D mirror. Trials (of 50) in which the QP '
        'relaxed a barrier row at least once, and the largest per-trial '
        '\\texttt{slack\\_max}. Configurations without a CBF (A, C) are zero by '
        'construction; on the mover-free families the CBF sees no obstacle. '
        'Source: \\texttt{experiments/results\\_2d/stats.json} '
        '(\\texttt{slack\\_usage}).}',
        '\\label{tab:slack}', '\\small',
        '\\begin{tabular}{lcccccc}', '\\toprule',
        'Family & ' + ' & '.join(
            c.split('_')[0].replace('Fminus', 'F$^{-}$') for c in ORDER) +
        ' \\\\', '\\midrule']
    for f in FAMILIES:
        cells = []
        for c in ORDER:
            u = st[f]['slack_usage'][c]
            if u['n_slack_pos'] == 0:
                cells.append('0')
            else:
                cells.append(f"{u['n_slack_pos']} ({u['slack_max_max']:.3f})")
        lines.append(f'{FAM[f]} & ' + ' & '.join(cells) + ' \\\\')
    lines += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    return '\n'.join(lines) + '\n'


def t_mechanism():
    with open(os.path.join(PKG, 'data', 'mechanism_validation.json')) as fh:
        m = json.load(fh)
    rows = [('U-trap', 'stock MPPI (A)', 'utrap_stock'),
            ('U-trap', 'escape (C)', 'utrap_escape'),
            ('Crossing mover', 'escape, no CBF (C)', 'dynamic_nocbf'),
            ('Crossing mover', '\\sname{} (F)', 'dynamic_cbf'),
            ('U-trap', '\\sname{} (F)', 'coord_F_utrap'),
            ('U-trap + mover', 'independent (E)', 'coord_E_coordworld')]
    lines = [
        '\\begin{table}[t]', '\\centering',
        '\\caption{\\textbf{Mechanism validation} (single seed-fixed runs of '
        'the 2D mirror; 40\\,s budget). ``Stalled\'\' = budget exhausted without '
        'progress. Source: \\texttt{data/mechanism\\_validation.json}, '
        'regenerated from \\texttt{experiments/prototype/run\\_validation.py}.}',
        '\\label{tab:mechanism}', '\\small',
        '\\begin{tabular}{llccccc}', '\\toprule',
        'Scenario & Configuration & Reached & Collided & Time (s) & '
        'Min.\\ clear.\\ (m) & $\\max_t\\alpha_t$ \\\\', '\\midrule']
    for scen, cfg, key in rows:
        r = m[key]
        reached = 'yes' if r['reached'] else ('no' if r['collided'] else
                                               'no (stalled)')
        coll = 'yes' if r['collided'] else 'no'
        t = f"{r['time_s']:.1f}" if r['reached'] else '--'
        mc = r['min_clear_m']
        mcs = f'{mc:.2f}' if abs(mc) >= 0.005 else f'{mc:.2f}'.replace('-', '$-$')
        lines.append(f"{scen} & {cfg} & {reached} & {coll} & {t} & {mcs} & "
                     f"{r['alpha_max']:.0f} \\\\")
    lines += ['\\bottomrule', '\\end{tabular}', '\\end{table}']
    return '\n'.join(lines) + '\n'


def build():
    sm, st = summary(), stats()
    return {'bench_success.tex': t_success(sm),
            'bench_contrasts.tex': t_contrasts(st),
            'bench_full.tex': t_full(sm), 'slack.tex': t_slack(st),
            'mechanism.tex': t_mechanism()}


def main():
    out = build()
    if '--check' in sys.argv:
        bad = [k for k, v in out.items()
               if not os.path.exists(os.path.join(TAB, k))
               or open(os.path.join(TAB, k)).read() != v]
        if bad:
            print('tables out of sync with artifacts:', ', '.join(bad))
            sys.exit(1)
        print('tables in sync with artifacts')
        return
    os.makedirs(TAB, exist_ok=True)
    for k, v in out.items():
        with open(os.path.join(TAB, k), 'w') as fh:
            fh.write(v)
    print('wrote', ', '.join(sorted(out)))


if __name__ == '__main__':
    main()
