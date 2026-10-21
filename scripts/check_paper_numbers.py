#!/usr/bin/env python3
"""Number guard: every headline figure quoted in the paper must trace to a
committed artifact, and retired/forbidden tokens must be absent.

Run from the repo root:  python3 scripts/check_paper_numbers.py
With --arxiv, the same guard runs on the arXiv package in
docs/papers/arxiv_ref/ (all section/figure/table sources concatenated), plus
package-specific traces: generated tables in sync with the artifacts, the
mechanism numbers against data/mechanism_validation.json, the live-pilot
times against experiments/results_pilot/, and the attribution rules.
Exit 0 = green. Any assertion failure names the violated trace.
"""
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ARXIV = '--arxiv' in sys.argv
ARXIV_DIR = ROOT / 'docs/papers/arxiv_ref'
TEX = (ARXIV_DIR / 'main.tex') if ARXIV else ROOT / 'docs/papers/latex/main.tex'
STATS = ROOT / 'experiments/results_2d/stats.json'
SUMMARY = ROOT / 'experiments/results_2d/summary.csv'
TESTS_DIR = ROOT / 'src/nav2_se_controller/test'
PILOT_LAUNCH = ROOT / 'experiments/results_pilot/launch_s1_utrap.log'

PAPER1_TEST_FILES = [
    'test_entrapment_detector.cpp', 'test_cbf_safety_filter.cpp',
    'test_escape_safety_coordinator.cpp', 'test_dynamic_obstacle_tracker.cpp',
    'test_gap_search.cpp', 'test_repulsion.cpp', 'test_path_progress.cpp',
    'test_plugin_load.cpp',
]

# Tokens that must never appear in shipped files. Stored base64-encoded so
# this checker itself never carries them in readable form.
import base64

_FORBIDDEN_B64 = [
    'WmVyb1dvcmtz', '7KCc66Gc7Iuh7Iqk',  # attribution rule (company names)
    'UkVTVUxUUy1TTE9U', 'VE9ETw==',      # placeholders / retired markers
    'cGVuZGluZw==',
    'Q2xhdWRl', 'Y2xhdWRl',              # AI signatures
    'emVyb193cw==', 'QnVuZ1A=',          # internal paths / codenames
]
FORBIDDEN_IN_TEX = [base64.b64decode(t).decode() for t in _FORBIDDEN_B64]

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)


if ARXIV:
    tex = '\n'.join(p.read_text() for p in
                    [TEX] + sorted(ARXIV_DIR.glob('sections/*.tex')) +
                    sorted(ARXIV_DIR.glob('figures/*.tex')) +
                    sorted(ARXIV_DIR.glob('tables/*.tex')))
else:
    tex = TEX.read_text()
stats = json.loads(STATS.read_text())


def cmp_row(family, baseline):
    for c in stats[family]['comparisons']:
        if c['baseline'] == baseline:
            return c
    raise KeyError((family, baseline))


# --- headline success rates trace to stats.json ---
u = cmp_row('utrap', 'A_stock')
check(abs(u['f_success_rate'] - 0.88) < 1e-9 and u['base_success_rate'] == 0.0,
      'utrap F=88%/A=0% no longer matches stats.json')
check('88' in tex and re.search(r'0\\?%[^0-9]{0,40}88', tex) or ('0\\%' in tex and '88\\%' in tex),
      'paper does not quote the utrap 0%->88% contrast')
n = cmp_row('narrowdyn', 'A_stock')
check(abs(n['f_success_rate'] - 0.62) < 1e-9 and n['base_success_rate'] == 0.0,
      'narrowdyn F=62%/A=0% no longer matches stats.json')
check('62\\%' in tex, 'paper does not quote the narrowdyn 62% figure')

# --- headline p-values trace to stats.json (rounded forms used in prose) ---
check(f"{u['mcnemar_p_adj']:.2e}".startswith('1.44e-09') or abs(u['mcnemar_p_adj'] - 1.443e-09) < 2e-11,
      'utrap adj p is no longer ~1.44e-9')
check('10^{-9}' in tex or '1.4' in tex, 'paper lost the utrap p~10^-9 quote')
check(abs(n['mcnemar_p_adj'] - 1.139e-06) < 2e-8, 'narrowdyn adj p is no longer ~1.1e-6')
check('10^{-6}' in tex, 'paper lost the narrowdyn p~10^-6 quote')

# --- E-vs-F null: McNemar p == 1 in every family ---
for fam in stats:
    e = cmp_row(fam, 'E_indep')
    check(e['mcnemar_p'] == 1.0, f'E-vs-F McNemar p != 1 in family {fam} — null claim broken')

# --- trial count ---
n_trials = sum(1 for _ in open(ROOT / 'experiments/results_2d/trials.csv')) - 1
check(n_trials == 1200, f'trials.csv has {n_trials} rows, paper claims 1,200')
check('1,200' in tex or '1200' in tex or '1{,}200' in tex, 'paper lost the 1,200-trial count')

# --- test-count claims (Sec. V) ---
paper1 = sum(open(TESTS_DIR / f).read().count('\nTEST') + open(TESTS_DIR / f).read().startswith('TEST')
             for f in PAPER1_TEST_FILES)
check(paper1 == 44, f'Paper-1 module TEST count is {paper1}, paper claims 44')
check('44' in tex, 'paper lost the 44-unit-test claim')
check('82' in tex and '13 files' in tex, 'paper lost the 82-TEST/13-file claim')
all_tests = sum(open(p).read().count('\nTEST') for p in TESTS_DIR.glob('test_*.cpp'))
n_files = len(list(TESTS_DIR.glob('test_*.cpp')))
check(all_tests == 82, f'total TEST count is {all_tests}, paper claims 82')
check(n_files == 13, f'{n_files} test files, paper claims 13')

# --- live claims (Sec. VI-B) stay within committed evidence ---
# The narrowed claim: load/activate/valid-commands (traced to committed pilot
# logs), plus "injection ran live once" traced to the committed regression test
# that replays the live-captured tensor shapes. Overclaim phrases that would
# require an unpreserved log are forbidden.
check((TESTS_DIR / 'test_escape_injection_live_shapes.cpp').exists(),
      'live-injection regression test missing — Sec. VI-B claim loses its artifact')
check(PILOT_LAUNCH.exists(),
      'committed pilot launch log missing — load/activate claim loses its artifact')
for phrase in ('fires live', 'injects escape costs live',
               'detects entrapment, injects escape costs'):
    check(phrase not in tex,
          f'Sec. VI-B overclaim reintroduced without a committed log: {phrase!r}')

# --- arXiv package: package-specific traces ---
if ARXIV:
    r = subprocess.run([sys.executable, str(ARXIV_DIR / 'scripts/make_tables.py'),
                        '--check'], capture_output=True, text=True)
    check(r.returncode == 0, 'generated tables out of sync with results_2d / '
          'mechanism_validation.json: ' + r.stdout.strip())
    mech = json.loads((ARXIV_DIR / 'data/mechanism_validation.json').read_text())
    check(mech['utrap_escape']['reached'] and mech['utrap_escape']['time_s'] == 27.7,
          'mechanism U-trap escape no longer 27.7 s')
    check(not mech['utrap_stock']['reached'] and not mech['utrap_stock']['collided'],
          'mechanism stock run no longer stalls')
    check(mech['dynamic_nocbf']['collided'] and mech['dynamic_nocbf']['time_s'] == 4.7,
          'mechanism no-CBF collision no longer at 4.7 s')
    check(mech['dynamic_cbf']['reached'] and mech['dynamic_cbf']['time_s'] == 18.6
          and mech['dynamic_cbf']['min_clear_m'] >= 0, 'mechanism CBF run changed')
    check(mech['coord_F_utrap']['alpha_max'] == 6.0 and
          mech['coord_F_utrap']['slack_max'] < 2e-6, 'coordination trace changed')
    check(mech['coord_E_coordworld']['time_s'] == mech['coord_F_utrap']['time_s'] == 27.7,
          'E/F mechanism times no longer equal')
    for tok in ('27.7', '4.7', '18.6', '2\\times10^{-6}'):
        check(tok in tex, f'mechanism number {tok!r} missing from tex')
    pilot = ROOT / 'experiments/results_pilot/barn/F_se_full'
    p1 = json.loads((pilot / 's1_utrap_seed0.json').read_text())
    p2 = json.loads((pilot / 's1_utrap_evidence_seed0.json').read_text())
    check(p1['outcome'] == 'STUCK' and p1['metrics']['time_to_goal'] == 254.5
          and not p1['metrics']['collided'], 'pilot STUCK 254.5 s trace broken')
    check(p2['outcome'] == 'TIMEOUT' and p2['metrics']['time_to_goal'] == 299.8
          and not p2['metrics']['collided'], 'pilot TIMEOUT 299.8 s trace broken')
    check('254.5' in tex and '299.8' in tex, 'paper lost the pilot times')
    check(p1['metrics']['compute']['n'] == 0 and p2['metrics']['compute']['n'] == 0,
          'pilot compute telemetry is no longer empty -- revisit Cost Analysis')
    summ = {(r_['family'], r_['config']): r_ for r_ in
            __import__('csv').DictReader(open(SUMMARY))}
    ttg_a = float(summ[('clutter', 'A_stock')]['time_to_goal_mean'])
    ttg_f = float(summ[('clutter', 'F_full')]['time_to_goal_mean'])
    check(round(ttg_a, 1) == 15.2 and round(ttg_f, 1) == 17.7
          and round(ttg_f - ttg_a, 1) == 2.4, 'clutter time-to-goal trace broken')
    check('15.2' in tex and '17.7' in tex and '2.4' in tex, 'clutter TTG quote missing')
    coll = {c: float(summ[('dynamic', c)]['collision_rate']) for c in
            ('A_stock', 'D_cbf', 'E_indep', 'F_full', 'Fminus_nogap')}
    check(coll['A_stock'] == 0.2 and min(coll[c] for c in coll if c != 'A_stock') == 0.16
          and max(coll[c] for c in coll if c != 'A_stock') == 0.18,
          'dynamic collision 20% -> 16-18% trace broken')
    nd = {c: float(summ[('narrowdyn', c)]['collision_rate']) for c in
          ('A_stock', 'C_escape', 'D_cbf', 'E_indep', 'F_full', 'Fminus_nogap')}
    check(nd['A_stock'] == nd['C_escape'] == 0.18 and
          all(nd[c] == 0.32 for c in ('D_cbf', 'E_indep', 'F_full', 'Fminus_nogap')),
          'narrowdyn collision 18% -> 32% trace broken')
    cf = cmp_row('narrowdyn', 'C_escape')
    check((cf['mcnemar_b'], cf['mcnemar_c']) == (8, 0) and round(cf['mcnemar_p'], 3) == 0.008
          and round(cf['mcnemar_p_adj'], 3) == 0.109, 'narrowdyn C->F 8/0 trace broken')
    deltas = [abs(v['cliffs_delta']) for fam in stats
              for v in cmp_row(fam, 'E_indep')['continuous'].values()]
    check(max(deltas) <= 0.032, 'E-vs-F max |Cliff delta| exceeds 0.032')
    su = {f: stats[f]['slack_usage'] for f in stats}
    check(su['narrowdyn']['E_indep']['n_slack_pos'] == su['narrowdyn']['F_full']['n_slack_pos'] == 34
          and su['narrowdyn']['D_cbf']['n_slack_pos'] == su['narrowdyn']['Fminus_nogap']['n_slack_pos'] == 26
          and round(su['narrowdyn']['F_full']['slack_max_max'], 2) == 0.47,
          'narrowdyn slack usage trace broken')
    dyn_pos = [su['dynamic'][c]['n_slack_pos'] for c in ('D_cbf', 'E_indep', 'F_full', 'Fminus_nogap')]
    check(min(dyn_pos) == 47 and max(dyn_pos) == 48 and
          round(su['dynamic']['F_full']['slack_max_max'], 3) == 0.015, 'dynamic slack trace broken')
    check(all(su[f][c]['n_slack_pos'] == 0 for f in ('utrap', 'clutter') for c in su[f]),
          'mover-free families no longer slack-free')
    c_u = float(summ[('utrap', 'C_escape')]['success_rate'])
    c_n = float(summ[('narrowdyn', 'C_escape')]['success_rate'])
    check(c_u == 0.9 and c_n == 0.78, 'C success 90%/78% trace broken')
    check('88$--$90' in tex and '62$--$78' in tex, 'range quotes 88-90 / 62-78 missing')
    for f in ('utrap', 'narrowdyn'):
        check(float(summ[(f, 'Fminus_nogap')]['success_rate']) == 0.0,
              f'F- no longer 0% on {f}')
    # attribution rules (STYLE_GUIDE)
    check('\\author[1]{Jungmo Kang}' in tex and
          'Independent Researcher, Seoul, Republic of Korea' in tex,
          'author block deviates from the style guide')
    # forbidden tokens are stored only as (length, SHA-256) so the names
    # themselves never appear in the public tree; every substring of that
    # length is hashed and compared.
    FORBIDDEN = [(8, 'cc2ecc637f1a8e972069134ef0c9927d9a1b97f20db545da06f5371db50e6226'), (4, '1dc38b0fb60171a93f1666514958be9a7be00621aa98556c076591f48f93c5d2')]
    for p in ARXIV_DIR.rglob('*'):
        if p.is_file() and p.suffix in ('.tex', '.bib', '.md', '.json', '.py', '.cls'):
            txt = p.read_text(errors='ignore').lower()
            for n, h in FORBIDDEN:
                hit = any(hashlib.sha256(txt[k:k + n].encode()).hexdigest() == h
                          for k in range(len(txt) - n + 1))
                check(not hit, f'forbidden token (len {n}) in {p.relative_to(ROOT)}')


# =====================================================================
# SE-MPPI v2 paper (default mode): the main text describes the shipped
# algorithm; the paper-v1 numbers live in the appendix only. Every number
# below is re-read from a committed artifact.
# =====================================================================
if not ARXIV:
    import csv
    import gzip

    EXP = ROOT / 'experiments'
    # ---- split the source: main text / design-stage appendix ----
    split_at = tex.index('\\section{Design-stage benchmark')
    main_tex, app_tex = tex[:split_at], tex[split_at:]
    flat = re.sub(r'\s+', ' ', tex)

    def table_rows(label):
        i = tex.index('\\label{' + label + '}')
        j = tex.index('\\midrule', i)
        k = tex.index('\\bottomrule', j)
        rows = []
        for line in tex[j + len('\\midrule'):k].split('\\\\'):
            line = line.strip()
            if line:
                rows.append([c.strip() for c in line.split('&')])
        return rows

    def pct(r, lo, hi):
        return f'{round(float(r) * 100)} [{round(float(lo) * 100)},{round(float(hi) * 100)}]'

    def load_arm(d):
        s = json.loads((EXP / d / 'stats.json').read_text())
        summ = {(r_['family'], r_['config']): r_ for r_ in csv.DictReader(open(EXP / d / 'summary.csv'))}
        n = sum(1 for _ in open(EXP / d / 'trials.csv')) - 1
        return s, summ, n

    v1s, v1m, v1n = load_arm('results_2d')
    v2s, v2m, v2n = load_arm('results_2d_v2')
    v2as, v2am, v2an = load_arm('results_2d_v2_advised')
    for name, nn in (('results_2d', v1n), ('results_2d_v2', v2n), ('results_2d_v2_advised', v2an)):
        check(nn == 1200, f'{name}/trials.csv has {nn} rows, paper claims 1,200 per arm')
    for name, st in (('v1', v1s), ('v2', v2s), ('v2_advised', v2as)):
        for fam in st:
            e = [c for c in st[fam]['comparisons'] if c['baseline'] == 'E_indep'][0]
            check(e['mcnemar_p'] == 1.0, f'E-vs-F McNemar p != 1 in {name}/{fam}')
    # the v2 coordination claims: 0/0 in every family; |delta| bounds
    for name, st, bound in (('v2', v2s, 0.002), ('v2_advised', v2as, 0.005)):
        for fam in st:
            e = [c for c in st[fam]['comparisons'] if c['baseline'] == 'E_indep'][0]
            check((e['mcnemar_b'], e['mcnemar_c']) == (0, 0), f'{name}/{fam} E-vs-F not 0/0')
            check(max(abs(v['cliffs_delta']) for v in e['continuous'].values()) <= bound,
                  f'{name}/{fam} E-vs-F |Cliff delta| above {bound}')

    CFG = ['A_stock', 'C_escape', 'D_cbf', 'E_indep', 'F_full', 'Fminus_nogap']
    FAM = ['utrap', 'clutter', 'dynamic', 'narrowdyn']

    def success_table(label, summ):
        rows = table_rows(label)
        check(len(rows) == 6, f'{label}: expected 6 rows')
        for row, cfg in zip(rows, CFG):
            exp = [pct(summ[(f, cfg)]['success_rate'], summ[(f, cfg)]['success_ci_low'],
                       summ[(f, cfg)]['success_ci_high']) for f in FAM]
            check(row[1:] == exp, f'{label}: row {cfg} is {row[1:]}, artifact says {exp}')

    success_table('tab:bench2d-success', v2m)
    success_table('tab:v1-success', v1m)
    rows = table_rows('tab:bench2d-arms')
    for row, summ in zip(rows, (v2m, v2am)):
        exp = [pct(summ[(f, 'F_full')]['success_rate'], summ[(f, 'F_full')]['success_ci_low'],
                   summ[(f, 'F_full')]['success_ci_high']) for f in FAM]
        check(row[1:] == exp, f'tab:bench2d-arms row {row[0]} differs from the artifact {exp}')

    def pcell(p):
        if p >= 0.995:
            return '1'
        star = '\\,*' if p < 0.05 else ''
        if p < 0.01:
            m, e = f'{p:.1e}'.split('e')
            return f'${m}\\!\\times\\!10^{{{int(e)}}}$' + star
        return f'${p:.3f}$' + star

    def stats_table(label, st):
        rows = table_rows(label)
        for row, fam in zip(rows, FAM if label == 'tab:bench2d-stats' else FAM):
            pass
        order = ['utrap', 'clutter', 'dynamic', 'narrowdyn']
        check(len(rows) == 4, f'{label}: expected 4 rows')
        for row, fam in zip(rows, order):
            a = [c for c in st[fam]['comparisons'] if c['baseline'] == 'A_stock'][0]
            e = [c for c in st[fam]['comparisons'] if c['baseline'] == 'E_indep'][0]
            exp = [f"{a['mcnemar_b']}/{a['mcnemar_c']}", pcell(a['mcnemar_p_adj']),
                   f"{e['mcnemar_b']}/{e['mcnemar_c']}", pcell(e['mcnemar_p_adj'])]
            check(row[1:] == exp, f'{label}: {fam} row {row[1:]} differs from {exp}')

    stats_table('tab:bench2d-stats', v2s)
    stats_table('tab:v1-stats', v1s)

    # ---- numbers quoted in prose, shipped arm ----
    f2 = lambda fam, cfg: v2m[(fam, cfg)]
    check(f2('utrap', 'F_full')['n_success'] == '40' and f2('utrap', 'F_full')['success_rate'] == '0.8',
          'v2 utrap F is no longer 40 of 50')
    check(v2am[('utrap', 'F_full')]['n_success'] == '29', 'v2_advised utrap F is no longer 29 of 50')
    check(v2am[('narrowdyn', 'F_full')]['success_rate'] == '0.44' and
          f2('narrowdyn', 'F_full')['success_rate'] == '0.6', 'narrowdyn 60%/44% trace broken')
    check('40 of 50' in flat and '29 of 50' in flat, 'paper lost the 40/29 of 50 headline counts')
    u = [c for c in v2s['utrap']['comparisons'] if c['baseline'] == 'A_stock'][0]
    check(abs(u['mcnemar_p_adj'] - 1.1175e-8) < 1e-11 and u['mcnemar_b'] == 0 and u['mcnemar_c'] == 40,
          'v2 utrap A->F trace broken')
    c_cl = [c for c in v2s['clutter']['comparisons'] if c['baseline'] == 'A_stock'][0]
    check(c_cl['mcnemar_c'] == 10 and round(c_cl['mcnemar_p_adj'], 3) == 0.031, 'v2 clutter A->F trace broken')
    ttg = float(v2m[('clutter', 'F_full')]['time_to_goal_mean']) - float(v2m[('clutter', 'A_stock')]['time_to_goal_mean'])
    check(round(ttg, 1) == 4.8 and '4.8' in flat, 'v2 clutter time cost trace (4.8 s) broken')
    check(float(v2m[('clutter', 'F_full')]['success_rate']) == 0.72 and float(v2am[('clutter', 'F_full')]['success_rate']) == 0.62,
          'clutter 72%/62% trace broken')
    nd = [c for c in v2s['narrowdyn']['comparisons'] if c['baseline'] == 'C_escape'][0]
    check((nd['mcnemar_b'], nd['mcnemar_c']) == (6, 0) and round(nd['mcnemar_p'], 3) == 0.031
          and round(nd['mcnemar_p_adj'], 3) == 0.438, 'v2 narrowdyn C->F trace broken')
    dyn = [c for c in v2s['dynamic']['comparisons'] if c['baseline'] == 'A_stock'][0]
    check((dyn['mcnemar_b'], dyn['mcnemar_c']) == (0, 2), 'v2 dynamic A->F trace broken')
    coll = lambda fam, cfg: float(v2m[(fam, cfg)]['collision_rate'])
    check(coll('dynamic', 'A_stock') == 0.2 and all(coll('dynamic', c) == 0.16 for c in ('D_cbf', 'E_indep', 'F_full', 'Fminus_nogap')),
          'v2 dynamic collision 20% -> 16% trace broken')
    check(coll('narrowdyn', 'A_stock') == coll('narrowdyn', 'C_escape') == 0.18 and
          all(coll('narrowdyn', c) == 0.32 for c in ('D_cbf', 'E_indep', 'F_full', 'Fminus_nogap')),
          'v2 narrowdyn collision 18% -> 32% trace broken')
    su = {f: v2s[f]['slack_usage'] for f in v2s}
    check(all(su[f][c]['n_slack_pos'] == 0 for f in ('utrap', 'clutter') for c in su[f]), 'v2 mover-free families not slack-free')
    check(su['narrowdyn']['E_indep']['n_slack_pos'] == su['narrowdyn']['F_full']['n_slack_pos'] == 35 and
          su['narrowdyn']['D_cbf']['n_slack_pos'] == su['narrowdyn']['Fminus_nogap']['n_slack_pos'] == 26 and
          round(su['narrowdyn']['F_full']['slack_max_max'], 2) == 0.47, 'v2 narrowdyn slack trace broken')
    dp = [su['dynamic'][c]['n_slack_pos'] for c in ('D_cbf', 'E_indep', 'F_full', 'Fminus_nogap')]
    check(min(dp) == 47 and max(dp) == 48 and round(su['dynamic']['F_full']['slack_max_max'], 3) == 0.014,
          'v2 dynamic slack trace broken')
    check('$26$--$35$' in tex and '$35/50$' in tex and '$0.014$' in tex, 'slack quotes missing from the paper')

    # ---- v1 numbers only in the appendix ----
    for tok in ('88\\%', '88$--$90', '62$--$78', '1.4\\!\\times', '1.1\\!\\times\\!10^{-6}', '27.7', '18.6', '90 [79'):
        check(tok not in main_tex, f'design-stage (paper-v1) number {tok!r} appears outside the appendix')
    check('88$--$90' in app_tex and '62$--$78' in app_tex, 'appendix lost the paper-v1 quotes')

    # ---- campaign 2 ----
    c2 = json.loads(gzip.open(EXP / 'campaign2/results.json.gz').read())
    cells = {(c['scenario'], c['config']): c for c in c2['cells']}
    rows = table_rows('tab:c2cells')
    for row, scen in zip(rows, ['s0_open', 's1_utrap', 's3_ctrap', 's4_ltrap']):
        exp = []
        for cfg in ('C1', 'C2', 'C3', 'C4'):
            c = cells.get((scen, cfg))
            if c is None:
                exp.append('--')
            else:
                check(c['n_valid'] == 6 and c['n_trials'] == 6, f'{scen}/{cfg} not 6 valid trials')
                lo, hi = c['wilson95']
                exp.append(f"{c['successes']} [{lo:.2f},{hi:.2f}]")
        check(row[1:] == exp, f'tab:c2cells row {scen}: {row[1:]} differs from {exp}')
    contr = {c['name']: c for c in c2['contrasts']}
    crow = table_rows('tab:c2contrasts')
    order = ['C4 vs C3 on s1_utrap', 'C4 vs C3 on s3_ctrap', 'C2 vs C1 on s1_utrap', 'C2 vs C1 on s3_ctrap',
             'C4 vs C3 on s4_ltrap', 'C4 on s3_ctrap vs C4 on s1_utrap (transfer)',
             'C4 on s4_ltrap vs C4 on s1_utrap (transfer)']
    check(len(crow) == 7, 'tab:c2contrasts: expected 7 rows')
    for row, name in zip(crow, order):
        c = contr[name]
        exp = [str(c['a_successes']), str(c['b_successes']), f"{c['fisher_p']:.4f}"]
        check(row[1:] == exp, f'tab:c2contrasts {name}: {row[1:]} differs from {exp}')
    dec = c2['decision']
    check(dec['branch'] == 'demonstrated' and dec['s1']['yes'] and dec['s4']['yes'] and not dec['s3']['yes'],
          'campaign-2 decision is not demonstrated / s1 yes / s4 yes / s3 no')
    check(len(c2['trials']) == 88 and not c2['superseded_attempts'] and not c2['stops'],
          'campaign 2: not 88 trials without superseded attempts and stops')
    check(all(c['rtf_min'] >= 0.93 for c in c2['cells']) and 0.93 <= min(c['rtf_min'] for c in c2['cells']) < 0.94 and
          max(c['rtf_max'] for c in c2['cells']) < 0.975, 'campaign-2 RTF range 0.93-0.97 broken')
    check(c2['h3_control']['reached'] == 24 and c2['h3_control']['escape_enter_total'] == 0, 'H3 control 24/24, 0 ENTER broken')
    se = [c for c in c2['cells'] if c['config'] in ('C2', 'C4') and c['scenario'] != 's0_open']
    check(all(c['reached_with_contact'] == 0 and c['optimizer_failures_total'] == 0 and c['missed_control_cycles_total'] == 0
              and c['reached_without_escape'] == 0 and c['bt_spin'] + c['bt_backup'] + c['bt_wait'] == 0 for c in se),
          'SE cells: contact / optimizer failure / missed cycle / recovery / reached-without-escape claim broken')
    check(all(c['reached_with_contact'] == 0 for c in c2['cells']), 'a campaign-2 cell has contacts')
    check(all(t['enter_before_leaving_pocket'] for t in c2['trials']
              if t['config'] in ('C2', 'C4') and t['scenario'] in ('s1_utrap', 's3_ctrap', 's4_ltrap')),
          'an SE pocket trial has no ENTER before leaving the pocket')
    fs = [t for t in c2['trials'] if t['outcome'] == 'NOT_REACHED_ABORTED' and t.get('nav2_result') == 'SUCCEEDED']
    reg = [t for t in fs if t['phase'] != 'evidence' and not t['scenario'].endswith('evidence')]
    check(len(fs) == 13 and len(reg) == 11, f'Nav2-SUCCEEDED-but-not-reached trials: {len(fs)} total / {len(reg)} registered, paper says 13 / 11')
    fd = [t['final_goal_distance_m'] for t in fs]
    check(round(min(fd), 2) == 0.54 and round(max(fd), 2) == 1.66, 'false-success distance range 0.54-1.66 broken')
    by = {}
    for t in reg:
        by[(t['scenario'], t['config'])] = by.get((t['scenario'], t['config']), 0) + 1
    check(by == {('s1_utrap', 'C2'): 3, ('s1_utrap', 'C4'): 1, ('s3_ctrap', 'C2'): 4, ('s3_ctrap', 'C4'): 2, ('s4_ltrap', 'C4'): 1},
          f'false-success split changed: {by}')
    se_trials = [t for t in c2['trials'] if t['config'] in ('C2', 'C4') and t['scenario'] != 's0_open' and not t['scenario'].endswith('evidence')]
    check(all(t.get('nav2_result') == 'SUCCEEDED' for t in se_trials) and len(se_trials) == 30,
          'Nav2 SUCCEEDED in every SE pocket trial (6 of 6 per cell) broken')
    og = json.loads((EXP / 'campaign2/ODOM_GAP.json').read_text())
    check(og['n'] == 13, 'ODOM_GAP.json is not about 13 trials')
    ogv = lambda k: [r_[k] for r_ in og['rows']]
    otab = table_rows('tab:odomgap')
    exp = [['Ground-truth distance to goal', f"{min(ogv('final_gt_distance_m')):.3f}", f"{max(ogv('final_gt_distance_m')):.3f}"],
           ['Distance printed by Nav2 (AMCL)', f"{min(ogv('nav2_amcl_distance_m')):.2f}", f"{max(ogv('nav2_amcl_distance_m')):.2f}"],
           ['Shifted odometry distance to goal', f"{min(ogv('odom_distance_m')):.3f}", f"{max(ogv('odom_distance_m')):.3f}"],
           ['Odometry vs ground truth', f"{min(ogv('odom_minus_gt_m')):.3f}", f"{max(ogv('odom_minus_gt_m')):.3f}"],
           ['AMCL vs ground truth', f"{min(ogv('amcl_minus_gt_m')):.3f}", f"{max(ogv('amcl_minus_gt_m')):.3f}"]]
    check(otab == exp, f'tab:odomgap differs from ODOM_GAP.json: {otab} vs {exp}')
    check(round(min(ogv('amcl_minus_gt_m')), 2) == 0.33 and round(max(ogv('amcl_minus_gt_m')), 2) == 1.56 and
          round(min(ogv('odom_minus_gt_m')), 2) == 0.29 and round(max(ogv('odom_minus_gt_m')), 2) == 1.59,
          'prose gap ranges (0.33-1.56, 0.29-1.59) broken')
    check('$0.33$ to $1.56$~m' in tex and '$0.29$ to\n$1.59$~m' in tex, 'prose gap ranges missing from the paper')
    check(max(ogv('nav2_amcl_distance_m')) <= 0.2 and min(ogv('nav2_amcl_distance_m')) >= 0.0, 'Nav2 printed distance range 0.00-0.20 broken')
    tc = c2['teardown_crashes']
    check(tc['by_cell'] == {'s4_ltrap/C3': 1} and tc['rows'][0]['exit_code'] == -6, 'teardown crash record changed')
    tr = {c['name']: c for c in c2['contrasts']}
    t4 = tr['C4 on s4_ltrap vs C4 on s1_utrap (transfer)']['mann_whitney']
    check(round(t4['median1'], 1) == 48.3 and round(t4['median2'], 1) == 34.3 and round(t4['p'], 4) == 0.0079,
          'transfer time medians 48.3 / 34.3 / p 0.0079 broken')
    check('48.3' in tex and '34.3' in tex and '0.0079' in tex, 'transfer time quote missing')

    # ---- PREREG: branch wording verbatim, amendment facts ----
    prereg = (EXP / 'campaign2/PREREG.md').read_text()
    m = re.search(r'> - \*Demonstrated\*: (.*?)\n> - \*s1 yes', prereg, re.S)
    want = re.sub(r'\s+', ' ', re.sub(r'^>\s?', '', m.group(1), flags=re.M)).strip()
    q = re.search(r'\\begin\{quote\}\\small(.*?)\\end\{quote\}', tex, re.S).group(1)
    got = re.sub(r'\s+', ' ', q.replace('``', '"').replace("''", '"')).strip()
    check(got == want, 'branch wording in the paper differs from PREREG.md.\n   paper: ' + got + '\n   prereg: ' + want)
    check('FROZEN on 2026-10-18' in prereg and '88 trials' in prereg and 'Amendment 3, 2026-10-22' in prereg
          and 'Amendment 2, 2026-10-22' in prereg and 'Amendment 1, 2026-10-18' in prereg, 'PREREG amendment headings changed')
    for tok in ('2026-10-18', 'Amendment 3', 'post hoc', 'teardown crash', 'amendment 3', 'Provenance refusal'):
        check(tok in tex or tok.lower() in tex.lower(), f'paper lost the disclosure {tok!r}')
    check('0.0152' in tex and '0.0606' in tex and '0.1818' in tex and '0.4545' in tex, 'Fisher p quotes missing')
    # ablation table = PREREG sweep table
    sw = re.findall(r'^\| ([^|]+?) \| (on|off) \| (\d+) of 56 \|', prereg, re.M)
    from_prereg = sorted((a, int(n)) for _, a, n in sw)
    rows = table_rows('tab:ablation')
    from_tex = sorted((r_[1], int(r_[2])) for r_ in rows)
    check(from_prereg == from_tex and len(from_tex) == 10, f'ablation table differs from PREREG: {from_tex} vs {from_prereg}')
    check('167 of 168' in prereg and '48 of 168' in prereg, 'PREREG 167/48 of 168 changed')
    check('167 of 168' in flat and 'passed 48 of 168' in flat, 'paper lost 167/168 and 48/168')
    check('96' in tex and '56 valid' in flat, 'family count (96 combos, 56 valid) missing')
    dev = (ROOT / 'src/nav2_se_controller/test/deployment/UTRAP_DEV_GEOMETRIES.md').read_text()
    check('That is 4 x 3 x 4 x 2 = 96 combinations' in dev and 'finds 56 valid geometries' in dev, 'family rule counts changed')
    check('w22_d15_r-25_a6' in prereg and 'w22\\_d15\\_r-25\\_a6' in tex, 'twin of s3 not named')
    check('2.05 m' in prereg and '1.85 m' in prereg and '$2.05$~m' in tex and '$1.85$~m' in tex, 'depth margin 2.05/1.85 missing')

    # ---- parameter file ----
    yml = (ROOT / 'experiments/sim/nav2_se_campaign2.yaml').read_text()
    def yv(key):
        return float(re.search(r'^\s*' + key + r':\s*([-\d.]+)', yml, re.M).group(1))
    for key, val in (('se_progress_stall_window', 30), ('se_progress_min_advance', 0.10), ('se_progress_adopt_advance', 0.5),
                     ('se_escape_waypoint_cap', 2.5), ('se_escape_reselect_radius', 0.4), ('se_escape_reselect_period', 15),
                     ('se_escape_stall_time', 3), ('se_escape_stall_distance', 0.15), ('se_escape_hysteresis', 0.5),
                     ('se_escape_avoid_radius', 1.0), ('se_escape_avoid_hop_radius', 0.6), ('se_escape_avoid_hops', 3),
                     ('se_gap_num_rays', 36), ('se_gap_max_range', 2.0), ('se_gap_min_clearance', 0.6), ('gap_weight', 0.0),
                     ('vx_min', -0.35), ('time_steps', 28), ('model_dt', 0.1), ('batch_size', 1500), ('controller_frequency', 10.0)):
        check(yv(key) == val, f'campaign-2 parameter {key} is {yv(key)}, paper says {val}')
    for tok in ('0.10 m / 0.5 m', '36 / 2.0 m', '2.5 m', '0.4 m / 15 s', '0.5 rad', '3 s / 0.15 m', '1.0 m', '0.6 m; 3', '0.2 m; 6', '$-0.35$', '$1500$'):
        check(tok in tex, f'parameter quote {tok!r} missing from the paper')

    # ---- campaign 1 ----
    c1 = json.loads((EXP / 'results_pilot_rerun/results.json').read_text())
    h = c1['headline']
    check(h['n_correct'] == 44 and h['n_affected'] == 4 and h['n_registered_trials'] == 48 and h['correct']['reached'] == 0
          and h['affected']['reached'] == 0, 'campaign 1: 0 of 44 (+4 affected) broken')
    check(h['outcomes_correct'] == {'NOT_REACHED_ABORTED': 2, 'NOT_REACHED_TIMEOUT': 42} and h['correct']['trials_with_escape_enter'] == 13,
          'campaign 1: 42 timeouts, 2 aborted, 13 with escape broken')
    check(0.93 <= h['correct']['rtf_min'] < 0.94 and h['correct']['rtf_max'] < 0.97 + 1e-3 and h['affected']['rtf_min'] > 0.96,
          'campaign 1 RTF range 0.93-0.97 broken')
    builds = {t['mppi_build'] for t in c1['trials']}
    check(builds == {'ros2-nav2-mppi-controller-1.3.13-np2py312h405a742_25', 'ros-jazzy-nav2-mppi-controller-1.3.11-np2py312h2ed9cc7_16'},
          'campaign 1 builds changed')
    rc = (EXP / 'results_pilot_rerun/ROOT_CAUSE.md').read_text()
    for tok in ('70 of 70 closed escape episodes', '24 to 83 s', '0.34 m', '14 of the 20 valid `P`', '3.1 s'):
        check(tok in rc.replace('\n  ', ' ').replace('\n', ' '), f'ROOT_CAUSE.md lost {tok!r}')
    for tok in ('70 closed escape', '14 of them', '$0.93$', '42 timeouts', '13 of them'):
        check(tok in flat or tok.replace(' ', '~') in flat, f'paper lost campaign-1 quote {tok!r}')

    # ---- test counts (deployment) ----
    dep_dir = TESTS_DIR / 'deployment'
    dep = sum(len(re.findall(r'^TEST', p.read_text(), re.M)) for p in dep_dir.glob('test_*.cpp'))
    dep_files = len(list(dep_dir.glob('test_*.cpp')))
    check(dep == 148 and dep_files == 10, f'deployment tests: {dep} in {dep_files} files, paper says 148 in 10')
    check('148' in tex and '10 files' in tex, 'paper lost the deployment-test count')

    # ---- git facts (skipped outside a checkout that has the tags) ----
    def git(*a):
        r_ = subprocess.run(['git', *a], capture_output=True, text=True, cwd=ROOT)
        return r_.stdout.strip() if r_.returncode == 0 else None
    tag = git('rev-parse', 'campaign2-controller^{commit}')
    if tag:
        check(tag in tex, f'tag commit of campaign2-controller ({tag}) not quoted in the paper')
    v1 = git('rev-parse', 'paper-v1^{commit}')
    if v1:
        check(v1 in tex, f'tag commit of paper-v1 ({v1}) not quoted in the paper')

    # ---- README / CITATION agree with the paper ----
    readme = (ROOT / 'README.md').read_text()
    check('5 of 6' in readme and '0.0152' in readme and 'one simulated host' in readme and '80%' in readme,
          'README does not carry the live headline and the 80% mirror headline')
    check('no live run has yet reached its goal' not in readme.lower(), 'README still says no live run reached its goal')
    cff = (ROOT / 'CITATION.cff').read_text()
    check('SE-MPPI: Coordinating Online Local-Minima Escape with Control-Barrier-Function Safety in a Nav2-Native Controller' in cff,
          'CITATION.cff title differs from the paper title')
    outside = re.sub(r'\\begin\{quote\}.*?\\end\{quote\}', '', tex, flags=re.S)
    check('no live run has yet' not in outside.lower(), 'paper still says that no live run has reached its goal')

# --- forbidden tokens in the paper source ---
for tok in FORBIDDEN_IN_TEX:
    check(tok not in tex, f'forbidden token in main.tex: {tok!r}')

# --- forbidden tokens in the compiled PDF (best effort) ---
pdf = TEX.with_suffix('.pdf')
if pdf.exists():
    try:
        from pypdf import PdfReader
        text = ''.join(p.extract_text() for p in PdfReader(str(pdf)).pages)
        for tok in (FORBIDDEN_IN_TEX[0], FORBIDDEN_IN_TEX[2], FORBIDDEN_IN_TEX[5]):
            check(tok not in text, f'forbidden token in main.pdf: {tok!r}')
    except ImportError:
        r_ = subprocess.run(['pdftotext', str(pdf), '-'], capture_output=True, text=True)
        if r_.returncode == 0:
            for tok in (FORBIDDEN_IN_TEX[0], FORBIDDEN_IN_TEX[2], FORBIDDEN_IN_TEX[5]):
                check(tok not in r_.stdout, f'forbidden token in main.pdf: {tok!r}')
        else:
            print('note: neither pypdf nor pdftotext available — PDF token sweep skipped', file=sys.stderr)

if fails:
    print('NUMBER GUARD: FAIL')
    for f in fails:
        print('  -', f)
    sys.exit(1)
print(f'NUMBER GUARD: OK ({"arXiv package, " if ARXIV else ""}{n_trials} trials, {all_tests} tests/{n_files} files, all headline figures traced)')
