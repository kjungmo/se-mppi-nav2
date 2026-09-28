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
        print('note: pypdf unavailable — PDF token sweep skipped', file=sys.stderr)

if fails:
    print('NUMBER GUARD: FAIL')
    for f in fails:
        print('  -', f)
    sys.exit(1)
print(f'NUMBER GUARD: OK ({"arXiv package, " if ARXIV else ""}{n_trials} trials, {all_tests} tests/{n_files} files, all headline figures traced)')
