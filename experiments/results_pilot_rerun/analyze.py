#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Regenerate every number of the live re-run from its raw files (PREREG.md).

    python3 experiments/results_pilot_rerun/analyze.py            # writes results.json, RESULTS.md
    python3 experiments/results_pilot_rerun/analyze.py --check    # fails if they are stale
    python3 experiments/results_pilot_rerun/analyze.py --check --digits 9
                                                 # same, floats compared to 9 significant digits

The committed files were written by Python 3.8. Python 3.12 sums floats with compensated
summation, so a fresh analysis there differs from the committed files in the last digit of some
floats (for example 3.669061014814539 and 3.6690610148145404). ``--check`` alone is exact and
passes only on the interpreter that wrote the files; ``--digits N`` rounds every float literal
in both texts to N significant digits first and is what CI uses.

Standard library only, so the paper-number guard can run it anywhere. Inputs:
``trials/<id>/`` and ``trials_shakedown/<id>/`` (``meta.json``, ``params.yaml``,
``drive.log``, ``launch.log[.gz]``, ``telemetry.csv[.gz]``,
``diagnostics.jsonl``), the world ``experiments/se_mppi_utrap/worlds/testbed.sdf``
and the robot ``experiments/se_mppi_utrap/urdf/amr.urdf.xacro`` for the contact
geometry, and the July pilot's ``experiments/results_pilot/drive_*.log`` for the
same first-seconds statistic. The definitions are those fixed in PREREG.md.
"""

from __future__ import annotations

import argparse
import csv
import glob
import gzip
import io
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hostload  # noqa: E402
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
WORLD = os.path.join(REPO, 'experiments', 'se_mppi_utrap', 'worlds', 'testbed.sdf')
URDF = os.path.join(REPO, 'experiments', 'se_mppi_utrap', 'urdf', 'amr.urdf.xacro')
PILOT = os.path.join(REPO, 'experiments', 'results_pilot')

GOAL_TOL = 0.5          # m, experiments/configs/suite.yaml goal_tol
CONTACT_TOL = 0.01      # m
START_TOL = 0.05        # m, ground-truth sanity check
FIRST_WINDOW_S = 5.0
VX_MIN_BAND = 0.01      # m/s
RTF_WINDOW_S = 10.0

_STAMP = r'\[(?P<stamp>\d+\.\d+)\]'
SEND_RE = re.compile(r'\[INFO\] ' + _STAMP + r' \[basic_navigator\]: Nav2 active\. Sending goal')
TELEM_RE = re.compile(
    r'\[INFO\] ' + _STAMP + r' \[basic_navigator\]: \[\s*(?P<t>[\d.]+)s\] dist=(?P<dist>[-\d.]+)m'
    r' amcl=\((?P<ax>[-\d.]+),(?P<ay>[-\d.]+) yaw=(?P<ayaw>[-\d.]+)rad\)'
    r' odom=\((?P<ox>[-\d.na]+),(?P<oy>[-\d.na]+) yaw=(?P<oyaw>[-\d.na]+)rad\)'
    r' nav=vx:(?P<nvx>[-\d.na]+) wz:(?P<nwz>[-\d.na]+)'
    r' cmd=vx:(?P<cvx>[-\d.na]+) wz:(?P<cwz>[-\d.na]+)')
RESULT_RE = re.compile(r'SMOKE_RESULT=TaskResult\.(\w+)')
OPTFAIL_RE = re.compile(_STAMP + r' \[controller_server\]: Optimizer fail to compute path')
ENTER_RE = re.compile(_STAMP + r' \[MPPIController\]: SE escape ENTER at \((?P<x>[-\d.]+), (?P<y>[-\d.]+)\)')


def read_text(path: str) -> str:
    if os.path.exists(path):
        with open(path, errors='replace') as f:
            return f.read()
    if os.path.exists(path + '.gz'):
        with gzip.open(path + '.gz', 'rt', errors='replace') as f:
            return f.read()
    return ''


def fnum(tok):
    try:
        v = float(tok)
    except (TypeError, ValueError):
        return math.nan
    return v


# --------------------------------------------------------------------------- #
# Geometry: chassis rectangle against the wall boxes of the world.
# --------------------------------------------------------------------------- #
def world_boxes() -> list:
    """Corner lists of every static box of the world except the ground plane."""
    boxes = []
    for model in ET.parse(WORLD).getroot().iter('model'):
        size = model.find('./link/collision/geometry/box/size')
        pose = model.find('pose')
        if size is None or pose is None:
            continue
        sx, sy, _ = (float(v) for v in size.text.split())
        x, y, _, _, _, yaw = (float(v) for v in pose.text.split())
        boxes.append((model.get('name'), rect(x, y, yaw, sx, sy)))
    return boxes


def chassis_size() -> tuple:
    """(length, width) of the base_link collision box."""
    text = open(URDF).read()
    m = re.search(r'<link name="base_link">.*?<collision>\s*<geometry><box size="([\d. ]+)"/>',
                  text, re.S)
    lx, ly, _ = (float(v) for v in m.group(1).split())
    return lx, ly


def rect(x, y, yaw, sx, sy) -> list:
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2))]


def _seg_dist(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _overlap(pa, pb) -> bool:
    """Separating-axis test for two convex polygons."""
    for poly in (pa, pb):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            nx, ny = y2 - y1, x1 - x2
            a = [nx * x + ny * y for x, y in pa]
            b = [nx * x + ny * y for x, y in pb]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def rect_distance(pa, pb) -> float:
    """Distance between two convex polygons; 0 when they overlap."""
    if _overlap(pa, pb):
        return 0.0
    best = math.inf
    for p, q in ((pa, pb), (pb, pa)):
        for pt in p:
            for i in range(len(q)):
                best = min(best, _seg_dist(pt, q[i], q[(i + 1) % len(q)]))
    return best


# --------------------------------------------------------------------------- #
def read_telemetry(tdir: str) -> list:
    text = read_text(os.path.join(tdir, 'telemetry.csv'))
    rows = []
    for r in csv.DictReader(io.StringIO(text)):
        rows.append({k: fnum(v) for k, v in r.items()})
    return rows


def drive_stats(drive_text: str, vx_min: float) -> dict:
    """Goal-sent time, telemetry lines and first-seconds statistic of a drive log."""
    send = SEND_RE.search(drive_text)
    lines = [m.groupdict() for m in TELEM_RE.finditer(drive_text)]
    out = {
        't0': float(send.group('stamp')) if send else None,
        't1': float(lines[-1]['stamp']) if lines else None,
        'n_telemetry_lines': len(lines),
        'nav2_result': (RESULT_RE.search(drive_text).group(1)
                        if RESULT_RE.search(drive_text) else None),
        'watchdog_fired': 'TIMEOUT after' in drive_text,
    }
    first = [fnum(m['nvx']) for m in lines if float(m['t']) <= FIRST_WINDOW_S]
    first = [v for v in first if not math.isnan(v)]
    if first:
        out['first5s'] = {
            'n': len(first), 'vx_mean': sum(first) / len(first),
            'vx_min_seen': min(first), 'vx_max_seen': max(first), 'vx_min_param': vx_min,
            'share_at_vx_min': sum(abs(v - vx_min) <= VX_MIN_BAND for v in first) / len(first),
        }
    else:
        out['first5s'] = None
    return out


def vx_min_of(params_text: str) -> float:
    m = re.search(r'^\s*vx_min:\s*([-\d.]+)\s*$', params_text, re.M)
    return float(m.group(1))


def analyze_trial(tdir: str, boxes: list, chassis: tuple) -> dict:
    with open(os.path.join(tdir, 'meta.json')) as f:
        meta = json.load(f)
    rec = {
        'trial_id': os.path.basename(tdir),
        'env_label': meta['env_label'], 'scenario': meta['scenario'],
        'config': meta['config'], 'param_set': meta['param_set'], 'rep': meta['rep'],
        'override_guard': meta['override_guard'],
        'mppi_build': next((b for b in meta['env']['package_builds']
                            if 'nav2-mppi-controller' in b and '-np2py' in b), None),
        'check_mppi_rollout_exit': meta['env']['check_mppi_rollout_exit'],
        'repo_commit': meta['env']['repo_commit'],
        'setup_attempts': meta.get('setup_attempts'),
        'load1_at_start': meta.get('load1_at_start'),
        'load_gate_waited_s': meta.get('load_gate_waited_s'),
    }
    if not meta.get('complete'):
        rec['outcome'] = 'INVALID'
        rec['invalid_reason'] = 'driver did not finish the trial'
        return rec
    if not meta.get('active'):
        rec['outcome'] = 'SETUP_FAIL'
        return rec

    launch_name = meta.get('launch_log', 'launch.log')
    launch = read_text(os.path.join(tdir, launch_name))
    params = read_text(os.path.join(tdir, 'params.yaml'))
    drive = drive_stats(read_text(os.path.join(tdir, 'drive.log')), vx_min_of(params))
    rec.update({k: drive[k] for k in ('nav2_result', 'watchdog_fired', 'n_telemetry_lines',
                                       'first5s')})
    rec['missed_control_cycles'] = launch.count('Control loop missed its desired rate')
    # Descriptive fields added after the first registered trials (PREREG.md,
    # amendment 7); no outcome definition uses them.
    fails = [float(m.group('stamp')) for m in OPTFAIL_RE.finditer(launch)]
    rec['optimizer_failures'] = len(fails)
    rec['first_optimizer_failure_s'] = (fails[0] - drive['t0']
                                        if fails and drive['t0'] else None)
    rec['progress_failures'] = launch.count('Failed to make progress')
    enters = [m.groupdict() for m in ENTER_RE.finditer(launch)]
    rec['escape_enter'] = len(enters)
    rec['escape_injection_begin'] = launch.count('escape-cost injection BEGIN')
    rec['first_escape_enter_s'] = (float(enters[0]['stamp']) - drive['t0']
                                   if enters and drive['t0'] else None)
    rec['first_escape_enter_odom_xy'] = ([float(enters[0]['x']), float(enters[0]['y'])]
                                         if enters else None)
    diag = [json.loads(ln) for ln in
            read_text(os.path.join(tdir, 'diagnostics.jsonl')).splitlines() if ln.strip()]
    rec['guard_state'] = diag[-1]['values'].get('mppi_rollout_check') if diag else None
    rec['diagnostics_last'] = ({k: diag[-1]['values'].get(k) for k in
                                ('cycles', 'escape_entries', 'forced_stops', 'qp_failures',
                                 'slack_active')} if diag else None)

    if drive['t0'] is None or drive['t1'] is None:
        rec['outcome'] = 'NO_GOAL_SENT'
        return rec
    t0, t1 = drive['t0'], drive['t1']

    rows = read_telemetry(tdir)
    gt_rows = [r for r in rows if not math.isnan(r['gt_x'])]
    start, goal = meta['start'], meta['goal']
    if not gt_rows:
        rec['outcome'] = 'INVALID'
        rec['invalid_reason'] = 'no ground truth in telemetry.csv'
        return rec
    d_start = math.hypot(gt_rows[0]['gt_x'] - start[0], gt_rows[0]['gt_y'] - start[1])
    if d_start > START_TOL:
        rec['outcome'] = 'INVALID'
        rec['invalid_reason'] = f'first ground-truth sample {d_start:.3f} m from the start'
        return rec

    drive_rows = [r for r in gt_rows if t0 <= r['wall_s'] <= t1 + 0.2]
    if not drive_rows:
        rec['outcome'] = 'INVALID'
        rec['invalid_reason'] = 'no ground-truth sample inside the drive interval'
        return rec
    last = drive_rows[-1]
    rec['final_gt_xy'] = [round(last['gt_x'], 3), round(last['gt_y'], 3)]
    rec['final_goal_distance_m'] = math.hypot(last['gt_x'] - goal[0], last['gt_y'] - goal[1])
    rec['max_gt_x'] = max(r['gt_x'] for r in drive_rows)
    # Last instant at which the robot was still moving (> 2 cm over 5 s).
    moving_until = drive_rows[0]['wall_s']
    k = 0
    for r in drive_rows:
        while r['wall_s'] - drive_rows[k]['wall_s'] > 5.0:
            k += 1
        if math.hypot(r['gt_x'] - drive_rows[k]['gt_x'],
                      r['gt_y'] - drive_rows[k]['gt_y']) > 0.02:
            moving_until = r['wall_s']
    rec['stationary_from_s'] = moving_until - t0
    rec['stationary_share'] = max(0.0, (t1 - moving_until) / (t1 - t0))
    rec['reached'] = rec['final_goal_distance_m'] <= GOAL_TOL

    lx, ly = chassis
    min_d, where = math.inf, None
    for r in drive_rows:
        body = rect(r['gt_x'], r['gt_y'], r['gt_yaw'], lx, ly)
        for name, box in boxes:
            d = rect_distance(body, box)
            if d < min_d:
                min_d, where = d, name
    rec['min_wall_distance_m'] = min_d
    rec['nearest_wall'] = where
    rec['contact'] = min_d < CONTACT_TOL
    rec['max_abs_pitch_deg'] = max(abs(math.degrees(r['gt_pitch'])) for r in drive_rows)
    rec['max_abs_roll_deg'] = max(abs(math.degrees(r['gt_roll'])) for r in drive_rows)

    # Path lengths: ground truth against odometry (wheel slip shows as a gap).
    def plen(key_x, key_y, rs):
        return sum(math.hypot(b[key_x] - a[key_x], b[key_y] - a[key_y])
                   for a, b in zip(rs, rs[1:]))
    rec['gt_path_m'] = plen('gt_x', 'gt_y', drive_rows)
    odom_rows = [r for r in drive_rows if not math.isnan(r['odom_x'])]
    rec['odom_path_m'] = plen('odom_x', 'odom_y', odom_rows) if odom_rows else None

    sim_rows = [r for r in drive_rows if not math.isnan(r['sim_s'])]
    wall = sim_rows[-1]['wall_s'] - sim_rows[0]['wall_s']
    sim = sim_rows[-1]['sim_s'] - sim_rows[0]['sim_s']
    rec['drive_wall_s'] = t1 - t0
    rec['drive_sim_s'] = sim
    rec['rtf'] = sim / wall if wall > 0 else None
    windows = []
    i = 0
    while i < len(sim_rows):
        j = i
        while j < len(sim_rows) and sim_rows[j]['wall_s'] - sim_rows[i]['wall_s'] < RTF_WINDOW_S:
            j += 1
        if j < len(sim_rows):
            windows.append((sim_rows[j]['sim_s'] - sim_rows[i]['sim_s'])
                           / (sim_rows[j]['wall_s'] - sim_rows[i]['wall_s']))
        i = j
    rec['rtf_min_10s'] = min(windows) if windows else None
    rec['load1_mean'] = sum(r['load1'] for r in drive_rows) / len(drive_rows)

    if rec['reached']:
        rec['outcome'] = 'REACHED_WITH_CONTACT' if rec['contact'] else 'REACHED'
    elif rec['watchdog_fired']:
        rec['outcome'] = 'NOT_REACHED_TIMEOUT'
    else:
        rec['outcome'] = 'NOT_REACHED_ABORTED'
    rec['nav2_false_success'] = rec['nav2_result'] == 'SUCCEEDED' and not rec['reached']

    # Host-load validity (PREREG.md, amendment 5), recomputed from the samples.
    load = hostload.verdict(hostload.read_rows(os.path.join(tdir, 'host_cpu.csv')),
                            meta['drive_wall_start'], meta['drive_wall_end'])
    rec['host_load'] = load
    if not load['valid']:
        rec['outcome_measured'] = rec['outcome']
        rec['outcome'] = 'INVALID_HOST_LOAD'
        rec['invalid_reason'] = load['reason']
    return rec


def pilot_first_seconds() -> dict:
    """The same first-seconds statistic for the July pilot's drive logs
    (vx_min 0.1 in experiments/sim/nav2_se_loopback.yaml)."""
    out = {}
    for name in ('s1_utrap', 's1_utrap_evidence'):
        with open(os.path.join(PILOT, f'drive_{name}.log'), errors='replace') as f:
            d = drive_stats(f.read(), 0.1)
        out[name] = {'first5s': d['first5s'], 'nav2_result': d['nav2_result'],
                     'watchdog_fired': d['watchdog_fired']}
    return out


def fisher_two_sided(a, b, c, d) -> float:
    """Two-sided Fisher exact p of the table [[a, b], [c, d]]."""
    def comb(n, k):
        return math.comb(n, k)
    r1, r2, c1, n = a + b, c + d, a + c, a + b + c + d
    def p(x):
        return comb(r1, x) * comb(r2, c1 - x) / comb(n, c1)
    p_obs = p(a)
    return min(1.0, sum(p(x) for x in range(max(0, c1 - r2), min(r1, c1) + 1)
                        if p(x) <= p_obs * (1 + 1e-9)))


def summarize(trials: list) -> list:
    cells = {}
    for t in trials:
        key = (t['env_label'], t['param_set'], t['scenario'], t['config'])
        cells.setdefault(key, []).append(t)
    out = []
    for key in sorted(cells):
        ts = cells[key]
        outcomes = {}
        for t in ts:
            outcomes[t['outcome']] = outcomes.get(t['outcome'], 0) + 1
        driven = [t for t in ts if 'reached' in t and t['outcome'] != 'INVALID_HOST_LOAD']

        def med(vals):
            vals = sorted(v for v in vals if v is not None)
            if not vals:
                return None
            m = len(vals) // 2
            return vals[m] if len(vals) % 2 else 0.5 * (vals[m - 1] + vals[m])
        first = [t['first5s']['vx_mean'] for t in driven if t.get('first5s')]
        out.append({
            'env_label': key[0], 'param_set': key[1], 'scenario': key[2], 'config': key[3],
            'n': len(ts), 'outcomes': outcomes,
            'reached': sum(t['reached'] for t in driven),
            'contact': sum(t['contact'] for t in driven),
            'nav2_succeeded': sum(t.get('nav2_result') == 'SUCCEEDED' for t in ts),
            'nav2_false_success': sum(t.get('nav2_false_success', False) for t in ts),
            'trials_with_escape_enter': sum(t.get('escape_enter', 0) > 0 for t in ts),
            'missed_control_cycles_total': sum(t.get('missed_control_cycles', 0) for t in ts),
            'trials_with_missed_cycles': sum(t.get('missed_control_cycles', 0) > 0 for t in ts),
            'rtf_min': min((t['rtf'] for t in driven), default=None),
            'rtf_median': med(t['rtf'] for t in driven),
            'rtf_max': max((t['rtf'] for t in driven), default=None),
            'first5s_vx_mean_min': min(first, default=None),
            'first5s_vx_mean_max': max(first, default=None),
            'load1_mean_max': max((t['load1_mean'] for t in driven), default=None),
            'foreign_cores_mean_max': max((t['host_load']['foreign_cores_mean']
                                           for t in driven), default=None),
        })
    return out


def build() -> dict:
    boxes, chassis = world_boxes(), chassis_size()
    def load(sub):
        return [analyze_trial(d, boxes, chassis)
                for d in sorted(glob.glob(os.path.join(HERE, sub, '*')))
                if os.path.exists(os.path.join(d, 'meta.json'))]
    everything = load('trials')
    trials = [t for t in everything if '.' not in t['trial_id']]
    result = {
        'definitions': {'goal_tol_m': GOAL_TOL, 'contact_tol_m': CONTACT_TOL,
                        'first_window_s': FIRST_WINDOW_S, 'vx_min_band': VX_MIN_BAND,
                        'rtf_window_s': RTF_WINDOW_S, 'chassis_m': list(chassis),
                        'walls': [n for n, _ in boxes]},
        'trials': trials,
        'cells': summarize(trials),
        'superseded_attempts': [t for t in everything if '.' in t['trial_id']],
        'exploratory_loaded': load('trials_exploratory_loaded'),
        'calibration': load('trials_calibration'),
        'shakedown': load('trials_shakedown'),
        'host_load_rule': {'admit_load1_below': hostload.ADMIT_LOAD1,
                           'admit_spacing_s': hostload.ADMIT_SPACING_S,
                           'valid_mean_foreign_cores': hostload.VALID_MEAN_CORES,
                           'valid_window_foreign_cores': hostload.VALID_WINDOW_CORES,
                           'window_s': hostload.WINDOW_S},
        'pilot_2026_07_03': pilot_first_seconds(),
    }
    result['headline'] = headline(result)
    return result


def headline(result: dict) -> dict:
    """The aggregate numbers the paper quotes (guarded by check_paper_numbers.py)."""
    ts = result['trials']
    def sel(**kw):
        return [t for t in ts if all(t.get(k) == v for k, v in kw.items())]
    def driven(xs):
        return [t for t in xs if 'reached' in t and t['outcome'] != 'INVALID_HOST_LOAD']
    correct, affected = sel(env_label='correct'), sel(env_label='affected')
    h = {
        'n_registered_trials': len(ts),
        'n_correct': len(correct), 'n_affected': len(affected),
        'outcomes_correct': {}, 'outcomes_affected': {},
    }
    for name, group in (('outcomes_correct', correct), ('outcomes_affected', affected)):
        for t in group:
            h[name][t['outcome']] = h[name].get(t['outcome'], 0) + 1
    for label, group in (('correct', correct), ('affected', affected)):
        d = driven(group)
        if not d:
            continue
        rtfs = sorted(t['rtf'] for t in d)
        first = [t['first5s'] for t in d if t.get('first5s')]
        h[label] = {
            'n_driven': len(d),
            'reached': sum(t['reached'] for t in d),
            'contact': sum(t['contact'] for t in d),
            'nav2_succeeded': sum(t['nav2_result'] == 'SUCCEEDED' for t in d),
            'nav2_false_success': sum(t['nav2_false_success'] for t in d),
            'rtf_min': rtfs[0], 'rtf_max': rtfs[-1],
            'rtf_median': (rtfs[len(rtfs) // 2] if len(rtfs) % 2
                           else 0.5 * (rtfs[len(rtfs) // 2 - 1] + rtfs[len(rtfs) // 2])),
            'missed_control_cycles_total': sum(t['missed_control_cycles'] for t in d),
            'trials_with_missed_cycles': sum(t['missed_control_cycles'] > 0 for t in d),
            'trials_with_optimizer_failures': sum(t['optimizer_failures'] > 0 for t in d),
            'not_reached': sum(not t['reached'] for t in d),
            'not_reached_with_optimizer_failures': sum(
                (not t['reached']) and t['optimizer_failures'] > 0 for t in d),
            'not_reached_without_escape_enter': sum(
                (not t['reached']) and t['escape_enter'] == 0 for t in d),
            'trials_with_escape_enter': sum(t['escape_enter'] > 0 for t in d),
            'first5s_vx_mean_min': min(f['vx_mean'] for f in first),
            'first5s_vx_mean_max': max(f['vx_mean'] for f in first),
            'load1_mean_min': min(t['load1_mean'] for t in d),
            'load1_mean_max': max(t['load1_mean'] for t in d),
            'load1_at_start_max': max(t['load1_at_start'] for t in d),
            'foreign_cores_mean_max': max(t['host_load']['foreign_cores_mean'] for t in d),
            'foreign_cores_worst_window_max': max(t['host_load']['foreign_cores_worst_window']
                                                  for t in d),
            'own_cores_mean_min': min(t['host_load']['own_cores_mean'] for t in d),
            'own_cores_mean_max': max(t['host_load']['own_cores_mean'] for t in d),
        }
    return h


def fmt(v, nd=2):
    if v is None:
        return '--'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def render(result: dict) -> str:
    L = ['# Live U-trap re-run: results', '',
         'Generated by `analyze.py` from the raw files in `trials/` and `trials_shakedown/`.',
         'Definitions and design: `PREREG.md`. Do not edit by hand.', '']
    L += ['## Per cell', '',
          '| Env | Set | Scenario | Config | n | Reached (ground truth) | Contact | Nav2 SUCCEEDED | of which not at the goal | Trials with escape ENTER | Missed control cycles | RTF min / median / max | First 5 s mean vx (min..max over trials) | Outcomes |',
          '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for c in result['cells']:
        L.append('| ' + ' | '.join([
            c['env_label'], c['param_set'], c['scenario'], c['config'], str(c['n']),
            str(c['reached']), str(c['contact']), str(c['nav2_succeeded']),
            str(c['nav2_false_success']), str(c['trials_with_escape_enter']),
            str(c['missed_control_cycles_total']),
            f"{fmt(c['rtf_min'])} / {fmt(c['rtf_median'])} / {fmt(c['rtf_max'])}",
            f"{fmt(c['first5s_vx_mean_min'], 3)}..{fmt(c['first5s_vx_mean_max'], 3)}",
            ', '.join(f'{k} {v}' for k, v in sorted(c['outcomes'].items()))]) + ' |')

    def table(title, trials):
        out = ['', f'## {title}', '',
               '| Trial | Outcome | Nav2 result | Final distance to goal (m) | Final ground-truth x, y | Min wall distance (m) | Max pitch (deg) | Drive wall / sim (s) | RTF (min 10 s) | Missed cycles | Optimizer failures (first, s); stationary from (s) | First 5 s vx mean [min, max], share at vx_min | Escape ENTER / injection BEGIN (first ENTER, s) | Odometry / ground-truth path (m) | Guard | Load1 at start / mean; foreign CPU cores mean / worst 30 s; own cores |',
               '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
        for t in trials:
            f = t.get('first5s')
            first = (f"{f['vx_mean']:.3f} [{f['vx_min_seen']:.3f}, {f['vx_max_seen']:.3f}], "
                     f"{100 * f['share_at_vx_min']:.0f}% (vx_min {f['vx_min_param']})"
                     if f else '--')
            xy = t.get('final_gt_xy')
            out.append('| ' + ' | '.join([
                t['trial_id'],
                t['outcome'] + (f" ({t['invalid_reason']})" if t.get('invalid_reason') else '')
                + (f"; measured: {t['outcome_measured']}" if t.get('outcome_measured') else ''),
                fmt(t.get('nav2_result')), fmt(t.get('final_goal_distance_m')),
                f'{xy[0]:.2f}, {xy[1]:.2f}' if xy else '--',
                fmt(t.get('min_wall_distance_m'), 3), fmt(t.get('max_abs_pitch_deg'), 1),
                f"{fmt(t.get('drive_wall_s'), 1)} / {fmt(t.get('drive_sim_s'), 1)}",
                f"{fmt(t.get('rtf'))} ({fmt(t.get('rtf_min_10s'))})",
                fmt(t.get('missed_control_cycles')),
                f"{fmt(t.get('optimizer_failures'))} ({fmt(t.get('first_optimizer_failure_s'), 1)}); "
                f"{fmt(t.get('stationary_from_s'), 1)}", first,
                f"{fmt(t.get('escape_enter'))} / {fmt(t.get('escape_injection_begin'))} "
                f"({fmt(t.get('first_escape_enter_s'), 1)})",
                f"{fmt(t.get('odom_path_m'))} / {fmt(t.get('gt_path_m'))}",
                fmt(t.get('guard_state')),
                f"{fmt(t.get('load1_at_start'), 1)} / {fmt(t.get('load1_mean'), 1)}; "
                f"{fmt((t.get('host_load') or {}).get('foreign_cores_mean'))} / "
                f"{fmt((t.get('host_load') or {}).get('foreign_cores_worst_window'))} / "
                f"{fmt((t.get('host_load') or {}).get('own_cores_mean'))}"]) + ' |')
        return out
    L += table('Registered trials', result['trials'])
    L += table('Attempts invalidated by host load or interrupted, and repeated '
               '(not counted above)', result['superseded_attempts'])
    L += table('Exploratory runs under uncontrolled host load (first start of the main '
               'block; not used for any statement)', result['exploratory_loaded'])
    L += table('Idle-host calibration run (not a registered trial)', result['calibration'])
    L += table('Shakedown runs (not registered trials)', result['shakedown'])
    L += ['', '## July 2026 pilot, same first-seconds statistic', '',
          '| Scenario | First 5 s vx mean [min, max] | Share at vx_min (0.1) | Nav2 result | Watchdog |',
          '|---|---|---|---|---|']
    for name, p in result['pilot_2026_07_03'].items():
        f = p['first5s']
        L.append(f"| {name} | {f['vx_mean']:.3f} [{f['vx_min_seen']:.3f}, {f['vx_max_seen']:.3f}] "
                 f"| {100 * f['share_at_vx_min']:.0f}% | {p['nav2_result']} "
                 f"| {fmt(p['watchdog_fired'])} |")
    L += ['', '## Headline numbers', '', '```json',
          json.dumps(result['headline'], indent=1, sort_keys=True), '```', '']
    return '\n'.join(L)


_FLOAT = re.compile(r'-?\d+\.\d+(?:[eE][+-]?\d+)?')


def _round_floats(text: str, digits: int) -> str:
    """Every float literal in ``text`` rounded to ``digits`` significant digits."""
    return _FLOAT.sub(lambda m: f'{float(m.group()):.{digits}g}', text)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='fail if results.json or RESULTS.md differ from a fresh analysis')
    ap.add_argument('--digits', type=int, default=None,
                    help='with --check: compare floats rounded to this many significant digits '
                         '(for an interpreter other than the one that wrote the files)')
    args = ap.parse_args()
    if args.digits is not None and not (args.check and 1 <= args.digits <= 15):
        ap.error('--digits goes with --check and takes 1 to 15')
    result = build()
    js = json.dumps(result, indent=1, sort_keys=True) + '\n'
    md = render(result)
    targets = ((os.path.join(HERE, 'results.json'), js), (os.path.join(HERE, 'RESULTS.md'), md))
    if args.check:
        def same(a, b):
            if args.digits is None:
                return a == b
            return _round_floats(a, args.digits) == _round_floats(b, args.digits)

        stale = [p for p, text in targets
                 if not os.path.exists(p) or not same(open(p).read(), text)]
        if stale:
            print('stale: ' + ', '.join(os.path.relpath(p, REPO) for p in stale))
            return 1
        print(f'results.json and RESULTS.md match a fresh analysis '
              f'({len(result["trials"])} trials)')
        return 0
    for p, text in targets:
        with open(p, 'w') as f:
            f.write(text)
    print(f'wrote results.json and RESULTS.md ({len(result["trials"])} trials, '
          f'{len(result["shakedown"])} shakedown runs)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
