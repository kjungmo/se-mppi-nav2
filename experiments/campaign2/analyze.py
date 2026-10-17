#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Regenerate every number of campaign 2 from its raw files (PREREG.md).

    python3 experiments/campaign2/analyze.py            # writes results.json.gz, RESULTS.md
    python3 experiments/campaign2/analyze.py --check    # fails if they are stale
    python3 experiments/campaign2/analyze.py --pack     # also writes logs/<trial>.log.gz excerpts

Raw files stay out of the repository, under ``/mnt/d/wsl/se-mppi/campaign2/``
(``--raw``): ``trials/<id>/`` (registered trials), ``trials/<id>.invalidN`` and
``.abortedN`` (attempts that were repeated), ``shakedown/<id>/`` (not counted).
Each trial directory holds ``meta.json``, ``params.yaml``, ``drive.log``,
``launch.log[.gz]``, ``telemetry.csv[.gz]``, ``diagnostics.jsonl``,
``escape_path.jsonl`` and ``host_cpu.csv``. Only the analysis result
(``results.json.gz``, ``RESULTS.md``) and small log excerpts are committed.

Definitions are those of PREREG.md. The outcome definitions, the contact and goal
tolerances, the world geometry check and the host-load verdict are the ones of
campaign 1 (``experiments/results_pilot_rerun``): this file imports them from
there. Standard library only (plus the campaign-1 modules, also standard library).
"""

from __future__ import annotations

import argparse
import glob
import gzip
import hashlib
import importlib.util
import io
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
PILOT_DIR = os.path.join(REPO, 'experiments', 'results_pilot_rerun')
sys.path.insert(0, HERE)
sys.path.insert(0, PILOT_DIR)
import hostload  # noqa: E402  (campaign 1, unchanged)
import plan as plan_mod  # noqa: E402


def _load_pilot():
    spec = importlib.util.spec_from_file_location('pilot_analyze',
                                                  os.path.join(PILOT_DIR, 'analyze.py'))
    mod = importlib.util.module_from_spec(spec)
    sys.modules['pilot_analyze'] = mod
    spec.loader.exec_module(mod)
    return mod


pilot = _load_pilot()

GOAL_TOL = pilot.GOAL_TOL
CONTACT_TOL = pilot.CONTACT_TOL
START_TOL = pilot.START_TOL
RTF_WINDOW_S = pilot.RTF_WINDOW_S
PROBE_FRAME = 'shakedown_probe'
URDF = pilot.URDF
BASE_WORLD = os.path.join(REPO, 'experiments', 'se_mppi_utrap', 'worlds', 'testbed.sdf')
WORLDS = {
    's1_utrap': BASE_WORLD,
    's1_utrap_evidence': BASE_WORLD,
    's3_ctrap': os.path.join(REPO, 'experiments', 'campaign2', 'worlds', 's3_ctrap.sdf'),
    's0_open': os.path.join(REPO, 'experiments', 'campaign2', 'worlds', 's0_open.sdf'),
}

# Pocket polygons (map frame = world frame), counter-clockwise. A trial "leaves the
# pocket" at the first ground-truth sample outside the polygon after the first sample
# inside it. s1: the U of the testbed, x from the open end of the side walls (-1.0) to
# the back wall face (0.85), |y| up to the inner faces of the side walls (0.85); the
# outside test is DESIGN_ADVICE 4.6, "x < -1.0 or |y| > 0.85". s3: the same construction
# in the pocket frame (open toward -x, back wall face at x 1.15, side-wall inner faces
# at |y| 1.05, mouth at the end of the longer side wall, x -0.3), rotated +25 degrees
# about (0.5, 0).
S3_PIVOT = (0.5, 0.0)
S3_ROT_DEG = 25.0
S3_POCKET_FRAME = [(-0.3, -1.05), (1.15, -1.05), (1.15, 1.05), (-0.3, 1.05)]


def _rotate(pt, pivot, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    dx, dy = pt[0] - pivot[0], pt[1] - pivot[1]
    return (pivot[0] + c * dx - s * dy, pivot[1] + s * dx + c * dy)


POCKETS = {
    's1_utrap': [(-1.0, -0.85), (0.85, -0.85), (0.85, 0.85), (-1.0, 0.85)],
    's3_ctrap': [_rotate(p, S3_PIVOT, S3_ROT_DEG) for p in S3_POCKET_FRAME],
}
POCKETS['s1_utrap_evidence'] = POCKETS['s1_utrap']


def inside_polygon(pt, poly) -> bool:
    """Point in a convex polygon (counter-clockwise), boundary counts as inside."""
    x, y = pt
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1) < -1e-12:
            return False
    return True


# --------------------------------------------------------------------------- #
# Log patterns. The format strings are those of safe_escape_controller.cpp and
# escape_critic.cpp; tests/test_analyze.py checks them against that source.
# --------------------------------------------------------------------------- #
_STAMP = r'\[(?P<stamp>\d+\.\d+)\]'
_XY = r'\((?P<{0}>[-\d.]+), (?P<{1}>[-\d.]+)\)'
ENTER_RE = re.compile(_STAMP + r' \[MPPIController\]: SE escape ENTER at ' + _XY.format('x', 'y'))
EXIT_RE = re.compile(_STAMP + r' \[MPPIController\]: SE escape EXIT at ' + _XY.format('x', 'y'))
WAYPOINT_RE = re.compile(
    _STAMP + r' \[MPPIController\]: SE escape waypoint (?P<hop>\d+) \((?P<why>[^)]*)\): '
    r'from ' + _XY.format('fx', 'fy') + r' to ' + _XY.format('tx', 'ty') +
    r'; bearing (?P<bearing>-?\d+) deg, clearance (?P<clearance>[\d.]+) m, '
    r'(?P<kind>horizon ray|longest ray \(no horizon ray\))')
INJECT_BEGIN_RE = re.compile(r'escape-cost injection BEGIN')
INJECT_END_RE = re.compile(r'escape-cost injection END')
# The critic's probe line is NOT the escape heading (no avoid list, no hysteresis, its goal is
# the escape waypoint). The controller's heading is the "SE escape waypoint" line (WAYPOINT_RE).
CRITIC_PROBE_RE = re.compile(
    _STAMP + r' \[controller_server\]: EscapeCritic\[[^\]]*\] critic-side probe \(no avoid list, '
    r'no hysteresis; NOT the escape heading\): bearing '
    r'(?P<bearing>-?\d+) deg, clearance (?P<clearance>[\d.]+) m')
OPTFAIL_RE = re.compile(_STAMP + r' \[controller_server\]: Optimizer fail to compute path')
PROGRESS_FAIL_RE = re.compile(r'\[controller_server\]: Failed to make progress')
MISSED_RE = re.compile(r'Control loop missed its desired rate')
SPIN_RE = re.compile(r'\[behavior_server\]: Running spin\b')
BACKUP_RE = re.compile(r'\[behavior_server\]: Running backup\b')
WAIT_RE = re.compile(r'\[behavior_server\]: Running wait\b')
CLEAR_LOCAL_RE = re.compile(r'Received request to clear entirely the local_costmap')
CLEAR_GLOBAL_RE = re.compile(r'Received request to clear entirely the global_costmap')
VX_MIN_WARN_RE = re.compile(r'vx_min = [-\d.]+ > 0 with se_enabled')
DIED_RE = re.compile(r'process has died \[pid \d+, exit code (?P<code>-?\d+)')
# Exit codes of the teardown: the driver stops a trial with SIGINT, SIGTERM, SIGKILL in that
# order, so a container that was killed shows -2, -15 or -9, and one that exited cleanly 0.
# Any other exit code (SIGSEGV -11, SIGABRT -6, a positive status) is a crash.
TEARDOWN_EXIT_CODES = (0, -2, -15, -9)
KEY_LINE_RE = re.compile(
    r'SE escape |escape-cost injection|critic-side probe|'
    r'Failed to make progress|Running (?:spin|backup|wait)\b|'
    r'Received request to clear entirely|SafeEscapeController\[|Control loop missed|'
    r'process has died|Aborting|Goal (?:succeeded|failed|canceled)|NavigateToPose')


def read_text(path: str) -> str:
    return pilot.read_text(path)


def fnum(tok):
    return pilot.fnum(tok)


# --------------------------------------------------------------------------- #
# World geometry
# --------------------------------------------------------------------------- #
def world_boxes(path: str) -> list:
    """(name, corner list) of every static box of a world file except the ground plane."""
    boxes = []
    for model in ET.parse(path).getroot().iter('model'):
        size = model.find('./link/collision/geometry/box/size')
        pose = model.find('pose')
        if size is None or pose is None:
            continue
        sx, sy, _ = (float(v) for v in size.text.split())
        x, y, _, _, _, yaw = (float(v) for v in pose.text.split())
        boxes.append((model.get('name'), pilot.rect(x, y, yaw, sx, sy)))
    return boxes


# --------------------------------------------------------------------------- #
# Log parsing
# --------------------------------------------------------------------------- #
def parse_launch_log(text: str, t0: float | None) -> dict:
    """Escape episodes, hops and Nav2 recovery counters from a launch log."""
    def rel(stamp):
        return None if t0 is None else float(stamp) - t0

    enters = [(float(m.group('stamp')), float(m.group('x')), float(m.group('y')))
              for m in ENTER_RE.finditer(text)]
    exits = [(float(m.group('stamp')), float(m.group('x')), float(m.group('y')))
             for m in EXIT_RE.finditer(text)]
    # Pair events in time order: an ENTER opens an episode, the next EXIT closes it.
    events = sorted([(t, 'ENTER', x, y) for t, x, y in enters] +
                    [(t, 'EXIT', x, y) for t, x, y in exits])
    episodes, open_ep = [], None
    for t, kind, x, y in events:
        if kind == 'ENTER':
            if open_ep is not None:               # a second ENTER without an EXIT
                episodes.append(open_ep)
            open_ep = {'enter_wall': t, 'exit_wall': None, 'enter_xy': [x, y]}
        elif open_ep is not None:
            open_ep['exit_wall'] = t
            episodes.append(open_ep)
            open_ep = None
    if open_ep is not None:
        episodes.append(open_ep)
    for ep in episodes:
        ep['enter_s'] = rel(ep['enter_wall'])
        ep['exit_s'] = rel(ep['exit_wall']) if ep['exit_wall'] is not None else None
        ep['duration_s'] = (ep['exit_wall'] - ep['enter_wall']
                            if ep['exit_wall'] is not None else None)
    hops = [{'stamp': float(m.group('stamp')), 't_s': rel(m.group('stamp')),
             'hop': int(m.group('hop')), 'why': m.group('why'),
             'from': [float(m.group('fx')), float(m.group('fy'))],
             'to': [float(m.group('tx')), float(m.group('ty'))],
             'bearing_deg': int(m.group('bearing')), 'clearance_m': float(m.group('clearance')),
             'horizon_ray': m.group('kind') == 'horizon ray'}
            for m in WAYPOINT_RE.finditer(text)]
    optfails = [float(m.group('stamp')) for m in OPTFAIL_RE.finditer(text)]
    done = [e['duration_s'] for e in episodes if e['duration_s'] is not None]
    return {
        'escape_enter': len(enters),
        'escape_exit': len(exits),
        'escape_episodes': [{k: ep[k] for k in ('enter_s', 'exit_s', 'duration_s', 'enter_xy')}
                            for ep in episodes],
        'escape_enter_walls': [e['enter_wall'] for e in episodes],
        'escape_total_s': sum(done) if done else 0.0,
        'escape_open_at_end': bool(episodes and episodes[-1]['exit_wall'] is None),
        'escape_hops': len(hops),
        'escape_hop_list': hops,
        'escape_injection_begin': len(INJECT_BEGIN_RE.findall(text)),
        'escape_injection_end': len(INJECT_END_RE.findall(text)),
        'escape_critic_probe_lines': len(CRITIC_PROBE_RE.findall(text)),
        'first_escape_enter_s': rel(enters[0][0]) if enters else None,
        'optimizer_failures': len(optfails),
        'first_optimizer_failure_s': rel(optfails[0]) if optfails else None,
        'progress_failures': len(PROGRESS_FAIL_RE.findall(text)),
        'missed_control_cycles': len(MISSED_RE.findall(text)),
        'bt_spin': len(SPIN_RE.findall(text)),
        'bt_backup': len(BACKUP_RE.findall(text)),
        'bt_wait': len(WAIT_RE.findall(text)),
        'bt_clear_local': len(CLEAR_LOCAL_RE.findall(text)),
        'bt_clear_global': len(CLEAR_GLOBAL_RE.findall(text)),
        'vx_min_warning': bool(VX_MIN_WARN_RE.search(text)),
        'process_exit_codes': [int(m.group('code')) for m in DIED_RE.finditer(text)],
        'process_crash': any(int(m.group('code')) not in TEARDOWN_EXIT_CODES
                             for m in DIED_RE.finditer(text)),
    }


def parse_escape_path_log(text: str) -> dict:
    """Summary of ``escape_path.jsonl``. Records with the shakedown probe frame are
    counted apart and take part in no statistic."""
    paths, probes, publishers = 0, 0, []
    first = None
    for ln in text.splitlines():
        if not ln.strip():
            continue
        try:
            rec = json.loads(ln)
        except ValueError:
            continue
        if rec.get('kind') == 'graph':
            publishers.append(rec.get('publishers'))
        elif rec.get('kind') == 'path':
            if rec.get('frame') == PROBE_FRAME:
                probes += 1
            else:
                paths += 1
                if first is None:
                    first = rec.get('wall_s')
    return {'escape_path_messages': paths, 'escape_path_probe_messages': probes,
            'escape_path_publishers_seen': max((p for p in publishers if p is not None),
                                               default=None),
            'escape_path_graph_records': len(publishers),
            'escape_path_first_wall': first}


def excerpt(launch_text: str) -> str:
    """The lines of a launch log that carry evidence (escape, recovery, progress), the
    first three optimizer failures and a count of the rest."""
    out, fails = [], 0
    for ln in launch_text.splitlines():
        if OPTFAIL_RE.search(ln):
            fails += 1
            if fails <= 3:
                out.append(ln)
        elif KEY_LINE_RE.search(ln) and not MISSED_RE.search(ln):
            out.append(ln)
    missed = len(MISSED_RE.findall(launch_text))
    out.append(f'# optimizer failures: {fails}; control loop missed its rate: {missed} times')
    return '\n'.join(out) + '\n'


# --------------------------------------------------------------------------- #
# One trial
# --------------------------------------------------------------------------- #
def read_telemetry(tdir: str) -> list:
    text = read_text(os.path.join(tdir, 'telemetry.csv'))
    import csv
    return [{k: fnum(v) for k, v in r.items()} for r in csv.DictReader(io.StringIO(text))]


def pocket_times(drive_rows: list, scenario: str):
    """(entered_wall, left_wall) of the pocket from ground truth; None when absent."""
    poly = POCKETS.get(scenario)
    if poly is None:
        return None, None
    entered = left = None
    for r in drive_rows:
        inside = inside_polygon((r['gt_x'], r['gt_y']), poly)
        if entered is None:
            if inside:
                entered = r['wall_s']
        elif not inside:
            left = r['wall_s']
            break
    return entered, left


def analyze_trial(tdir: str, boxes: list, chassis: tuple, scenario: str | None = None) -> dict:
    with open(os.path.join(tdir, 'meta.json')) as f:
        meta = json.load(f)
    scenario = scenario or meta['scenario']
    cfg = meta['config']
    spec = plan_mod.CONFIGS.get(cfg, {})
    rec = {
        'trial_id': os.path.basename(tdir), 'scenario': scenario, 'config': cfg,
        'rep': meta['rep'], 'phase': meta.get('phase'),
        'controller': spec.get('ablation'), 'se_config': bool(spec.get('se')),
        'repo_commit': meta.get('env', {}).get('repo_commit'),
        'params_sha256': meta.get('params_yaml_sha256'),
        'setup_attempts': meta.get('setup_attempts'),
        'load1_at_start': meta.get('load1_at_start'),
        'load_gate_waited_s': meta.get('load_gate_waited_s'),
    }
    if not meta.get('complete'):
        rec.update(outcome='INVALID', invalid_reason='driver did not finish the trial')
        return rec
    if not meta.get('active'):
        rec['outcome'] = 'SETUP_FAIL'
        return rec

    launch = read_text(os.path.join(tdir, meta.get('launch_log', 'launch.log')))
    params = read_text(os.path.join(tdir, 'params.yaml'))
    drive = pilot.drive_stats(read_text(os.path.join(tdir, 'drive.log')), pilot.vx_min_of(params))
    rec.update({k: drive[k] for k in ('nav2_result', 'watchdog_fired', 'n_telemetry_lines')})
    t0, t1 = drive['t0'], drive['t1']
    rec.update(parse_launch_log(launch, t0))
    rec.update(parse_escape_path_log(read_text(os.path.join(tdir, 'escape_path.jsonl'))))
    diag = [json.loads(ln) for ln in
            read_text(os.path.join(tdir, 'diagnostics.jsonl')).splitlines() if ln.strip()]
    rec['guard_state'] = diag[-1]['values'].get('mppi_rollout_check') if diag else None
    rec['diagnostics_last'] = ({k: diag[-1]['values'].get(k) for k in
                                ('cycles', 'escape_entries', 'escape_hops', 'base_failures',
                                 'forced_stops', 'qp_failures', 'vx_min', 'escape_bearing_deg',
                                 'escape_clearance')} if diag else None)

    if t0 is None or t1 is None:
        rec['outcome'] = 'NO_GOAL_SENT'
        return rec
    rows = read_telemetry(tdir)
    gt_rows = [r for r in rows if not math.isnan(r['gt_x'])]
    start, goal = meta['start'], meta['goal']
    if not gt_rows:
        rec.update(outcome='INVALID', invalid_reason='no ground truth in telemetry.csv')
        return rec
    d_start = math.hypot(gt_rows[0]['gt_x'] - start[0], gt_rows[0]['gt_y'] - start[1])
    if d_start > START_TOL:
        rec.update(outcome='INVALID',
                   invalid_reason=f'first ground-truth sample {d_start:.3f} m from the start')
        return rec
    drive_rows = [r for r in gt_rows if t0 <= r['wall_s'] <= t1 + 0.2]
    if not drive_rows:
        rec.update(outcome='INVALID',
                   invalid_reason='no ground-truth sample inside the drive interval')
        return rec

    last = drive_rows[-1]
    rec['final_gt_xy'] = [round(last['gt_x'], 3), round(last['gt_y'], 3)]
    rec['final_goal_distance_m'] = math.hypot(last['gt_x'] - goal[0], last['gt_y'] - goal[1])
    rec['reached'] = rec['final_goal_distance_m'] <= GOAL_TOL
    rec['drive_wall_s'] = t1 - t0

    lx, ly = chassis
    min_d, where = math.inf, None
    for r in drive_rows:
        body = pilot.rect(r['gt_x'], r['gt_y'], r['gt_yaw'], lx, ly)
        for name, box in boxes:
            d = pilot.rect_distance(body, box)
            if d < min_d:
                min_d, where = d, name
    rec['min_wall_distance_m'] = min_d
    rec['nearest_wall'] = where
    rec['contact'] = min_d < CONTACT_TOL

    sim_rows = [r for r in drive_rows if not math.isnan(r['sim_s'])]
    if len(sim_rows) >= 2:
        wall = sim_rows[-1]['wall_s'] - sim_rows[0]['wall_s']
        sim = sim_rows[-1]['sim_s'] - sim_rows[0]['sim_s']
        rec['drive_sim_s'] = sim
        rec['rtf'] = sim / wall if wall > 0 else None
        windows, i = [], 0
        while i < len(sim_rows):
            j = i
            while j < len(sim_rows) and sim_rows[j]['wall_s'] - sim_rows[i]['wall_s'] < RTF_WINDOW_S:
                j += 1
            if j < len(sim_rows):
                windows.append((sim_rows[j]['sim_s'] - sim_rows[i]['sim_s'])
                               / (sim_rows[j]['wall_s'] - sim_rows[i]['wall_s']))
            i = j
        rec['rtf_min_10s'] = min(windows) if windows else None
    else:
        rec.update(drive_sim_s=None, rtf=None, rtf_min_10s=None)
    rec['load1_mean'] = sum(r['load1'] for r in drive_rows) / len(drive_rows)

    # Time to goal: first ground-truth sample within the goal tolerance.
    rec['time_to_goal_wall_s'] = rec['time_to_goal_sim_s'] = None
    if rec['reached']:
        hit = next((r for r in drive_rows
                    if math.hypot(r['gt_x'] - goal[0], r['gt_y'] - goal[1]) <= GOAL_TOL), None)
        if hit is not None:
            rec['time_to_goal_wall_s'] = hit['wall_s'] - t0
            if not math.isnan(hit['sim_s']) and sim_rows:
                rec['time_to_goal_sim_s'] = hit['sim_s'] - sim_rows[0]['sim_s']

    # Pocket: did SE escape ENTER come before the chassis left the pocket polygon?
    entered, left = pocket_times(drive_rows, scenario)
    rec['pocket'] = scenario in POCKETS
    rec['entered_pocket_s'] = entered - t0 if entered is not None else None
    rec['left_pocket_s'] = left - t0 if left is not None else None
    rec['enter_before_leaving_pocket'] = None
    rec['reached_without_escape'] = None
    if rec['se_config'] and rec['pocket'] and entered is not None:
        limit = left if left is not None else math.inf
        rec['enter_before_leaving_pocket'] = any(w <= limit for w in rec['escape_enter_walls'])

    if rec['reached']:
        rec['outcome'] = 'REACHED_WITH_CONTACT' if rec['contact'] else 'REACHED'
    elif drive['watchdog_fired']:
        rec['outcome'] = 'NOT_REACHED_TIMEOUT'
    else:
        rec['outcome'] = 'NOT_REACHED_ABORTED'
    rec['success'] = rec['outcome'] == 'REACHED'
    rec['nav2_false_success'] = rec['nav2_result'] == 'SUCCEEDED' and not rec['reached']
    if rec['success'] and rec['se_config'] and rec['pocket']:
        rec['reached_without_escape'] = rec['enter_before_leaving_pocket'] is not True

    load = hostload.verdict(hostload.read_rows(os.path.join(tdir, 'host_cpu.csv')),
                            meta['drive_wall_start'], meta['drive_wall_end'])
    rec['host_load'] = load
    if not load['valid']:
        rec['outcome_measured'] = rec['outcome']
        rec['outcome'] = 'INVALID_HOST_LOAD'
        rec['success'] = False
        rec['invalid_reason'] = load['reason']
    return rec


# --------------------------------------------------------------------------- #
# Statistics
# --------------------------------------------------------------------------- #
def wilson(k: int, n: int, z: float = 1.959964) -> tuple | None:
    """Wilson score interval of k successes in n trials."""
    if n <= 0:
        return None
    p = k / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, centre - half), min(1.0, centre + half))


def mann_whitney(a: list, b: list) -> dict | None:
    """Two-sided Mann-Whitney U (descriptive). Exact by enumeration for up to 20,000
    arrangements, otherwise the normal approximation with tie correction."""
    if not a or not b:
        return None
    n1, n2 = len(a), len(b)
    pooled = sorted(a + b)

    def ranks(vals):
        out, i = {}, 0
        while i < len(vals):
            j = i
            while j + 1 < len(vals) and vals[j + 1] == vals[i]:
                j += 1
            out[vals[i]] = 0.5 * (i + j) + 1.0
            i = j + 1
        return out
    rk = ranks(pooled)
    r1 = sum(rk[v] for v in a)
    u1 = r1 - n1 * (n1 + 1) / 2.0
    u = min(u1, n1 * n2 - u1)
    from itertools import combinations
    total = math.comb(n1 + n2, n1)
    if total <= 20000:
        obs = abs(u1 - n1 * n2 / 2.0)
        hit = 0
        for idx in combinations(range(n1 + n2), n1):
            s = sum(rk[pooled[i]] for i in idx) - n1 * (n1 + 1) / 2.0
            if abs(s - n1 * n2 / 2.0) >= obs - 1e-9:
                hit += 1
        p = hit / total
        method = 'exact'
    else:
        n = n1 + n2
        ties = {}
        for v in pooled:
            ties[v] = ties.get(v, 0) + 1
        corr = sum(t ** 3 - t for t in ties.values()) / (n * (n - 1))
        var = n1 * n2 / 12.0 * ((n + 1) - corr)
        z = (abs(u1 - n1 * n2 / 2.0) - 0.5) / math.sqrt(var) if var > 0 else 0.0
        p = math.erfc(z / math.sqrt(2))
        method = 'normal'
    med = lambda v: sorted(v)[len(v) // 2] if len(v) % 2 else 0.5 * (sorted(v)[len(v) // 2 - 1] + sorted(v)[len(v) // 2])
    return {'U': u, 'p': min(1.0, p), 'method': method, 'n1': n1, 'n2': n2,
            'median1': med(a), 'median2': med(b)}


def is_valid_driven(t: dict) -> bool:
    """A trial that counts toward a cell: it ran and was not invalidated."""
    return t['outcome'] not in ('INVALID', 'INVALID_HOST_LOAD', 'SETUP_FAIL', 'NO_GOAL_SENT')


def cell_stats(trials: list) -> dict:
    valid = [t for t in trials if is_valid_driven(t)]
    k = sum(1 for t in valid if t['success'])
    outcomes = {}
    for t in trials:
        outcomes[t['outcome']] = outcomes.get(t['outcome'], 0) + 1
    ttg = [t['time_to_goal_wall_s'] for t in valid if t['success'] and t.get('time_to_goal_wall_s') is not None]
    ttg_sim = [t['time_to_goal_sim_s'] for t in valid if t['success'] and t.get('time_to_goal_sim_s') is not None]
    ci = wilson(k, len(valid))
    return {
        'n_trials': len(trials), 'n_valid': len(valid), 'successes': k,
        'wilson95': [round(ci[0], 4), round(ci[1], 4)] if ci else None,
        'outcomes': outcomes,
        'reached_with_contact': sum(t['outcome'] == 'REACHED_WITH_CONTACT' for t in valid),
        'nav2_false_success': sum(bool(t.get('nav2_false_success')) for t in valid),
        'time_to_goal_wall_s': ttg, 'time_to_goal_sim_s': ttg_sim,
        'trials_with_escape_enter': sum(t.get('escape_enter', 0) > 0 for t in valid),
        'escape_enter_total': sum(t.get('escape_enter', 0) for t in valid),
        'escape_hops_total': sum(t.get('escape_hops', 0) for t in valid),
        'reached_without_escape': sum(bool(t.get('reached_without_escape')) for t in valid),
        'bt_spin': sum(t.get('bt_spin', 0) for t in valid),
        'bt_backup': sum(t.get('bt_backup', 0) for t in valid),
        'bt_wait': sum(t.get('bt_wait', 0) for t in valid),
        'bt_clear_local': sum(t.get('bt_clear_local', 0) for t in valid),
        'bt_clear_global': sum(t.get('bt_clear_global', 0) for t in valid),
        'progress_failures': sum(t.get('progress_failures', 0) for t in valid),
        'optimizer_failures_total': sum(t.get('optimizer_failures', 0) for t in valid),
        'trials_with_optimizer_failures': sum(t.get('optimizer_failures', 0) > 0 for t in valid),
        'missed_control_cycles_total': sum(t.get('missed_control_cycles', 0) for t in valid),
        'rtf_min': min((t['rtf'] for t in valid if t.get('rtf') is not None), default=None),
        'rtf_max': max((t['rtf'] for t in valid if t.get('rtf') is not None), default=None),
        'foreign_cores_mean_max': max((t['host_load']['foreign_cores_mean'] for t in valid
                                       if t.get('host_load')), default=None),
    }


CONTRASTS = [
    ('C4 vs C3 on s1_utrap', ('s1_utrap', 'C4'), ('s1_utrap', 'C3')),
    ('C4 vs C3 on s3_ctrap', ('s3_ctrap', 'C4'), ('s3_ctrap', 'C3')),
    ('C2 vs C1 on s1_utrap', ('s1_utrap', 'C2'), ('s1_utrap', 'C1')),
    ('C2 vs C1 on s3_ctrap', ('s3_ctrap', 'C2'), ('s3_ctrap', 'C1')),
    ('C4 on s3_ctrap vs C4 on s1_utrap (transfer)', ('s3_ctrap', 'C4'), ('s1_utrap', 'C4')),
]


def contrasts(cells: dict) -> list:
    out = []
    for name, ka, kb in CONTRASTS:
        a, b = cells.get(ka), cells.get(kb)
        row = {'name': name, 'a': list(ka), 'b': list(kb)}
        if not a or not b or a['n_valid'] == 0 or b['n_valid'] == 0:
            row.update(fisher_p=None, mann_whitney=None)
        else:
            ka_, na = a['successes'], a['n_valid']
            kb_, nb = b['successes'], b['n_valid']
            row.update(a_successes=ka_, a_n=na, b_successes=kb_, b_n=nb,
                       fisher_p=pilot.fisher_two_sided(ka_, na - ka_, kb_, nb - kb_),
                       mann_whitney=mann_whitney(a['time_to_goal_wall_s'], b['time_to_goal_wall_s']))
        out.append(row)
    return out


def h3(cells: dict) -> dict:
    """The trap-free control: reached count and ENTER count on s0_open, all configurations."""
    reached = n = enters = 0
    per = {}
    for c in plan_mod.CONFIG_ORDER:
        cs = cells.get(('s0_open', c))
        if not cs:
            continue
        per[c] = {'successes': cs['successes'], 'n_valid': cs['n_valid'],
                  'escape_enter_total': cs['escape_enter_total']}
        reached += cs['successes']
        n += cs['n_valid']
        enters += cs['escape_enter_total']
    return {'reached': reached, 'n_valid': n, 'escape_enter_total': enters, 'per_config': per}


# --------------------------------------------------------------------------- #
# Stopping rules (used by the driver after every trial)
# --------------------------------------------------------------------------- #
def count_invalid_attempts(raw_root: str, cell: tuple) -> int:
    """Attempts of a cell that were kept as ``.invalidN``."""
    pat = os.path.join(raw_root, 'trials', f'{plan_mod.ENV_LABEL}__{cell[0]}__{cell[1]}__rep*.invalid*')
    return len(glob.glob(pat))


def evaluate_stops(records: list, invalid_attempts: dict | None = None) -> list:
    """The stopping rules of PREREG.md applied to the registered trials so far.

    Returns a list of (code, message); an empty list means: continue.
    """
    stops = []
    by_cell = {}
    for t in records:
        by_cell.setdefault((t['scenario'], t['config']), []).append(t)

    def valid(cell):
        return [t for t in by_cell.get(cell, []) if is_valid_driven(t)]
    gate = valid(plan_mod.GATE_CELL)
    if len(gate) >= plan_mod.PHASE_A_C4_REPS:
        k = sum(1 for t in gate if t['success'])
        if k <= plan_mod.GATE_MAX_SUCCESSES:
            stops.append(('PHASE_A_GATE',
                          f'C4 x s1_utrap reached {k} of {len(gate)} (<= '
                          f'{plan_mod.GATE_MAX_SUCCESSES}): stop and return to the headless test'))
    for c in ('C1', 'C3'):
        s0 = valid(('s0_open', c))
        if len(s0) >= plan_mod.REPS:
            k = sum(1 for t in s0 if t['success'])
            if k <= plan_mod.STOCK_S0_MAX_SUCCESSES:
                stops.append(('STACK_BROKEN',
                              f'stock MPPI ({c}) reached {k} of {len(s0)} on s0_open (<= '
                              f'{plan_mod.STOCK_S0_MAX_SUCCESSES}): the stack is broken'))
    for cell, n in (invalid_attempts or {}).items():
        if n > plan_mod.MAX_INVALID_PER_CELL:
            stops.append(('INVALID_CAP', f'cell {cell} has {n} invalid attempts '
                          f'(> {plan_mod.MAX_INVALID_PER_CELL}): report it as incomplete'))
    for t in records:
        if t.get('process_crash'):
            stops.append(('CONTROLLER_CRASH', f'{t["trial_id"]}: a launched process died with exit '
                          f'code {t["process_exit_codes"]} (not a teardown signal)'))
    for t in records:
        if t['outcome'] == 'SETUP_FAIL':
            stops.append(('SETUP_FAIL', f'{t["trial_id"]}: the stack did not become active '
                          '(controller crash or refusal to configure)'))
    return stops


# --------------------------------------------------------------------------- #
# Whole campaign
# --------------------------------------------------------------------------- #
_CACHE = {}


def analyze_dir(tdir: str, boxes_cache: dict, chassis: tuple) -> dict:
    meta_path = os.path.join(tdir, 'meta.json')
    key = (tdir, os.path.getmtime(meta_path), os.path.getsize(meta_path))
    if key in _CACHE:
        return _CACHE[key]
    with open(meta_path) as f:
        scenario = json.load(f)['scenario']
    if scenario not in boxes_cache:
        boxes_cache[scenario] = world_boxes(WORLDS[scenario])
    rec = analyze_trial(tdir, boxes_cache[scenario], chassis)
    if rec.get('outcome') not in ('INVALID',) or 'driver did not finish' not in rec.get('invalid_reason', ''):
        _CACHE[key] = rec
    return rec


def load_dir(raw_root: str, sub: str) -> list:
    boxes_cache, chassis = {}, pilot.chassis_size()
    out = []
    for d in sorted(glob.glob(os.path.join(raw_root, sub, '*'))):
        if os.path.isdir(d) and os.path.exists(os.path.join(d, 'meta.json')):
            out.append(analyze_dir(d, boxes_cache, chassis))
    return out


def registered_records(raw_root: str) -> list:
    """Registered trials: the directories of ``trials/`` without a suffix."""
    return [t for t in load_dir(raw_root, 'trials') if '.' not in t['trial_id']]


def build(raw_root: str) -> dict:
    everything = load_dir(raw_root, 'trials')
    trials = [t for t in everything if '.' not in t['trial_id']]
    superseded = [t for t in everything if '.' in t['trial_id']]
    counted = [t for t in trials if t['scenario'] in plan_mod.SCENARIO_ORDER]
    evidence = [t for t in trials if t['scenario'] == plan_mod.EVIDENCE_SCENARIO]
    groups = {}
    for t in counted:
        groups.setdefault((t['scenario'], t['config']), []).append(t)
    cells = {k: cell_stats(v) for k, v in groups.items()}
    ev_groups = {}
    for t in evidence:
        ev_groups.setdefault((t['scenario'], t['config']), []).append(t)
    result = {
        'definitions': {
            'goal_tol_m': GOAL_TOL, 'contact_tol_m': CONTACT_TOL, 'rtf_window_s': RTF_WINDOW_S,
            'success': 'REACHED: ground truth within goal_tol_m of the goal at drive end and no '
                       'contact (chassis-to-wall distance >= contact_tol_m)',
            'time_to_goal': 'first ground-truth sample within goal_tol_m, from the goal being sent',
            'pocket_polygons': {k: [list(map(lambda v: round(v, 4), p)) for p in poly]
                                for k, poly in POCKETS.items()},
        },
        'host_load_rule': {'admit_load1_below': hostload.ADMIT_LOAD1,
                           'admit_spacing_s': hostload.ADMIT_SPACING_S,
                           'admit_busy_cores_below': hostload.ADMIT_BUSY_CORES,
                           'valid_mean_foreign_cores': hostload.VALID_MEAN_CORES,
                           'valid_window_foreign_cores': hostload.VALID_WINDOW_CORES,
                           'window_s': hostload.WINDOW_S},
        'trials': trials,
        'cells': [dict(scenario=k[0], config=k[1], **v) for k, v in sorted(cells.items())],
        'evidence_cells': [dict(scenario=k[0], config=k[1], **cell_stats(v))
                           for k, v in sorted(ev_groups.items())],
        'contrasts': contrasts(cells),
        'h3_control': h3(cells),
        'superseded_attempts': superseded,
        'shakedown': load_dir(raw_root, 'shakedown'),
        'stops': evaluate_stops(trials, {
            f'{k[0]}/{k[1]}': count_invalid_attempts(raw_root, k) for k in groups}),
    }
    return result


def fmt(v, nd=2):
    if v is None:
        return '--'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def render(result: dict) -> str:
    L = ['# Campaign 2: results', '',
         'Generated by `analyze.py` from the raw files outside the repository. '
         'Definitions and design: `PREREG.md`. Do not edit by hand.', '']
    L += ['## Cells (counted scenarios)', '',
          '| Scenario | Config | Trials | Valid | Reached (no contact) | Wilson 95% | With contact | Outcomes | ENTER total (trials with ENTER) | Hops | Reached without escape | Spin / BackUp / Wait | Clear local / global | Failed to make progress | Optimizer failures (trials) | Missed cycles | RTF min..max |',
          '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for c in result['cells']:
        ci = c['wilson95']
        L.append('| ' + ' | '.join([
            c['scenario'], c['config'], str(c['n_trials']), str(c['n_valid']), str(c['successes']),
            f'{ci[0]:.2f}..{ci[1]:.2f}' if ci else '--', str(c['reached_with_contact']),
            ', '.join(f'{k} {v}' for k, v in sorted(c['outcomes'].items())),
            f"{c['escape_enter_total']} ({c['trials_with_escape_enter']})", str(c['escape_hops_total']),
            str(c['reached_without_escape']),
            f"{c['bt_spin']} / {c['bt_backup']} / {c['bt_wait']}",
            f"{c['bt_clear_local']} / {c['bt_clear_global']}", str(c['progress_failures']),
            f"{c['optimizer_failures_total']} ({c['trials_with_optimizer_failures']})",
            str(c['missed_control_cycles_total']),
            f"{fmt(c['rtf_min'])}..{fmt(c['rtf_max'])}"]) + ' |')
    L += ['', '## Evidence trials (not counted)', '',
          '| Scenario | Config | Trials | ENTER total | Hops | Reached |', '|---|---|---|---|---|---|']
    for c in result['evidence_cells']:
        L.append(f"| {c['scenario']} | {c['config']} | {c['n_trials']} | {c['escape_enter_total']} "
                 f"| {c['escape_hops_total']} | {c['successes']} |")
    L += ['', '## Contrasts (two-sided Fisher exact on reached counts; Mann-Whitney on wall time to goal, descriptive)', '',
          '| Contrast | A reached | B reached | Fisher p | Time to goal: median A / B (n) | Mann-Whitney p |',
          '|---|---|---|---|---|---|']
    for r in result['contrasts']:
        if r['fisher_p'] is None:
            L.append(f"| {r['name']} | -- | -- | -- | -- | -- |")
            continue
        mw = r['mann_whitney']
        L.append(f"| {r['name']} | {r['a_successes']} of {r['a_n']} | {r['b_successes']} of {r['b_n']} "
                 f"| {r['fisher_p']:.4f} | "
                 + (f"{mw['median1']:.1f} / {mw['median2']:.1f} ({mw['n1']}, {mw['n2']}) | {mw['p']:.4f} ({mw['method']}) |"
                    if mw else '-- | -- |'))
    h = result['h3_control']
    L += ['', '## H3: trap-free control (s0_open)', '',
          f"Reached {h['reached']} of {h['n_valid']} valid trials; escape ENTER lines: {h['escape_enter_total']}.", '']
    if result['stops']:
        L += ['## Stopping rules triggered', ''] + [f'- {c}: {m}' for c, m in result['stops']] + ['']

    def table(title, trials):
        out = ['', f'## {title}', '',
               '| Trial | Phase | Outcome | Nav2 result | Final distance (m) | Min wall distance (m) | Time to goal wall / sim (s) | ENTER / EXIT | Escape episodes (s) | Hops | Entered / left pocket (s) | ENTER before leaving pocket | Escape-path msgs (probe) / publishers | Spin / BackUp / Wait / clears | Failed to make progress | Optimizer failures | Missed cycles | RTF (min 10 s) | Foreign CPU mean / worst 30 s |',
               '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
        for t in trials:
            eps = ', '.join(fmt(e['duration_s'], 1) if e['duration_s'] is not None else 'open'
                            for e in t.get('escape_episodes', [])) or '--'
            hl = t.get('host_load') or {}
            out.append('| ' + ' | '.join([
                t['trial_id'], fmt(t.get('phase')),
                t['outcome'] + (f" ({t['invalid_reason']})" if t.get('invalid_reason') else ''),
                fmt(t.get('nav2_result')), fmt(t.get('final_goal_distance_m')),
                fmt(t.get('min_wall_distance_m'), 3),
                f"{fmt(t.get('time_to_goal_wall_s'), 1)} / {fmt(t.get('time_to_goal_sim_s'), 1)}",
                f"{fmt(t.get('escape_enter'))} / {fmt(t.get('escape_exit'))}", eps,
                fmt(t.get('escape_hops')),
                f"{fmt(t.get('entered_pocket_s'), 1)} / {fmt(t.get('left_pocket_s'), 1)}",
                fmt(t.get('enter_before_leaving_pocket')),
                f"{fmt(t.get('escape_path_messages'))} ({fmt(t.get('escape_path_probe_messages'))}) / "
                f"{fmt(t.get('escape_path_publishers_seen'))}",
                f"{fmt(t.get('bt_spin'))} / {fmt(t.get('bt_backup'))} / {fmt(t.get('bt_wait'))} / "
                f"{fmt(t.get('bt_clear_local'))}+{fmt(t.get('bt_clear_global'))}",
                fmt(t.get('progress_failures')), fmt(t.get('optimizer_failures')),
                fmt(t.get('missed_control_cycles')),
                f"{fmt(t.get('rtf'))} ({fmt(t.get('rtf_min_10s'))})",
                f"{fmt(hl.get('foreign_cores_mean'))} / {fmt(hl.get('foreign_cores_worst_window'))}"]) + ' |')
        return out
    L += table('Registered trials', result['trials'])
    L += table('Attempts invalidated by host load or interrupted, and repeated (not counted above)',
               result['superseded_attempts'])
    L += table('Shakedown runs (not registered trials)', result['shakedown'])
    return '\n'.join(L) + '\n'


def gz_bytes(text: str) -> bytes:
    buf = io.BytesIO()
    with gzip.GzipFile(fileobj=buf, mode='wb', mtime=0, compresslevel=9) as g:
        g.write(text.encode())
    return buf.getvalue()


def pack_excerpts(raw_root: str, out_dir: str) -> int:
    os.makedirs(out_dir, exist_ok=True)
    n = 0
    for sub in ('trials', 'shakedown'):
        for d in sorted(glob.glob(os.path.join(raw_root, sub, '*'))):
            meta_path = os.path.join(d, 'meta.json')
            if not os.path.exists(meta_path):
                continue
            with open(meta_path) as f:
                meta = json.load(f)
            text = read_text(os.path.join(d, meta.get('launch_log', 'launch.log')))
            sha = hashlib.sha256(text.encode()).hexdigest()
            body = f'# {os.path.basename(d)}; sha256 of the full launch log: {sha}\n' + excerpt(text)
            with open(os.path.join(out_dir, f'{sub}__{os.path.basename(d)}.log.gz'), 'wb') as f:
                f.write(gz_bytes(body))
            n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', default=plan_mod.RAW_ROOT)
    ap.add_argument('--out', default=HERE)
    ap.add_argument('--check', action='store_true',
                    help='fail if results.json.gz or RESULTS.md differ from a fresh analysis')
    ap.add_argument('--pack', action='store_true', help='also write logs/<trial>.log.gz excerpts')
    args = ap.parse_args()
    result = build(args.raw)
    js = gz_bytes(json.dumps(result, indent=1, sort_keys=True) + '\n')
    md = render(result).encode()
    targets = ((os.path.join(args.out, 'results.json.gz'), js), (os.path.join(args.out, 'RESULTS.md'), md))
    if args.check:
        stale = [p for p, data in targets if not os.path.exists(p) or open(p, 'rb').read() != data]
        if stale:
            print('stale: ' + ', '.join(os.path.relpath(p, REPO) for p in stale))
            return 1
        print(f'results.json.gz and RESULTS.md match a fresh analysis ({len(result["trials"])} trials)')
        return 0
    for p, data in targets:
        with open(p, 'wb') as f:
            f.write(data)
    if args.pack:
        print(f'wrote {pack_excerpts(args.raw, os.path.join(args.out, "logs"))} log excerpts')
    print(f'wrote results.json.gz and RESULTS.md ({len(result["trials"])} trials, '
          f'{len(result["shakedown"])} shakedown runs)')
    for code, msg in result['stops']:
        print(f'STOP {code}: {msg}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
