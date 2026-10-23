#!/usr/bin/env python3
"""Post-hoc check (not part of the frozen analysis): compare the Gazebo ground truth with the
AMCL pose that Nav2's goal checker uses (last ``amcl=`` of ``drive.log``) and with the wheel
odometry shifted to the start pose (``odom_x``, ``odom_y`` of ``telemetry.csv`` plus the start)
at the end of the drive.

Two sets of trials:

* ``rows`` (13 trials): the trials in which Nav2 reported SUCCEEDED although the ground-truth
  distance to the goal is above the 0.5 m criterion. This is the set of the first version of
  this script; its values are unchanged.
* ``all_rows`` (every trial of ``results.json.gz``, 88): the same three positions for each trial,
  plus the peak commanded turn rate (``cmd_wz``, the command that reaches the base, and
  ``nav_wz``, the controller output) over the drive. ``by_cell`` and ``by_class`` hold ranges.

The set of trials is read from ``results.json.gz``; nothing in it is edited. The raw trial
files are outside the repository: ``--raw`` names their root (default below).
Run from the repository root, in the environment of the campaign:

    python3 experiments/campaign2/odom_gap.py [--raw /mnt/d/wsl/se-mppi/campaign2] [--out-dir DIR]

It writes ``ODOM_GAP.json`` and ``ODOM_GAP.md`` into ``--out-dir``. The default is the directory
of this file, so a run inside a checkout overwrites the committed outputs; give ``--out-dir`` to
write elsewhere. The paper quotes the output verbatim (``scripts/check_paper_numbers.py``
re-reads ``ODOM_GAP.json``).
"""
import argparse
import csv
import gzip
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
LINE = re.compile(r'\[basic_navigator\]: \[\s*([\d.]+)s\] dist=([\d.]+)m amcl=\(([-\d.]+),([-\d.]+) ')
STAMP = re.compile(r'^\[INFO\] \[([\d.]+)\]')

# Classes of trial for the ranges. Stock = the controller with se_enabled false (C1, C3).
STOCK = ('C1', 'C3')


def trial_class(t):
    if t['scenario'] == 's0_open':
        return 'trap-free control (s0_open, all four configurations)'
    if t['config'] in STOCK:
        return 'stock controller in a pocket (C1, C3)'
    return 'SE-MPPI in a pocket (C2, C4)'


def end_of_drive(tdir, meta):
    """Ground truth, AMCL and shifted odometry at the last ``amcl=`` line of drive.log, and the
    peak absolute turn rates over the telemetry."""
    start = meta['start']
    drive = open(os.path.join(tdir, 'drive.log')).read().splitlines()
    last = None
    for ln in drive:
        m = LINE.search(ln)
        if m:
            s = STAMP.match(ln)
            last = (float(s.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)))
    wall, nav_dist, ax, ay = last
    tel = list(csv.DictReader(open(os.path.join(tdir, 'telemetry.csv'))))
    best = None
    peak_cmd = peak_nav = 0.0
    for r in tel:
        try:
            w = float(r['wall_s'])
            gx, gy, ox, oy = (float(r[k]) for k in ('gt_x', 'gt_y', 'odom_x', 'odom_y'))
            cw, nw = float(r['cmd_wz']), float(r['nav_wz'])
        except ValueError:
            continue
        if any(math.isnan(v) for v in (gx, gy, ox, oy)):
            continue
        if not math.isnan(cw):
            peak_cmd = max(peak_cmd, abs(cw))
        if not math.isnan(nw):
            peak_nav = max(peak_nav, abs(nw))
        if best is None or abs(w - wall) < abs(best[0] - wall):
            best = (w, gx, gy, ox + start[0], oy + start[1])
    _, gx, gy, ox, oy = best
    return {'nav_dist': nav_dist, 'amcl': (ax, ay), 'gt': (gx, gy), 'odom': (ox, oy),
            'peak_cmd_wz': peak_cmd, 'peak_nav_wz': peak_nav}


def rng(rows, key):
    v = [r[key] for r in rows]
    return {'n': len(v), 'min': min(v), 'max': max(v)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', default='/mnt/d/wsl/se-mppi/campaign2')
    ap.add_argument('--out-dir', default=HERE)
    a = ap.parse_args()
    res = json.load(gzip.open(os.path.join(HERE, 'results.json.gz')))
    rows, all_rows = [], []
    for t in res['trials']:
        tdir = os.path.join(a.raw, 'trials', t['trial_id'])
        meta = json.load(open(os.path.join(tdir, 'meta.json')))
        goal = meta['goal']
        e = end_of_drive(tdir, meta)
        gx, gy = e['gt']
        ox, oy = e['odom']
        ax, ay = e['amcl']
        d = lambda x, y: math.hypot(x - goal[0], y - goal[1])
        if t['outcome'] == 'NOT_REACHED_ABORTED' and t.get('nav2_result') == 'SUCCEEDED':
            rows.append({
                'trial': t['trial_id'],
                'final_gt_distance_m': round(t['final_goal_distance_m'], 3),
                'nav2_amcl_distance_m': round(e['nav_dist'], 2),
                'odom_distance_m': round(d(ox, oy), 3),
                'odom_minus_gt_m': round(math.hypot(ox - gx, oy - gy), 3),
                'amcl_minus_gt_m': round(math.hypot(ax - gx, ay - gy), 3),
            })
        all_rows.append({
            'trial': t['trial_id'], 'scenario': t['scenario'], 'config': t['config'],
            'class': trial_class(t), 'outcome': t['outcome'], 'nav2_result': t.get('nav2_result'),
            'odom_minus_gt_m': round(math.hypot(ox - gx, oy - gy), 3),
            'amcl_minus_gt_m': round(math.hypot(ax - gx, ay - gy), 3),
            'peak_abs_cmd_wz_rad_s': round(e['peak_cmd_wz'], 3),
            'peak_abs_nav_wz_rad_s': round(e['peak_nav_wz'], 3),
            'left_pocket_s_set': t.get('left_pocket_s') is not None,
        })
    rows.sort(key=lambda r: r['trial'])
    all_rows.sort(key=lambda r: r['trial'])
    keys = ('odom_minus_gt_m', 'amcl_minus_gt_m', 'peak_abs_cmd_wz_rad_s', 'peak_abs_nav_wz_rad_s')
    by_cell, by_class, by_class_reached = {}, {}, {}
    for r in all_rows:
        by_cell.setdefault(f"{r['scenario']}/{r['config']}", []).append(r)
        by_class.setdefault(r['class'], []).append(r)
        if r['outcome'] == 'REACHED':
            by_class_reached.setdefault(r['class'], []).append(r)
    summ = lambda groups: {g: {k: rng(v, k) for k in keys} for g, v in sorted(groups.items())}
    left = {c: {'n': len(v), 'left_pocket_s_set': sum(r['left_pocket_s_set'] for r in v)}
            for c, v in sorted(by_class.items())}
    out = {'n': len(rows), 'rows': rows, 'all_n': len(all_rows), 'all_rows': all_rows,
           'by_cell': summ(by_cell), 'by_class': summ(by_class),
           'by_class_reached': summ(by_class_reached), 'left_pocket': left}
    os.makedirs(a.out_dir, exist_ok=True)
    json.dump(out, open(os.path.join(a.out_dir, 'ODOM_GAP.json'), 'w'), indent=1, sort_keys=True)
    lines = ['# Ground truth against the localization estimate at the end of the drive', '',
             'Generated by `odom_gap.py` (post hoc, not frozen).', '',
             '## 1. The 13 trials with Nav2 SUCCEEDED and a ground-truth distance above 0.5 m', '',
             '| Trial | GT distance to goal (m) | Distance Nav2 printed from AMCL (m) | '
             'Shifted odometry distance (m) | Odometry vs GT (m) | AMCL vs GT (m) |',
             '|---|---|---|---|---|---|']
    for r in rows:
        lines.append(f"| {r['trial']} | {r['final_gt_distance_m']:.3f} | {r['nav2_amcl_distance_m']:.2f} | "
                     f"{r['odom_distance_m']:.3f} | {r['odom_minus_gt_m']:.3f} | {r['amcl_minus_gt_m']:.3f} |")
    hdr = ['| Group | Trials | Odometry vs GT (m) | AMCL vs GT (m) | Peak commanded turn rate (rad/s) |',
           '|---|---|---|---|---|']

    def table(title, groups):
        lines.extend(['', title, ''] + hdr)
        for g, v in sorted(groups.items()):
            f = lambda k: f"{min(r[k] for r in v):.3f} to {max(r[k] for r in v):.3f}"
            lines.append(f"| {g} | {len(v)} | {f('odom_minus_gt_m')} | {f('amcl_minus_gt_m')} | "
                         f"{f('peak_abs_cmd_wz_rad_s')} |")
    table(f'## 2. All {len(all_rows)} trials, by class (end of the drive)', by_class)
    table('## 3. Trials that reached the goal, by class', by_class_reached)
    table('## 4. All trials, by cell', by_cell)
    lines.extend(['', '## 5. Trials in which `left_pocket_s` is set', '',
                  '| Class | Trials | `left_pocket_s` set |', '|---|---|---|'])
    for c, v in left.items():
        lines.append(f"| {c} | {v['n']} | {v['left_pocket_s_set']} |")
    open(os.path.join(a.out_dir, 'ODOM_GAP.md'), 'w').write('\n'.join(lines) + '\n')
    print(len(rows), 'trials in the 13-trial set;', len(all_rows), 'trials in all')


if __name__ == '__main__':
    main()
