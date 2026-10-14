#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Offline replay: what would the entrapment detector have seen? (ROOT_CAUSE.md)

Feeds the recorded ground-truth positions of a trial (10 Hz, the controller
rate of parameter set P) through a Python transcription of
``EntrapmentDetector::update`` / ``rebase``
(src/nav2_se_controller/include/nav2_se_controller/entrapment_detector.hpp) and
of the progress bookkeeping of ``SafeEscapeController::computeVelocityCommands``
(furthest index, re-anchoring on a replanned path to the same goal).

Assumptions, stated because this is a reconstruction and not a log:
* the detector runs on EVERY sample, including those in which MPPI's optimizer
  threw (in the live controller it does not run then);
* the global path is a straight line to the goal with 0.0252 m spacing (the
  planner's map does not contain the U; the live plan had 238 poses over 6 m);
* mode ``replan``: a new path from the robot's current position arrives every
  1.0 s (the behaviour tree's rate), which re-anchors the index as the code
  does; mode ``fixed``: one path from the start pose, never replaced.

Prints, per trial and mode, the times (s after the goal was sent) at which the
replayed detector would have switched on and off.

    python3 experiments/results_pilot_rerun/replay_detector.py [trial_id ...]
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze  # noqa: E402

W = 30            # se_progress_stall_window (cycles)
EPS_GOAL = 0.5    # se_goal_reached_tolerance (m)
SPACING = 0.0252  # m between path poses
REPLAN_S = 1.0


class Detector:
    """Transcription of EntrapmentDetector (update, rebase, reset)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.seen, self.entrapped, self.stall, self.last = False, False, 0, 0

    def rebase(self, idx):
        self.seen, self.last = True, idx

    def update(self, idx):
        if not self.seen:
            self.seen, self.last = True, idx
            return self.entrapped
        if idx > self.last:
            self.last, self.stall, self.entrapped = idx, 0, False
        elif not self.entrapped:
            self.stall += 1
            if self.stall >= W:
                self.entrapped = True
        return self.entrapped


def nearest_index(origin, goal, p):
    """Index of the path pose nearest to ``p`` on the straight path origin->goal."""
    dx, dy = goal[0] - origin[0], goal[1] - origin[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0
    s = ((p[0] - origin[0]) * dx + (p[1] - origin[1]) * dy) / length
    return int(round(min(max(s, 0.0), length) / SPACING))


def replay(tdir, mode):
    with open(os.path.join(tdir, 'meta.json')) as f:
        meta = json.load(f)
    drive = analyze.drive_stats(analyze.read_text(os.path.join(tdir, 'drive.log')), 0.0)
    t0, t1 = drive['t0'], drive['t1']
    rows = [r for r in analyze.read_telemetry(tdir)
            if not math.isnan(r['gt_x']) and t0 <= r['wall_s'] <= t1]
    goal = meta['goal'][:2]
    det, furthest, prev = Detector(), 0, False
    origin, last_plan = (rows[0]['gt_x'], rows[0]['gt_y']), rows[0]['wall_s']
    events = []
    for r in rows:
        p = (r['gt_x'], r['gt_y'])
        rebase = False
        if mode == 'replan' and r['wall_s'] - last_plan >= REPLAN_S:
            origin, last_plan, rebase = p, r['wall_s'], True
        idx = nearest_index(origin, goal, p)
        if rebase:
            furthest = idx
            det.rebase(idx)
        furthest = max(furthest, idx)
        ent = det.update(furthest)
        if math.hypot(goal[0] - p[0], goal[1] - p[1]) <= EPS_GOAL:
            ent = False
            det.reset()
            furthest = 0
        if ent != prev:
            events.append(('ON' if ent else 'OFF', round(r['wall_s'] - t0, 1)))
            prev = ent
    return events


def main():
    ids = sys.argv[1:] or [os.path.basename(d) for d in
                           sorted(glob.glob(os.path.join(HERE, 'trials', '*')))
                           if '.' not in os.path.basename(d)]
    for tid in ids:
        tdir = os.path.join(HERE, 'trials', tid)
        if not os.path.exists(os.path.join(tdir, 'drive.log')):
            continue
        out = []
        for mode in ('replan', 'fixed'):
            ev = replay(tdir, mode)
            out.append(f'{mode}: ' + (', '.join(f'{k}@{t}' for k, t in ev[:6]) or 'never')
                       + (f' (+{len(ev) - 6} more)' if len(ev) > 6 else ''))
        print(f'{tid}: ' + ' | '.join(out))


if __name__ == '__main__':
    main()
