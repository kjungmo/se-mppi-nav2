#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Freeze manifest of campaign 2: sha256 of every file the pre-registration fixes.

    python3 experiments/campaign2/freeze_manifest.py --write   # at the tag, before Phase A
    python3 experiments/campaign2/freeze_manifest.py --check   # exit 1 on any difference

The driver refuses to run a counted trial (Phase A or B) unless ``--check`` passes.
The manifest covers the parameter file, the behaviour trees, the worlds and scenarios,
the plan, the driver, the recorder, the host-load module, the analyzer and its tests
(DECISION D-4b: the analysis script and its tests are frozen with the pre-registration
tag), and the campaign-1 analyzer that the new analyzer imports. ``PREREG.md`` and the
generated results are not in the manifest: a dated amendment edits the first, and the
second does not exist yet when the manifest is written.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
MANIFEST = os.path.join(HERE, 'FREEZE_MANIFEST.json')

FROZEN = [
    'experiments/sim/nav2_se_campaign2.yaml',
    'experiments/configs/ablations.yaml',
    'experiments/se_mppi_utrap/behavior_trees/nav_to_pose_default_recovery.xml',
    'experiments/se_mppi_utrap/behavior_trees/nav_to_pose_replanning_only.xml',
    'experiments/se_mppi_utrap/worlds/testbed.sdf',
    'experiments/se_mppi_utrap/urdf/amr.urdf.xacro',
    'experiments/se_mppi_utrap/launch/utrap_bench.launch.py',
    'experiments/se_mppi_utrap/maps/utrap_loc.yaml',
    'experiments/se_mppi_utrap/maps/utrap_loc.pgm',
    'experiments/barn/s1_utrap.scenario.yaml',
    'experiments/barn/s1_utrap_evidence.scenario.yaml',
    'experiments/campaign2/s0_open.scenario.yaml',
    'experiments/campaign2/s3_ctrap.scenario.yaml',
    'experiments/campaign2/worlds/s0_open.sdf',
    'experiments/campaign2/worlds/s3_ctrap.sdf',
    'experiments/campaign2/plan.py',
    'experiments/campaign2/run_campaign2.py',
    'experiments/campaign2/recorder.py',
    'experiments/campaign2/probe_escape_path.py',
    'experiments/campaign2/analyze.py',
    'experiments/campaign2/freeze_manifest.py',
    'experiments/results_pilot_rerun/hostload.py',
    'experiments/results_pilot_rerun/run_rerun.py',
    'experiments/results_pilot_rerun/analyze.py',
    'experiments/sim/smoke_drive.py',
]
FROZEN_GLOBS = [
    'experiments/campaign2/tests/*.py',
    'src/nav2_se_controller/src/*.cpp',
    'src/nav2_se_controller/include/**/*.hpp',
    'src/nav2_se_controller/CMakeLists.txt',
]


def files() -> list:
    out = list(FROZEN)
    for pat in FROZEN_GLOBS:
        out += [os.path.relpath(p, REPO) for p in glob.glob(os.path.join(REPO, pat), recursive=True)]
    return sorted(set(out))


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def compute(root: str = REPO) -> dict:
    return {rel: sha256(os.path.join(root, rel)) for rel in files()}


def compare(recorded: dict, current: dict) -> list:
    """Differences as strings: changed, missing, new."""
    diffs = []
    for k in sorted(set(recorded) | set(current)):
        if k not in current:
            diffs.append(f'missing: {k}')
        elif k not in recorded:
            diffs.append(f'new (not in the manifest): {k}')
        elif recorded[k] != current[k]:
            diffs.append(f'changed: {k}')
    return diffs


def check(manifest_path: str = MANIFEST) -> list:
    if not os.path.exists(manifest_path):
        return ['no FREEZE_MANIFEST.json: the campaign is not frozen']
    with open(manifest_path) as f:
        recorded = json.load(f)['files']
    return compare(recorded, compute())


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--write', action='store_true')
    g.add_argument('--check', action='store_true')
    args = ap.parse_args()
    if args.write:
        data = {'note': 'sha256 of the files fixed by experiments/campaign2/PREREG.md',
                'files': compute()}
        with open(MANIFEST, 'w') as f:
            json.dump(data, f, indent=1, sort_keys=True)
            f.write('\n')
        print(f'wrote {os.path.relpath(MANIFEST, REPO)} ({len(data["files"])} files)')
        return 0
    diffs = check()
    for d in diffs:
        print(d)
    if diffs:
        return 1
    print('frozen files unchanged')
    return 0


if __name__ == '__main__':
    sys.exit(main())
