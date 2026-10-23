#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Freeze manifest of campaign 2: sha256 of every file the pre-registration fixes.

    python3 experiments/campaign2/freeze_manifest.py --write   # at the tag, before Phase A
    python3 experiments/campaign2/freeze_manifest.py --check   # exit 1 on any difference
    python3 experiments/campaign2/freeze_manifest.py --check --ref campaign2-controller

``--check`` compares the manifest with the working tree. The manifest documents the frozen state
at the tag ``campaign2-controller`` (commit 6a50723), so ``--check`` passes on a checkout of that
tag and the driver may run counted trials only there. After the campaign (2026-10-22) the branch
received three edits to frozen files that the manifest does not describe: the Humble compile fix
in ``safe_escape_controller.hpp`` (commit 06da6ce), ``--digits`` in
``results_pilot_rerun/analyze.py`` (commit 010f46e) and ``--ref`` in this tool (commit 2d6b51f).
None changes controller behaviour or a campaign input; on such a checkout ``--check`` correctly
reports ``changed:`` for them. ``--check --ref REF`` compares the manifest
with the files as committed at REF (a tag or a commit) instead of the working tree, so the
claim "the tag holds the frozen state" stays checkable from any later checkout that has the tag.

The driver refuses to run a counted trial (Phase A or B) unless ``--check`` passes.
The manifest covers the parameter file, the behaviour trees, the worlds and scenarios,
the plan, the driver, the provenance module, the recorder, the host-load module, the analyzer and its tests
(DECISION D-4b: the analysis script and its tests are frozen with the pre-registration
tag), and the campaign-1 analyzer that the new analyzer imports. ``PREREG.md`` and the
generated results are not in the manifest: a dated amendment edits the first, and the
second does not exist yet when the manifest is written.
"""

from __future__ import annotations

import argparse
import fnmatch
import glob
import hashlib
import json
import os
import subprocess
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
    'experiments/campaign2/s4_ltrap.scenario.yaml',
    'experiments/campaign2/worlds/s0_open.sdf',
    'experiments/campaign2/worlds/s3_ctrap.sdf',
    'experiments/campaign2/worlds/s4_ltrap.sdf',
    'experiments/campaign2/plan.py',
    'experiments/campaign2/run_campaign2.py',
    'experiments/campaign2/recorder.py',
    'experiments/campaign2/probe_escape_path.py',
    'experiments/campaign2/analyze.py',
    'experiments/campaign2/provenance.py',
    'experiments/se_mppi_utrap/CMakeLists.txt',
    'experiments/campaign2/freeze_manifest.py',
    'experiments/results_pilot_rerun/hostload.py',
    'experiments/results_pilot_rerun/run_rerun.py',
    'experiments/results_pilot_rerun/analyze.py',
    'experiments/sim/smoke_drive.py',
]
FROZEN_GLOBS = [
    'experiments/campaign2/tests/*.py',
    'experiments/campaign2/tests/fixtures/**/*',
    'src/nav2_se_controller/src/*.cpp',
    'src/nav2_se_controller/include/**/*.hpp',
    'src/nav2_se_controller/CMakeLists.txt',
    'src/nav2_se_controller/cmake/*.cmake',
]


def files() -> list:
    out = list(FROZEN)
    for pat in FROZEN_GLOBS:
        out += [os.path.relpath(p, REPO) for p in glob.glob(os.path.join(REPO, pat), recursive=True)
                if os.path.isfile(p)]
    return sorted(set(out))


def _git(*args: str) -> bytes:
    return subprocess.run(['git', *args], cwd=REPO, check=True, capture_output=True).stdout


def files_at(ref: str) -> list:
    """The frozen file list evaluated on the tree committed at ``ref`` (same rules as files())."""
    tree = _git('ls-tree', '-r', '--name-only', ref).decode().splitlines()
    out = [f for f in FROZEN if f in tree]
    for pat in FROZEN_GLOBS:
        # glob's recursive '**/' also matches zero directories; fnmatch's '*' crosses '/'.
        pats = {pat, pat.replace('**/', '')}
        out += [t for t in tree if any(fnmatch.fnmatchcase(t, q) for q in pats)]
    return sorted(set(out))


def compute_at(ref: str) -> dict:
    """sha256 of the frozen files as committed at ``ref``; a frozen file absent there is left out."""
    h = {}
    for rel in files_at(ref):
        h[rel] = hashlib.sha256(_git('show', f'{ref}:{rel}')).hexdigest()
    return h


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


def check(manifest_path: str = MANIFEST, ref: str = None) -> list:
    """Differences between the manifest and the working tree, or the tree committed at ``ref``."""
    if not os.path.exists(manifest_path):
        return ['no FREEZE_MANIFEST.json: the campaign is not frozen']
    with open(manifest_path) as f:
        recorded = json.load(f)['files']
    return compare(recorded, compute_at(ref) if ref else compute())


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--write', action='store_true')
    g.add_argument('--check', action='store_true')
    ap.add_argument('--ref', help='with --check: compare with the files committed at this tag or commit')
    args = ap.parse_args()
    if args.ref and not args.check:
        ap.error('--ref goes with --check')
    if args.write:
        data = {'note': 'sha256 of the files fixed by experiments/campaign2/PREREG.md',
                'files': compute()}
        with open(MANIFEST, 'w') as f:
            json.dump(data, f, indent=1, sort_keys=True)
            f.write('\n')
        print(f'wrote {os.path.relpath(MANIFEST, REPO)} ({len(data["files"])} files)')
        return 0
    diffs = check(ref=args.ref)
    for d in diffs:
        print(d)
    if diffs:
        return 1
    print(f'frozen files unchanged{" at " + args.ref if args.ref else ""}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
