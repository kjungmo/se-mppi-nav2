# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Provenance of what a counted trial runs (PREREG.md, "Frozen files" and "Provenance").

The freeze manifest hashes repository sources. At run time the controller libraries come from
the colcon install prefix and the behaviour trees, worlds and launch files of ``se_mppi_utrap``
come from its installed share directory, which holds copies. A stale build would pass every
freeze check. This module closes that gap:

* ``compare_installed``  every installed file of ``se_mppi_utrap`` equals the repository file
                         (and the manifest hash when the file is frozen);
* ``library_hashes``     sha256 of the installed ``nav2_se_controller`` libraries;
* ``read_build_stamp`` / ``check_build_stamp``
                         the stamp that CMake writes at every build (``git describe`` and
                         ``git rev-parse HEAD`` of the tree that was built); it must name the
                         tag ``campaign2-controller`` exactly, at the tag commit;
* ``check_git``          the repository is at the tag, or later with no change to a frozen
                         file, and has no uncommitted change to a tracked file other than the
                         generated analysis outputs;
* ``identity``           what a trial directory must carry to belong to this attempt.

Pure Python (standard library only), no ROS: every function takes paths and strings.
"""

from __future__ import annotations

import glob
import hashlib
import os
import subprocess

TAG = 'campaign2-controller'
# Subdirectories of experiments/se_mppi_utrap that its CMakeLists installs.
INSTALLED_SUBDIRS = ('worlds', 'urdf', 'launch', 'config', 'behavior_trees')
LIBRARIES = ('libse_mppi_core.so', 'libescape_critic.so', 'libmppi_rollout_check.so',
             'libsafe_escape_controller.so')
# Tracked files that analyze.py regenerates: a change to them does not make the tree dirty.
GENERATED = ('experiments/campaign2/RESULTS.md', 'experiments/campaign2/results.json.gz',
             'experiments/campaign2/logs/')


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _files_under(root: str) -> list:
    out = []
    for dirpath, _dirs, names in os.walk(root):
        for n in names:
            out.append(os.path.relpath(os.path.join(dirpath, n), root))
    return sorted(out)


def compare_installed(share_dir: str, repo_pkg_dir: str, manifest_files: dict | None = None,
                      repo_rel: str = 'experiments/se_mppi_utrap') -> list:
    """Differences between the installed share directory and the repository package.

    ``share_dir``: ``<prefix>/share/se_mppi_utrap``. ``repo_pkg_dir``: ``experiments/se_mppi_utrap``.
    ``manifest_files``: the ``files`` dict of FREEZE_MANIFEST.json (relative to the repository
    root, with ``repo_rel`` as the prefix of the package). Every repository file under an
    installed subdirectory must be installed with the same bytes, and nothing else may be
    installed there. A frozen file must also match its manifest hash.
    """
    diffs = []
    manifest_files = manifest_files or {}
    for sub in INSTALLED_SUBDIRS:
        rdir = os.path.join(repo_pkg_dir, sub)
        idir = os.path.join(share_dir, sub)
        repo_names = set(_files_under(rdir)) if os.path.isdir(rdir) else set()
        inst_names = set(_files_under(idir)) if os.path.isdir(idir) else set()
        for n in sorted(repo_names | inst_names):
            rel = f'{sub}/{n}'
            if n not in inst_names:
                diffs.append(f'not installed: {rel}')
                continue
            if n not in repo_names:
                diffs.append(f'installed but not in the repository: {rel}')
                continue
            inst_sha = sha256_file(os.path.join(idir, n))
            if inst_sha != sha256_file(os.path.join(rdir, n)):
                diffs.append(f'installed copy differs from the repository file: {rel}')
                continue
            frozen = manifest_files.get(f'{repo_rel}/{rel}')
            if frozen is not None and frozen != inst_sha:
                diffs.append(f'installed copy differs from the frozen hash: {rel}')
    if not os.path.isdir(share_dir):
        diffs.append(f'no installed share directory: {share_dir}')
    return diffs


def library_hashes(prefix: str) -> dict:
    """sha256 of the installed controller libraries; a missing library is None."""
    out = {}
    for name in LIBRARIES:
        found = glob.glob(os.path.join(prefix, 'lib', name))
        out[name] = sha256_file(found[0]) if found else None
    return out


def read_build_stamp(prefix: str) -> dict | None:
    """``{'git_describe': ..., 'git_head': ...}`` from ``share/nav2_se_controller/build_stamp.txt``."""
    path = os.path.join(prefix, 'share', 'nav2_se_controller', 'build_stamp.txt')
    if not os.path.exists(path):
        return None
    out = {}
    with open(path) as f:
        for ln in f:
            if '=' in ln:
                k, v = ln.strip().split('=', 1)
                out[k] = v
    return out


def check_build_stamp(stamp: dict | None, tag_commit: str | None, libs: dict) -> list:
    diffs = []
    if stamp is None:
        return ['no build stamp (share/nav2_se_controller/build_stamp.txt): the installed '
                'controller was not built by the CMakeLists of the frozen tag']
    if stamp.get('git_describe') != TAG:
        diffs.append(f'build stamp git describe is {stamp.get("git_describe")!r}, not {TAG!r} '
                     '(dirty, or built from another commit)')
    if tag_commit is None:
        diffs.append(f'the tag {TAG} does not exist')
    elif stamp.get('git_head') != tag_commit:
        diffs.append(f'build stamp commit {stamp.get("git_head")} is not the tag commit {tag_commit}')
    for name, sha in libs.items():
        if sha is None:
            diffs.append(f'installed library missing: {name}')
    return diffs


def _git(repo: str, *args: str) -> tuple:
    r = subprocess.run(['git', '-C', repo, *args], capture_output=True, text=True)
    return r.returncode, r.stdout.rstrip('\n')


def tag_commit(repo: str) -> str | None:
    rc, out = _git(repo, 'rev-parse', '--verify', '--quiet', f'{TAG}^{{commit}}')
    return out if rc == 0 and out else None


def git_describe(repo: str) -> str:
    return _git(repo, 'describe', '--tags', '--always', '--dirty')[1]


def check_git(repo: str, frozen_files: list) -> list:
    """The repository is at the tag (or later without a change to a frozen file) and clean."""
    diffs = []
    tc = tag_commit(repo)
    if tc is None:
        return [f'the tag {TAG} does not exist: the controller is not frozen']
    rc, _ = _git(repo, 'merge-base', '--is-ancestor', tc, 'HEAD')
    if rc != 0:
        diffs.append(f'HEAD does not contain the tag {TAG}')
        return diffs
    rc, changed = _git(repo, 'diff', '--name-only', tc, 'HEAD')
    frozen = set(frozen_files)
    for rel in changed.splitlines():
        if rel in frozen:
            diffs.append(f'frozen file changed since the tag: {rel}')
    rc, status = _git(repo, 'status', '--porcelain', '--untracked-files=no')
    for ln in status.splitlines():
        rel = ln[3:].strip()
        if any(rel == g or (g.endswith('/') and rel.startswith(g)) for g in GENERATED):
            continue
        diffs.append(f'uncommitted change to a tracked file: {rel}')
    return diffs


def identity(repo: str, manifest_path: str) -> dict:
    """What a trial carries in meta.json to belong to this attempt."""
    return {
        'freeze_manifest_sha256': sha256_file(manifest_path) if os.path.exists(manifest_path) else None,
        'controller_tag_commit': tag_commit(repo),
    }


# Identities that PREREG.md amendments accept beside the current one. Amendment 3 (2026-10-22)
# moved the tag after trials of attempt 1 were recorded; those trials carry this pair. Only
# this exact pair is accepted. Any other manifest hash or tag commit is still foreign.
AMENDED_PRIOR_IDENTITIES = (
    {'freeze_manifest_sha256': 'd96cc1c24798536d7e64bb43eef1f15436d77086c7fd9bbb7667fdadfef17f00',
     'controller_tag_commit': '012f4f1db11de8ce607854578cb99ab932bf3969'},
)


# Amendment 3 accepts the prior pair only for trials that started before the amendment was made
# (the commit time of the amended tag, 2026-10-22 02:34:33 KST). A later trial that still carries
# the prior pair (a stale checkout or build) is foreign.
AMENDMENT3_CUTOFF_WALL = 1792604073


def is_amended_prior_identity(meta: dict) -> bool:
    start = meta.get('wall_start')
    if not isinstance(start, (int, float)) or isinstance(start, bool) or start >= AMENDMENT3_CUTOFF_WALL:
        return False
    return any(all(meta.get(k) == v for k, v in prior.items()) for prior in AMENDED_PRIOR_IDENTITIES)


def foreign_reason(meta: dict, ident: dict | None) -> str | None:
    """Why a trial's meta.json does not belong to the attempt with identity ``ident``."""
    if ident is None or is_amended_prior_identity(meta):
        return None
    for k, v in ident.items():
        if meta.get(k) != v:
            return f'{k} is {meta.get(k)!r}, this attempt has {v!r}'
    return None


def verify_installed(repo: str, controller_prefix: str, testbed_prefix: str,
                     manifest_files: dict, frozen_files: list) -> tuple:
    """(diffs, record). Run before every counted trial; any diff refuses the trial."""
    share = os.path.join(testbed_prefix, 'share', 'se_mppi_utrap')
    diffs = compare_installed(share, os.path.join(repo, 'experiments', 'se_mppi_utrap'),
                              manifest_files)
    libs = library_hashes(controller_prefix)
    stamp = read_build_stamp(controller_prefix)
    diffs += check_build_stamp(stamp, tag_commit(repo), libs)
    diffs += check_git(repo, frozen_files)
    record = {'installed_libraries_sha256': libs, 'build_stamp': stamp,
              'git_describe': git_describe(repo), 'git_head': _git(repo, 'rev-parse', 'HEAD')[1]}
    return diffs, record
