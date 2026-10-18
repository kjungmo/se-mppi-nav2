"""Tests of provenance.py: what is installed is what is frozen (PREREG.md, "Provenance")."""

import json
import os
import shutil
import subprocess

import pytest

import provenance as pv
import run_campaign2 as drv

PKG_FILES = {
    'behavior_trees/nav_to_pose_default_recovery.xml': b'<root>default</root>',
    'behavior_trees/nav_to_pose_replanning_only.xml': b'<root>replan</root>',
    'worlds/testbed.sdf': b'<sdf>testbed</sdf>',
    'launch/utrap_bench.launch.py': b'# launch',
    'urdf/amr.urdf.xacro': b'<robot/>',
    'config/ros_gz_bridge.yaml': b'- topic: x',
}


def write_tree(root, files):
    for rel, data in files.items():
        path = os.path.join(root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(data)


@pytest.fixture
def pkg(tmp_path):
    repo_pkg = tmp_path / 'repo' / 'experiments' / 'se_mppi_utrap'
    share = tmp_path / 'prefix' / 'share' / 'se_mppi_utrap'
    write_tree(str(repo_pkg), PKG_FILES)
    write_tree(str(share), PKG_FILES)
    manifest = {f'experiments/se_mppi_utrap/{rel}': pv.sha256_file(str(repo_pkg / rel))
                for rel in ('behavior_trees/nav_to_pose_default_recovery.xml', 'worlds/testbed.sdf')}
    return str(share), str(repo_pkg), manifest


def test_identical_install_has_no_difference(pkg):
    share, repo_pkg, manifest = pkg
    assert pv.compare_installed(share, repo_pkg, manifest) == []


def test_a_stale_installed_behaviour_tree_is_found(pkg):
    share, repo_pkg, manifest = pkg
    with open(os.path.join(share, 'behavior_trees/nav_to_pose_replanning_only.xml'), 'wb') as f:
        f.write(b'<root>stale</root>')
    diffs = pv.compare_installed(share, repo_pkg, manifest)
    assert diffs == ['installed copy differs from the repository file: '
                     'behavior_trees/nav_to_pose_replanning_only.xml']


@pytest.mark.parametrize('rel', ['worlds/testbed.sdf', 'launch/utrap_bench.launch.py',
                                 'urdf/amr.urdf.xacro', 'config/ros_gz_bridge.yaml',
                                 'behavior_trees/nav_to_pose_default_recovery.xml'])
def test_every_installed_file_class_is_compared(pkg, rel):
    share, repo_pkg, manifest = pkg
    with open(os.path.join(share, rel), 'ab') as f:
        f.write(b' ')                                          # one byte
    assert len(pv.compare_installed(share, repo_pkg, manifest)) == 1


def test_missing_and_extra_installed_files_are_found(pkg):
    share, repo_pkg, manifest = pkg
    os.remove(os.path.join(share, 'worlds/testbed.sdf'))
    write_tree(share, {'worlds/other.sdf': b'x'})
    diffs = pv.compare_installed(share, repo_pkg, manifest)
    assert 'not installed: worlds/testbed.sdf' in diffs
    assert 'installed but not in the repository: worlds/other.sdf' in diffs


def test_installed_equal_to_the_repository_but_not_to_the_frozen_hash_is_found(pkg):
    share, repo_pkg, manifest = pkg
    manifest['experiments/se_mppi_utrap/worlds/testbed.sdf'] = '0' * 64
    assert pv.compare_installed(share, repo_pkg, manifest) == [
        'installed copy differs from the frozen hash: worlds/testbed.sdf']


def test_no_installed_share_directory(tmp_path, pkg):
    _, repo_pkg, manifest = pkg
    diffs = pv.compare_installed(str(tmp_path / 'nope'), repo_pkg, manifest)
    assert any('no installed share directory' in d for d in diffs)


def test_library_hashes_and_the_build_stamp(tmp_path):
    prefix = tmp_path / 'p'
    (prefix / 'lib').mkdir(parents=True)
    for name in pv.LIBRARIES:
        (prefix / 'lib' / name).write_bytes(name.encode())
    libs = pv.library_hashes(str(prefix))
    assert all(libs.values()) and libs['libescape_critic.so'] == pv.sha256_file(
        str(prefix / 'lib' / 'libescape_critic.so'))
    assert pv.read_build_stamp(str(prefix)) is None
    stamp_dir = prefix / 'share' / 'nav2_se_controller'
    stamp_dir.mkdir(parents=True)
    (stamp_dir / 'build_stamp.txt').write_text('git_describe=campaign2-controller\ngit_head=abc\n')
    stamp = pv.read_build_stamp(str(prefix))
    assert stamp == {'git_describe': 'campaign2-controller', 'git_head': 'abc'}
    assert pv.check_build_stamp(stamp, 'abc', libs) == []
    os.remove(prefix / 'lib' / 'libse_mppi_core.so')
    assert pv.check_build_stamp(stamp, 'abc', pv.library_hashes(str(prefix))) == [
        'installed library missing: libse_mppi_core.so']


@pytest.mark.parametrize('stamp,tag_commit,needle', [
    (None, 'abc', 'no build stamp'),
    ({'git_describe': 'campaign2-controller-dirty', 'git_head': 'abc'}, 'abc', 'not '),
    ({'git_describe': 'campaign2-controller-3-gdeadbee', 'git_head': 'abc'}, 'abc', 'not '),
    ({'git_describe': 'unknown', 'git_head': 'abc'}, 'abc', 'not '),
    ({'git_describe': 'campaign2-controller', 'git_head': 'def'}, 'abc', 'is not the tag commit'),
    ({'git_describe': 'campaign2-controller', 'git_head': 'abc'}, None, 'does not exist'),
])
def test_a_bad_build_stamp_is_refused(stamp, tag_commit, needle):
    libs = {n: 'x' for n in pv.LIBRARIES}
    diffs = pv.check_build_stamp(stamp, tag_commit, libs)
    assert diffs and any(needle in d for d in diffs), diffs


# --------------------------------------------------------------------------- #
# git state
# --------------------------------------------------------------------------- #
def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), '-c', 'user.name=t', '-c', 'user.email=t@example.org',
                           *args], capture_output=True, text=True, check=True).stdout.strip()


@pytest.fixture
def repo(tmp_path):
    r = tmp_path / 'r'
    r.mkdir()
    git(r, 'init', '-q')
    (r / 'plan.py').write_text('a\n')
    (r / 'notes.md').write_text('n\n')
    os.makedirs(r / 'experiments' / 'campaign2' / 'logs')
    (r / 'experiments' / 'campaign2' / 'RESULTS.md').write_text('r\n')
    (r / 'experiments' / 'campaign2' / 'logs' / 'x.log.gz').write_text('l\n')
    git(r, 'add', '-A')
    git(r, 'commit', '-q', '-m', 'frozen')
    return r


def test_no_tag_means_not_frozen(repo):
    assert pv.check_git(str(repo), ['plan.py']) == [
        'the tag campaign2-controller does not exist: the controller is not frozen']
    assert pv.tag_commit(str(repo)) is None


def test_at_the_tag_and_clean(repo):
    git(repo, 'tag', pv.TAG)
    assert pv.check_git(str(repo), ['plan.py']) == []
    assert pv.git_describe(str(repo)) == pv.TAG


def test_a_modified_tracked_file_is_refused_but_generated_outputs_are_not(repo):
    git(repo, 'tag', pv.TAG)
    (repo / 'notes.md').write_text('changed\n')
    assert pv.check_git(str(repo), ['plan.py']) == ['uncommitted change to a tracked file: notes.md']
    git(repo, 'checkout', '--', 'notes.md')
    (repo / 'experiments' / 'campaign2' / 'RESULTS.md').write_text('regenerated\n')
    (repo / 'experiments' / 'campaign2' / 'logs' / 'x.log.gz').write_text('regenerated\n')
    assert pv.check_git(str(repo), ['plan.py']) == []
    (repo / 'untracked.txt').write_text('u\n')                 # untracked files do not count
    assert pv.check_git(str(repo), ['plan.py']) == []


def test_commits_after_the_tag_are_fine_unless_they_touch_a_frozen_file(repo):
    git(repo, 'tag', pv.TAG)
    (repo / 'notes.md').write_text('later\n')
    git(repo, 'commit', '-q', '-am', 'notes')
    assert pv.check_git(str(repo), ['plan.py']) == []
    (repo / 'plan.py').write_text('b\n')
    git(repo, 'commit', '-q', '-am', 'plan')
    assert pv.check_git(str(repo), ['plan.py']) == ['frozen file changed since the tag: plan.py']


def test_head_must_contain_the_tag(repo):
    git(repo, 'tag', pv.TAG)
    git(repo, 'checkout', '-q', '--orphan', 'other')
    git(repo, 'commit', '-q', '-m', 'unrelated', '--allow-empty')
    assert pv.check_git(str(repo), []) == [f'HEAD does not contain the tag {pv.TAG}']


# --------------------------------------------------------------------------- #
# identity and foreign trials
# --------------------------------------------------------------------------- #
def test_identity_carries_manifest_hash_and_tag_commit(repo, tmp_path):
    manifest = tmp_path / 'FREEZE_MANIFEST.json'
    assert pv.identity(str(repo), str(manifest)) == {'freeze_manifest_sha256': None,
                                                     'controller_tag_commit': None}
    manifest.write_text('{"files": {}}')
    git(repo, 'tag', pv.TAG)
    ident = pv.identity(str(repo), str(manifest))
    assert ident['freeze_manifest_sha256'] == pv.sha256_file(str(manifest))
    assert ident['controller_tag_commit'] == git(repo, 'rev-parse', 'HEAD')


def test_foreign_reason_names_the_key_that_differs():
    ident = {'freeze_manifest_sha256': 'aa', 'controller_tag_commit': 'cc'}
    assert pv.foreign_reason(dict(ident), ident) is None
    assert pv.foreign_reason({}, None) is None
    assert 'freeze_manifest_sha256' in pv.foreign_reason(dict(ident, freeze_manifest_sha256='bb'), ident)
    assert 'controller_tag_commit' in pv.foreign_reason(dict(ident, controller_tag_commit='dd'), ident)
    assert pv.foreign_reason({}, ident) is not None             # an old trial without identity


def test_verify_installed_collects_every_kind_of_difference(tmp_path, pkg):
    share, repo_pkg, manifest = pkg
    root = tmp_path / 'repo2'
    shutil.copytree(os.path.dirname(os.path.dirname(repo_pkg)), root)
    git(root, 'init', '-q')
    git(root, 'add', '-A')
    git(root, 'commit', '-q', '-m', 'c')
    git(root, 'tag', pv.TAG)
    head = git(root, 'rev-parse', 'HEAD')
    ctrl = tmp_path / 'ctrl'
    (ctrl / 'lib').mkdir(parents=True)
    for n in pv.LIBRARIES:
        (ctrl / 'lib' / n).write_bytes(b'x')
    stamp_dir = ctrl / 'share' / 'nav2_se_controller'
    stamp_dir.mkdir(parents=True)
    (stamp_dir / 'build_stamp.txt').write_text(f'git_describe={pv.TAG}\ngit_head={head}\n')
    testbed_prefix = os.path.dirname(os.path.dirname(share))
    diffs, record = pv.verify_installed(str(root), str(ctrl), testbed_prefix, manifest, [])
    assert diffs == []
    assert record['build_stamp']['git_head'] == head and record['git_describe'] == pv.TAG
    assert set(record['installed_libraries_sha256']) == set(pv.LIBRARIES)
    # a stale library stamp and a stale installed tree are both reported
    (stamp_dir / 'build_stamp.txt').write_text('git_describe=campaign2-controller-dirty\ngit_head=x\n')
    with open(os.path.join(share, 'worlds/testbed.sdf'), 'ab') as f:
        f.write(b' ')
    diffs, _ = pv.verify_installed(str(root), str(ctrl), testbed_prefix, manifest, [])
    assert len(diffs) >= 3
    assert any('build stamp' in d for d in diffs) and any('installed copy differs' in d for d in diffs)
    # a dirty tracked file of the repository is reported too (the git check is part of the verdict)
    (stamp_dir / 'build_stamp.txt').write_text(f'git_describe={pv.TAG}\ngit_head={head}\n')
    with open(os.path.join(share, 'worlds/testbed.sdf'), 'wb') as f:
        f.write(PKG_FILES['worlds/testbed.sdf'])
    diffs, _ = pv.verify_installed(str(root), str(ctrl), testbed_prefix, manifest, [])
    assert diffs == []
    (root / 'experiments' / 'se_mppi_utrap' / 'launch' / 'utrap_bench.launch.py').write_bytes(b'# edited')
    diffs, _ = pv.verify_installed(str(root), str(ctrl), testbed_prefix, manifest, [])
    assert any('uncommitted change to a tracked file' in d for d in diffs), diffs


def test_the_cmake_stamp_script_writes_describe_and_head(repo, tmp_path):
    cmake = shutil.which('cmake')
    if cmake is None:
        pytest.skip('cmake is not on PATH')
    git(repo, 'tag', pv.TAG)
    script = os.path.join(os.path.dirname(pv.__file__), '..', '..', 'src',
                          'nav2_se_controller', 'cmake', 'write_build_stamp.cmake')
    out = tmp_path / 'stamp.txt'
    subprocess.run([cmake, f'-DSRC={repo}', f'-DOUT={out}', '-P', os.path.normpath(script)], check=True)
    assert out.read_text() == f'git_describe={pv.TAG}\ngit_head={git(repo, "rev-parse", "HEAD")}\n'
    (repo / 'notes.md').write_text('dirty\n')
    subprocess.run([cmake, f'-DSRC={repo}', f'-DOUT={out}', '-P', os.path.normpath(script)], check=True)
    assert out.read_text().splitlines()[0] == f'git_describe={pv.TAG}-dirty'
    subprocess.run([cmake, f'-DSRC={tmp_path}', f'-DOUT={out}', '-P', os.path.normpath(script)], check=True)
    assert out.read_text().startswith('git_describe=unknown\ngit_head=unknown')


def test_the_driver_records_and_checks_identity_in_existing_state(tmp_path):
    ident = {'freeze_manifest_sha256': 'aa', 'controller_tag_commit': 'cc'}
    d = tmp_path / 't'
    d.mkdir()
    (d / 'meta.json').write_text(json.dumps(dict(ident, complete=True, active=True)))
    assert drv.existing_state(str(d), ident) == 'complete_valid'
    (d / 'meta.json').write_text(json.dumps(dict(ident, controller_tag_commit='zz', complete=True)))
    assert drv.existing_state(str(d), ident) == 'foreign'
    assert drv.existing_state(str(d)) == 'complete_valid'       # no identity given (shakedown)
