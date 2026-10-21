"""Import-shadowing regression of the campaign-2 driver (PREREG.md, amendment 2).

``main()`` imports ``run_rerun`` of campaign 1, which puts ``results_pilot_rerun`` first on
``sys.path``. That directory has an ``analyze.py`` and a ``recorder.py`` of its own. A later
bare ``import analyze`` then returns the campaign-1 module, which has no ``registered_records``.
The first launch of Phase A died of this before any trial started.

The unit tests import ``campaign2/analyze`` directly and never see it, so each check here runs in
a fresh child interpreter that mutates ``sys.path`` the way ``main()`` does.
"""

import ast
import os
import subprocess
import sys

import plan

HERE = plan.HERE
DRIVER = os.path.join(HERE, 'run_campaign2.py')
PILOT_DIR = os.path.normpath(os.path.join(HERE, '..', 'results_pilot_rerun'))

# The child loads the driver source under the driver's real path (so HERE and REPO are right),
# optionally with the pre-fix lookup put back, then imports run_rerun as main() does.
PROBE = r'''
import os, sys, types
path, root, mutant = sys.argv[1], sys.argv[2], sys.argv[3] == 'mutant'
src = open(path).read()
if mutant:
    assert 'an = c2_analyze()' in src
    src = src.replace('an = c2_analyze()', 'import analyze as an')
drv = types.ModuleType('drv_under_test')
drv.__file__ = path
sys.modules['drv_under_test'] = drv
exec(compile(src, path, 'exec'), drv.__dict__)
import run_rerun as rr                         # as main() does: sys.path is mutated here
pilot = os.path.normpath(os.path.join(drv.HERE, '..', 'results_pilot_rerun'))
assert os.path.normpath(sys.path[0]) == pilot, sys.path[:3]
import plan
os.makedirs(os.path.join(root, 'trials'), exist_ok=True)
cells = sorted({t.cell for t in plan.plan('all')})
assert drv.check_stops(root, cells) == []
assert drv.phase_gate_refusal(root, 'A') is None
assert drv.phase_gate_refusal(root, 'B')                       # block B waits for Phase A
t_a = plan.plan('A')[0]
t_b = plan.plan('B')[0]
assert drv.trial_refusal(root, os.path.join(root, 'trials'), t_a, None) is None
assert drv.trial_refusal(root, os.path.join(root, 'trials'), t_b, None)
if not mutant:
    assert sys.modules['c2_analyze'].__file__ == os.path.join(drv.HERE, 'analyze.py')
print('PROBE OK')
'''


def probe(tmp_path, mutant=False):
    return subprocess.run([sys.executable, '-c', PROBE, DRIVER, str(tmp_path),
                           'mutant' if mutant else 'real'], capture_output=True, text=True,
                          cwd=str(tmp_path))


def test_driver_functions_after_run_rerun_is_imported(tmp_path):
    r = probe(tmp_path)
    assert r.returncode == 0, r.stderr
    assert 'PROBE OK' in r.stdout


def test_the_probe_catches_the_old_bare_import(tmp_path):
    """With the bare ``import analyze as an`` put back, the probe must fail the way Phase A did."""
    r = probe(tmp_path, mutant=True)
    assert r.returncode != 0
    assert 'registered_records' in r.stderr


def _module_names(directory):
    return {f[:-3] for f in os.listdir(directory) if f.endswith('.py')}


def test_driver_has_no_bare_import_of_a_name_that_both_directories_define():
    clash = _module_names(HERE) & _module_names(PILOT_DIR)
    assert {'analyze', 'recorder'} <= clash                     # the known clashes
    tree = ast.parse(open(DRIVER).read())
    bare = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            bare += [a.name.split('.')[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            bare.append(node.module.split('.')[0])
    assert not (set(bare) & clash), sorted(set(bare) & clash)


def test_every_name_the_driver_imports_bare_has_one_definition():
    """plan, provenance, freeze_manifest, hostload and run_rerun each exist in one directory only."""
    for name in ('plan', 'provenance', 'freeze_manifest', 'hostload', 'run_rerun'):
        homes = [d for d in (HERE, PILOT_DIR) if os.path.exists(os.path.join(d, name + '.py'))]
        assert len(homes) == 1, (name, homes)


MAIN_PROBE = r'''
import os, sys, types
path, root = sys.argv[1], sys.argv[2]
src = open(path).read()
drv = types.ModuleType('drv_under_test')
drv.__file__ = path
sys.modules['drv_under_test'] = drv
exec(compile(src, path, 'exec'), drv.__dict__)
import run_rerun as rr
import freeze_manifest, provenance
rr.rl.ros_available = lambda: True
freeze_manifest.check = lambda: []
provenance.identity = lambda repo, manifest: {'freeze_manifest_sha256': 'x', 'controller_tag_commit': 'y'}
provenance.foreign_reason = lambda meta, ident: None
provenance.verify_installed = lambda *a, **k: ([], {'fake': True})
drv.environment_info = lambda rr_: {'check_mppi_rollout_exit': 0, 'tracked_inputs_modified': '',
                                    'controller_prefix': '/x', 'testbed_prefix': '/y'}
seen = []
def fake_run_trial(rr_, config_mod, suite, base_tree, scenario, trial, out_root, env_info, probe,
                   ident=None, prov=None):
    # The real build_params runs: the C1-C4 parameter overlays and the safety refusal.
    params = drv.build_params(rr_, config_mod, suite, base_tree, scenario, trial.config)
    assert params
    seen.append(trial.trial_id)
    return {'trial_id': trial.trial_id, 'host_load': {'valid': True}}
drv.run_trial = fake_run_trial
drv.time.sleep = lambda s: None
sys.argv = ['run_campaign2.py', '--block', 'A', '--raw', root]
rc = drv.main()
print('RC', rc, 'TRIALS', len(seen))
'''


def test_main_block_a_runs_through_startup_and_the_loop_without_a_trial(tmp_path):
    r = subprocess.run([sys.executable, '-c', MAIN_PROBE, DRIVER, str(tmp_path)],
                       capture_output=True, text=True, cwd=str(tmp_path))
    assert r.returncode == 0, r.stderr
    assert 'RC 0 TRIALS 9' in r.stdout
    assert 'block finished' in r.stdout
