# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Offline tests for the live RosLauncher (design §9, H-4).

Only the pure helpers are exercised — argv/params/env construction, the active
log matcher, the drive-telemetry parser, and the fail-fast guard. The imperative
shell (ros2 launch + Gazebo) needs the GPU workstation and is out of scope here.
"""

import os

import yaml

from experiments.runner import ros_launcher as RL
from experiments.runner.scenario import Scenario
from experiments.runner.trial import DriveResult


def _scenario():
    return Scenario(name='utrap', tier='barn', map_yaml='/tmp/m.yaml',
                    start=(-2.0, -0.5, 0.0), goal=(0.9, -2.25, 0.0))


def test_build_launch_argv_matches_run_sim():
    argv = RL.build_launch_argv('/tmp/p.yaml', headless=True, use_rviz=False)
    assert argv[:4] == ['ros2', 'launch', 'nav2_bringup',
                        'tb3_simulation_launch.py']
    assert 'headless:=True' in argv
    assert 'use_rviz:=False' in argv
    assert 'params_file:=/tmp/p.yaml' in argv


def test_build_launch_argv_gui_variant():
    argv = RL.build_launch_argv('/tmp/p.yaml', headless=False, use_rviz=True)
    assert 'headless:=False' in argv
    assert 'use_rviz:=True' in argv


def _utrap_scenario():
    return Scenario(
        name='s1_utrap', tier='barn', map_yaml='/maps/utrap_loc.yaml',
        start=(-3.0, 0.0, 0.0), goal=(3.0, 0.0, 0.0),
        meta={'launch': {'pkg': 'se_mppi_utrap',
                         'file': 'utrap_bench.launch.py',
                         'pass_map': True, 'pass_spawn': True,
                         'use_sim_time': True}})


def test_launch_spec_default_is_tb3():
    spec = RL.launch_spec_for(_scenario())   # no meta.launch
    assert spec.pkg == 'nav2_bringup'
    assert spec.launch_file == 'tb3_simulation_launch.py'
    assert not spec.pass_map and not spec.pass_spawn


def test_launch_spec_from_meta_selects_testbed():
    spec = RL.launch_spec_for(_utrap_scenario())
    assert spec.pkg == 'se_mppi_utrap'
    assert spec.launch_file == 'utrap_bench.launch.py'
    assert spec.pass_map and spec.pass_spawn and spec.use_sim_time is True


def test_build_launch_argv_testbed_passes_map_and_spawn():
    sc = _utrap_scenario()
    spec = RL.launch_spec_for(sc)
    argv = RL.build_launch_argv('/tmp/p.yaml', headless=True, use_rviz=False,
                                spec=spec, scenario=sc)
    assert argv[:4] == ['ros2', 'launch', 'se_mppi_utrap',
                        'utrap_bench.launch.py']
    assert 'map:=/maps/utrap_loc.yaml' in argv
    assert 'x_pose:=-3.0' in argv and 'y_pose:=0.0' in argv
    assert 'yaw:=0.0' in argv
    assert 'use_sim_time:=True' in argv
    assert 'params_file:=/tmp/p.yaml' in argv


def test_build_launch_argv_default_omits_map_and_spawn():
    # The tb3 regression path must NOT gain map:=/x_pose:= (tb3_simulation owns
    # its own world, map and spawn).
    sc = _scenario()
    argv = RL.build_launch_argv('/tmp/p.yaml', spec=RL.launch_spec_for(sc),
                                scenario=sc)
    assert not any(a.startswith('map:=') for a in argv)
    assert not any(a.startswith('x_pose:=') for a in argv)


def test_apply_scenario_localization_seeds_amcl():
    params = {'amcl': {'ros__parameters': {
        'set_initial_pose': True,
        'initial_pose': {'x': -2.0, 'y': -0.5, 'z': 0.0, 'yaw': 0.0}}}}
    out = RL.apply_scenario_localization(params, _utrap_scenario())
    ip = out['amcl']['ros__parameters']['initial_pose']
    assert ip['x'] == -3.0 and ip['y'] == 0.0 and ip['yaw'] == 0.0
    assert out['amcl']['ros__parameters']['set_initial_pose'] is True
    # original untouched (deepcopy) — resolved params are shared across trials.
    assert params['amcl']['ros__parameters']['initial_pose']['x'] == -2.0


def test_apply_scenario_localization_noop_without_amcl():
    params = {'controller_server': {'ros__parameters': {}}}
    out = RL.apply_scenario_localization(params, _utrap_scenario())
    assert out == params


def test_write_params_overlay_roundtrips(tmp_path):
    params = {'controller_server': {'ros__parameters':
              {'FollowPath': {'se_enabled': True, 'se_alpha_escape': 6.0}}}}
    path = RL.write_params_overlay(params, out_dir=str(tmp_path))
    assert os.path.exists(path)
    with open(path) as f:
        got = yaml.safe_load(f)
    assert got == params


def test_write_params_overlay_creates_missing_out_dir(tmp_path):
    # results_dir root does not exist until the first JSON write; the params
    # overlay is written before that, so it must create the dir itself.
    out = tmp_path / 'results_pilot'   # does not exist yet
    path = RL.write_params_overlay({'a': 1}, out_dir=str(out))
    assert os.path.exists(path)
    assert os.path.dirname(path) == str(out)


def test_drive_env_carries_scenario_poses():
    env = RL.drive_env(_scenario(), base_env={}, localizer='amcl')
    assert float(env['SE_START_X']) == -2.0
    assert float(env['SE_START_Y']) == -0.5
    assert float(env['SE_GOAL_X']) == 0.9
    assert float(env['SE_GOAL_Y']) == -2.25
    assert env['SE_LOCALIZER'] == 'amcl'
    assert env['TURTLEBOT3_MODEL'] == 'waffle'


def test_drive_env_sets_drive_timeout_when_given():
    # The benchmark path must raise smoke_drive's cap to the trial watchdog so a
    # long-but-legitimate goal is not truncated at the 30 s manual-playbook default.
    env = RL.drive_env(_scenario(), base_env={}, localizer='amcl',
                       drive_timeout=120.0)
    assert float(env['SE_DRIVE_TIMEOUT']) == 120.0


def test_drive_env_omits_drive_timeout_by_default():
    # Unset => smoke_drive keeps its 30 s default (manual playbook unaffected).
    env = RL.drive_env(_scenario(), base_env={}, localizer='amcl')
    assert 'SE_DRIVE_TIMEOUT' not in env


def test_log_is_active_requires_all_markers():
    partial = 'Managed nodes are active\n'
    full = ('... Created controller : FollowPath of type '
            'nav2_se_controller::SafeEscapeController\n'
            '[lifecycle] Managed nodes are active\n')
    assert not RL.log_is_active(partial)
    assert RL.log_is_active(full)
    assert not RL.log_is_active('nothing useful here')


_SAMPLE_LOG = """\
[INFO] Nav2 active. Sending goal (0.9, -2.25).
[  0.0s] dist=2.95m amcl=(-2.00,-0.50 yaw=0.00rad) odom=(0.00,0.00 yaw=0.00rad) nav=vx:0.000 wz:0.000 cmd=vx:0.000 wz:0.000
[  0.5s] dist=2.50m amcl=(-1.80,-0.70 yaw=0.10rad) odom=(0.20,-0.20 yaw=0.10rad) nav=vx:0.250 wz:0.050 cmd=vx:0.240 wz:0.050
[  1.0s] dist=2.00m amcl=(-1.50,-1.00 yaw=0.30rad) odom=(0.50,-0.50 yaw=0.30rad) nav=vx:0.300 wz:0.100 cmd=vx:0.300 wz:0.100
SMOKE_RESULT=TaskResult.SUCCEEDED
"""


def test_parse_drive_log_builds_samples():
    res = RL.parse_drive_log(_SAMPLE_LOG)
    assert isinstance(res, DriveResult)
    assert len(res.samples) == 3
    first, last = res.samples[0], res.samples[-1]
    assert first == {'t': 0.0, 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
                     'v': 0.0, 'w': 0.0}
    assert last['t'] == 1.0 and last['x'] == 0.5 and last['y'] == -0.5
    assert last['v'] == 0.3 and last['w'] == 0.1
    assert res.nav_status == 'TaskResult.SUCCEEDED'
    assert res.timeout is False


def test_parse_drive_log_marks_timeout():
    log = _SAMPLE_LOG.replace('SMOKE_RESULT=TaskResult.SUCCEEDED',
                              'TIMEOUT after 90 s')
    res = RL.parse_drive_log(log)
    assert res.timeout is True
    assert len(res.samples) == 3


def test_parse_drive_log_tolerates_nan_cmd():
    log = ('[  0.0s] dist=1.0m amcl=(0,0 yaw=0rad) odom=(0.00,0.00 yaw=0.00rad) '
           'nav=vx:nan wz:nan cmd=vx:nan wz:nan\n')
    res = RL.parse_drive_log(log)
    assert len(res.samples) == 1
    s = res.samples[0]
    assert s['x'] == 0.0 and s['y'] == 0.0
    assert 'v' not in s and 'w' not in s   # NaN cmd dropped, pose kept


def test_parse_empty_log_is_empty_result():
    res = RL.parse_drive_log('')
    assert res.samples == []
    assert res.nav_status is None


def test_roslauncher_fail_fast_without_ros(monkeypatch):
    monkeypatch.setattr(RL, 'ros_available', lambda: False)
    launcher = RL.RosLauncher()
    try:
        launcher.launch({}, _scenario())
        assert False, 'expected RuntimeError when ros2 is absent'
    except RuntimeError as e:
        assert 'ROS2' in str(e)


def test_roslauncher_importable_via_trial():
    # Backward-compatible name path (lazy re-export) still resolves.
    from experiments.runner.trial import RosLauncher
    assert RosLauncher is RL.RosLauncher


def test_launch_env_forces_software_rendering_by_default():
    env = RL.launch_env(base_env={})
    assert env['MESA_GL_VERSION_OVERRIDE'] == '3.3'
    assert env['LIBGL_ALWAYS_SOFTWARE'] == '1'


def test_launch_env_respects_caller_override():
    env = RL.launch_env(base_env={'MESA_GL_VERSION_OVERRIDE': '4.5',
                                  'LIBGL_ALWAYS_SOFTWARE': '0'})
    assert env['MESA_GL_VERSION_OVERRIDE'] == '4.5'   # GPU host stays in charge
    assert env['LIBGL_ALWAYS_SOFTWARE'] == '0'
