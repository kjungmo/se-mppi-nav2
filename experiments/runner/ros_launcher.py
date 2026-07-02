# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Live ROS2/Gazebo launcher backend (design §9, H-4) — pure helpers + shell.

This module implements the concrete :class:`Launcher` (the seam defined in
``trial.py``) for a real ROS2 Jazzy + Nav2 + Gazebo workstation. It brings up the
SE-MPPI stack with ``ros2 launch nav2_bringup tb3_simulation_launch.py`` (the same
invocation as ``scripts/run_sim.sh``), waits for the managed nodes to go active,
drives the goal by reusing ``experiments/sim/smoke_drive.py``, parses its
telemetry into the metrics.py sample stream, and tears everything down with the
clean-restart discipline from the live-run handoff (§2-A).

Design choice (so the harness stays offline-testable): everything that does NOT
need ROS is a **pure function** at module scope — launch-argv construction,
params-overlay serialisation, the "active" log matcher, the drive-telemetry
parser, and the env construction for the goal. ``rclpy``/subprocess only appear
inside the imperative :class:`RosLauncher` methods, and even ``ros2`` is probed
lazily so importing this module on a no-ROS machine is safe. The unit tests
exercise the pure helpers with canned strings; the live methods need the GPU box.

Honest scope (reported, not hidden): kinematic metrics (path length, time,
smoothness, oscillation) come straight from the odom telemetry smoke_drive
already prints. Ground-truth dynamic-obstacle sampling for collision/clearance on
the DynaBARN/HuNav tiers needs extra subscriptions smoke_drive does not make —
that is the workstation-side extension point flagged in :func:`drive`.
"""

from __future__ import annotations

import copy
import os
import re
import shutil
import signal
import subprocess
import tempfile
import time
from dataclasses import dataclass, field

import yaml

from . import cleanup as cleanup_mod
from .scenario import Scenario
from .trial import DriveResult

# Default launch entrypoint (matches scripts/run_sim.sh): the tb3_sandbox world +
# Nav2, unchanged so the verified passage scenario keeps its regression path.
LAUNCH_PKG = 'nav2_bringup'
LAUNCH_FILE = 'tb3_simulation_launch.py'

# bt_navigator / lifecycle log fragments that mark a fully-active stack.
_ACTIVE_MARKERS = (
    'Managed nodes are active',
    'Created controller : FollowPath',
)

# smoke_drive.py telemetry line, e.g.:
#   [ 12.4s] dist=1.83m amcl=(-1.20,-0.50 yaw=0.01rad) odom=(0.79,0.00 yaw=0.00rad) \
#            nav=vx:0.250 wz:0.000 cmd=vx:0.240 wz:0.010
_TELEM_RE = re.compile(
    r'\[\s*(?P<t>[-\d.]+)s\]\s+dist=(?P<dist>[-\d.]+)m'
    r'.*?odom=\((?P<ox>[-\d.]+),(?P<oy>[-\d.]+)\s+yaw=(?P<oyaw>[-\d.]+)rad\)'
    r'.*?cmd=vx:(?P<vx>[-\d.naN]+)\s+wz:(?P<wz>[-\d.naN]+)'
)
# Terminal result line printed by smoke_drive.py.
_RESULT_RE = re.compile(r'SMOKE_RESULT=(?P<result>\S+)')


# --------------------------------------------------------------------------- #
# Pure helpers (no ROS, no subprocess) — these are what the offline tests cover
# --------------------------------------------------------------------------- #
def ros_available() -> bool:
    """True iff a ``ros2`` executable is on PATH (cheap fail-fast probe)."""
    return shutil.which('ros2') is not None


def write_params_overlay(params: dict, out_dir: str | None = None) -> str:
    """Dump a resolved params dict (from ``config.resolve_params``) to a temp
    Nav2 yaml and return its path. The runner overlays this via ``params_file:=``.
    """
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)  # results_dir root need not exist yet
    fd, path = tempfile.mkstemp(prefix='se_params_', suffix='.yaml', dir=out_dir)
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
    return path


@dataclass
class LaunchSpec:
    """Which ``ros2 launch`` file brings up a scenario's world + Nav2.

    A scenario selects its testbed by carrying a ``meta['launch']`` dict; when
    absent, the default is the verified ``nav2_bringup tb3_simulation_launch.py``
    (tb3_sandbox), so the passage scenario's regression path is unchanged.

    ``pass_map``/``pass_spawn`` let a self-contained testbed (the U-trap gz world,
    which does NOT hardcode a map or a spawn pose the way tb3_simulation does)
    receive them from the Scenario, so the map the planner localises against and
    the physical spawn both come from one source of truth. ``extra_args`` are
    static ``k:=v`` launch arguments (e.g. a fixed world file).
    """
    pkg: str = LAUNCH_PKG
    launch_file: str = LAUNCH_FILE
    pass_map: bool = False
    pass_spawn: bool = False
    use_sim_time: bool | None = None
    extra_args: dict = field(default_factory=dict)


DEFAULT_LAUNCH_SPEC = LaunchSpec()


def launch_spec_for(scenario: Scenario) -> LaunchSpec:
    """Resolve a scenario's :class:`LaunchSpec` from ``meta['launch']``.

    No ``meta['launch']`` => the tb3_sandbox default (regression preserved).
    """
    m = (scenario.meta or {}).get('launch')
    if not m:
        return DEFAULT_LAUNCH_SPEC
    return LaunchSpec(
        pkg=m.get('pkg', LAUNCH_PKG),
        launch_file=m.get('file', LAUNCH_FILE),
        pass_map=bool(m.get('pass_map', False)),
        pass_spawn=bool(m.get('pass_spawn', False)),
        use_sim_time=m.get('use_sim_time'),
        extra_args=dict(m.get('args', {}) or {}),
    )


def build_launch_argv(params_file: str, *, headless: bool = True,
                      use_rviz: bool = False, spec: LaunchSpec | None = None,
                      scenario: Scenario | None = None) -> list:
    """The ``ros2 launch`` argv that brings up the sim with our params.

    With no ``spec`` this mirrors ``scripts/run_sim.sh``'s headless tb3 invocation
    so behaviour stays identical between the manual playbook and the runner. A
    ``spec`` (from :func:`launch_spec_for`) redirects to another testbed launch
    and, when it opts in, appends ``map:=`` / ``x_pose:=`` etc. from the scenario.
    """
    spec = spec or DEFAULT_LAUNCH_SPEC
    argv = [
        'ros2', 'launch', spec.pkg, spec.launch_file,
        f'headless:={"True" if headless else "False"}',
        f'use_rviz:={"True" if use_rviz else "False"}',
        f'params_file:={params_file}',
    ]
    if spec.use_sim_time is not None:
        argv.append(f'use_sim_time:={"True" if spec.use_sim_time else "False"}')
    if spec.pass_map and scenario is not None:
        argv.append(f'map:={scenario.map_yaml}')
    if spec.pass_spawn and scenario is not None:
        sx, sy = scenario.start_xy
        yaw = scenario.start[2] if len(scenario.start) > 2 else 0.0
        argv += [f'x_pose:={sx}', f'y_pose:={sy}', f'yaw:={yaw}']
    for k, v in spec.extra_args.items():
        argv.append(f'{k}:={v}')
    return argv


def apply_scenario_localization(params: dict, scenario: Scenario) -> dict:
    """Return a copy of ``params`` with AMCL seeded at the scenario's start.

    A fresh Gazebo launch has no localization belief; AMCL's near-zero motion
    noise (the loopback config) means it never diffuses to the truth, so it must
    be seeded at the true spawn. Both the physical spawn (``pass_spawn``) and this
    belief come from ``scenario.start``, so they cannot drift apart between
    scenarios that share one base params file (tb3 spawns at its own default,
    which the tb3 scenario's start mirrors). No AMCL section => unchanged.
    """
    out = copy.deepcopy(params)
    amcl = out.get('amcl')
    if not isinstance(amcl, dict) or 'ros__parameters' not in amcl:
        return out
    sx, sy = scenario.start_xy
    yaw = scenario.start[2] if len(scenario.start) > 2 else 0.0
    amcl['ros__parameters']['set_initial_pose'] = True
    amcl['ros__parameters']['initial_pose'] = {
        'x': float(sx), 'y': float(sy), 'z': 0.0, 'yaw': float(yaw)}
    return out


def drive_env(scenario: Scenario, *, base_env: dict | None = None,
              localizer: str = 'amcl', drive_timeout: float | None = None) -> dict:
    """Environment for the smoke_drive subprocess: feed start/goal from the
    scenario via the SE_* overrides smoke_drive.py already honours.

    ``localizer='amcl'`` matches the Gazebo live-run path (handoff §2-C): under
    AMCL, smoke_drive skips the initialpose re-publish that would corrupt a
    correct belief.

    ``drive_timeout`` (when set) is passed as ``SE_DRIVE_TIMEOUT`` so smoke_drive
    does NOT fall back to its 30 s manual-playbook default and truncate a real
    benchmark goal mid-navigation. The runner sets this to the trial watchdog.
    """
    env = dict(base_env if base_env is not None else os.environ)
    sx, sy = scenario.start_xy
    gx, gy = scenario.goal_xy
    env.update({
        'SE_START_X': repr(float(sx)), 'SE_START_Y': repr(float(sy)),
        'SE_GOAL_X': repr(float(gx)), 'SE_GOAL_Y': repr(float(gy)),
        'SE_LOCALIZER': localizer,
        'TURTLEBOT3_MODEL': env.get('TURTLEBOT3_MODEL', 'waffle'),
    })
    if drive_timeout is not None:
        env['SE_DRIVE_TIMEOUT'] = repr(float(drive_timeout))
    return env


def launch_env(base_env: dict | None = None) -> dict:
    """Environment for the ``ros2 launch`` subprocess.

    Forces Mesa software rendering (``MESA_GL_VERSION_OVERRIDE=3.3`` +
    ``LIBGL_ALWAYS_SOFTWARE=1``) unless the caller already chose: on this WSL2
    host gz's gpu_lidar otherwise renders a degenerate buffer and every beam
    reads exactly ``range_min`` (verified with a live scan probe: 0.080 m at
    all 360 bearings; with the override the same probe returns true geometry).
    A blind lidar poisons the whole benchmark — the costmap saturates into a
    uniform lethal ring, obstacle avoidance silently turns off, and AMCL never
    corrects. ``setdefault`` keeps a real-GPU workstation free to override.
    """
    env = dict(base_env if base_env is not None else os.environ)
    env.setdefault('MESA_GL_VERSION_OVERRIDE', '3.3')
    env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    return env


def log_is_active(log_text: str) -> bool:
    """True if the launch log shows the stack reached the active state."""
    return all(marker in log_text for marker in _ACTIVE_MARKERS)


def _to_float(tok: str) -> float | None:
    try:
        v = float(tok)
    except (TypeError, ValueError):
        return None
    return None if v != v else v  # drop NaN


def parse_drive_log(log_text: str) -> DriveResult:
    """Turn smoke_drive.py stdout into a :class:`DriveResult`.

    Each telemetry line becomes one metrics.py sample (``t,x,y,yaw,v,w`` from the
    odom + cmd fields). ``SMOKE_RESULT=`` maps to ``nav_status``; an explicit
    ``TIMEOUT`` line (smoke_drive's watchdog) sets ``timeout``. This is the live
    counterpart of the synthetic ``DriveResult`` a FakeLauncher returns.
    """
    samples: list = []
    for m in _TELEM_RE.finditer(log_text):
        t = _to_float(m.group('t'))
        ox = _to_float(m.group('ox'))
        oy = _to_float(m.group('oy'))
        if t is None or ox is None or oy is None:
            continue
        sample = {'t': t, 'x': ox, 'y': oy}
        oyaw = _to_float(m.group('oyaw'))
        if oyaw is not None:
            sample['yaw'] = oyaw
        vx = _to_float(m.group('vx'))
        wz = _to_float(m.group('wz'))
        if vx is not None:
            sample['v'] = vx
        if wz is not None:
            sample['w'] = wz
        samples.append(sample)

    nav_status = None
    rm = _RESULT_RE.search(log_text)
    if rm:
        nav_status = rm.group('result')
    timeout = 'TIMEOUT' in log_text
    return DriveResult(samples=samples, timeout=timeout, nav_status=nav_status)


# --------------------------------------------------------------------------- #
# Imperative shell — needs ROS2 + Gazebo (GPU workstation only)
# --------------------------------------------------------------------------- #
@dataclass
class _Handle:
    """Opaque per-trial state returned by ``launch`` (passed back to the rest)."""
    proc: object                       # the ros2 launch Popen
    params_file: str                   # temp params yaml to clean up
    log_path: str                      # captured launch log
    log_file: object = None            # open file handle for the log


class RosLauncher:
    """ROS2/Gazebo backend implementing the :class:`trial.Launcher` protocol.

    Drop-in for ``FakeLauncher``: ``run_suite``/``run_trial`` use it unchanged via
    a launcher factory. Fail-fast (no ROS / no GPU) raises in ``launch`` — never
    at import — so the offline harness and its 48 tests keep importing cleanly.
    """

    def __init__(self, *, headless: bool = True, use_rviz: bool = False,
                 localizer: str = 'amcl', smoke_drive: str | None = None,
                 results_dir: str | None = None, logger=None):
        self.headless = headless
        self.use_rviz = use_rviz
        self.localizer = localizer
        self.results_dir = results_dir
        self._logger = logger
        # experiments/sim/smoke_drive.py relative to this file (../sim/...).
        here = os.path.dirname(os.path.abspath(__file__))
        self.smoke_drive = smoke_drive or os.path.normpath(
            os.path.join(here, '..', 'sim', 'smoke_drive.py'))

    def _log(self, msg):
        if self._logger:
            self._logger(msg)

    def launch(self, params: dict, scenario: Scenario) -> object:
        if not ros_available():
            raise RuntimeError(
                'RosLauncher requires a working ROS2 install (no `ros2` on '
                'PATH). Activate the env (see CLAUDE.md / scripts/run_sim.sh) '
                'and run on a GPU workstation; design §9.')
        params = apply_scenario_localization(params, scenario)
        params_file = write_params_overlay(params, out_dir=self.results_dir)
        spec = launch_spec_for(scenario)
        argv = build_launch_argv(params_file, headless=self.headless,
                                 use_rviz=self.use_rviz, spec=spec,
                                 scenario=scenario)
        log_path = self._log_path(scenario)
        log_file = open(log_path, 'wb')
        self._log(f'launch: {" ".join(argv)}')
        # New process group so teardown can SIGTERM the whole launch tree.
        proc = subprocess.Popen(argv, stdout=log_file, stderr=subprocess.STDOUT,
                                 env=launch_env(), start_new_session=True)
        return _Handle(proc=proc, params_file=params_file, log_path=log_path,
                       log_file=log_file)

    def wait_active(self, handle: object, timeout: float) -> bool:
        """Poll the launch log until the active markers appear or ``timeout``."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if handle.proc.poll() is not None:  # launch died early
                return False
            try:
                with open(handle.log_path) as f:
                    if log_is_active(f.read()):
                        return True
            except OSError:
                pass
            time.sleep(1.0)
        return False

    def drive(self, handle: object, scenario: Scenario,
              watchdog_s: float) -> DriveResult:
        """Run smoke_drive.py against the live stack; parse its telemetry.

        smoke_drive.py self-cancels at ``SE_DRIVE_TIMEOUT`` (set here to
        ``watchdog_s``) and prints its result; the launcher then allows a short
        grace and hard-kills only if it hangs past that. This is what makes the
        run turnkey: without it smoke_drive's hardcoded 30 s default would
        truncate every goal that legitimately takes longer. The manual playbook
        is unaffected — its 30 s default applies whenever SE_DRIVE_TIMEOUT is unset.

        Extension point (workstation): for DynaBARN/HuNav, subscribe to the
        ground-truth agent topics here and merge per-instant obstacle lists into
        the samples so collision/clearance use GT, not costmap estimates
        (protocol §3). Static-tier kinematics need no change.
        """
        env = drive_env(scenario, localizer=self.localizer,
                        drive_timeout=watchdog_s)
        proc = subprocess.Popen(
            ['python3', self.smoke_drive], env=env, start_new_session=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            # smoke_drive self-cancels at watchdog_s and prints; the +10 s grace
            # lets us capture that telemetry instead of SIGKILLing mid-print.
            out, _ = proc.communicate(timeout=watchdog_s + 10.0)
            result = parse_drive_log(out)
        except subprocess.TimeoutExpired:
            self._kill_group(proc)
            out = ''
            try:
                out = (proc.communicate(timeout=5)[0] or '')
            except Exception:
                pass
            result = parse_drive_log(out)
            result.timeout = True
        self._save_drive_log(scenario, out)
        return result

    def teardown(self, handle: object) -> None:
        """SIGTERM the launch process group, then run the clean-restart sweep."""
        if handle is None:
            return
        try:
            self._kill_group(handle.proc)
        finally:
            if handle.log_file is not None:
                try:
                    handle.log_file.close()
                except Exception:
                    pass
            try:
                os.remove(handle.params_file)
            except OSError:
                pass
            # Process/SHM/daemon clean-restart (handoff §2-A) — codified once.
            cleanup_mod.run_cleanup(logger=self._logger)

    def _save_drive_log(self, scenario: Scenario, out: str) -> None:
        """Persist the raw smoke_drive telemetry next to the launch log, so a
        STUCK/TIMEOUT trial can be diagnosed (cmd_vel, dist, odom progression)
        after the fact instead of being lost to the parser."""
        if not self.results_dir or not out:
            return
        try:
            os.makedirs(self.results_dir, exist_ok=True)
            with open(os.path.join(self.results_dir,
                                   f'drive_{scenario.name}.log'), 'w') as f:
                f.write(out)
        except OSError:
            pass

    # -- internals ---------------------------------------------------------- #
    def _log_path(self, scenario: Scenario) -> str:
        if self.results_dir:
            os.makedirs(self.results_dir, exist_ok=True)
            return os.path.join(self.results_dir, f'launch_{scenario.name}.log')
        fd, path = tempfile.mkstemp(prefix=f'se_launch_{scenario.name}_',
                                    suffix='.log')
        os.close(fd)
        return path

    @staticmethod
    def _kill_group(proc) -> None:
        """SIGTERM then SIGKILL the whole process group started by ``proc``."""
        if proc is None or proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except (ProcessLookupError, OSError):
            return
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                os.killpg(pgid, sig)
            except (ProcessLookupError, OSError):
                return
            for _ in range(10):
                if proc.poll() is not None:
                    return
                time.sleep(0.5)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
