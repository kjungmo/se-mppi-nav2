"""Tests of the campaign-2 analyzer on synthetic trial directories and logs."""

import gzip
import json
import math
import os
import re

import pytest

import analyze as an
import plan

REPO = an.REPO
T0 = 1_792_000_000.0            # wall time the goal is sent
SRC = os.path.join(REPO, 'src', 'nav2_se_controller', 'src', 'safe_escape_controller.cpp')
CRIT = os.path.join(REPO, 'src', 'nav2_se_controller', 'src', 'escape_critic.cpp')

# Real lines of the controller, copied from the headless closed-loop run
# (/mnt/d/wsl/se-mppi/escape-fix/logs/cl_final20_w16_d15_r-15_a6.log).
REAL_ENTER = ('[INFO] [1792268244.589171131] [MPPIController]: SE escape ENTER at (0.45, 0.17): '
              'progress_events=22 arc=2.99 stall=30')
REAL_WP1 = ('[INFO] [1792268244.589408192] [MPPIController]: SE escape waypoint 1 (entrapped): '
            'from (0.45, 0.18) to (-0.73, 1.58); bearing 130 deg, clearance 2.00 m, horizon ray; '
            'goal bearing -4 deg')
REAL_WP2 = ('[INFO] [1792268244.653080479] [MPPIController]: SE escape waypoint 2 (waypoint reached): '
            'from (-0.51, 1.26) to (0.85, 1.50); bearing 10 deg, clearance 1.55 m, horizon ray; '
            'goal bearing -20 deg')
REAL_EXIT = ('[INFO] [1792268244.704772041] [MPPIController]: SE escape EXIT at (0.50, 1.69): '
             'progress_events=23 arc=3.10 stall=0')
REAL_BEGIN = ('[INFO] [1792268244.589940793] [controller_server]: EscapeCritic[FollowPath.EscapeCritic] '
              'escape-cost injection BEGIN (entrapped=1)')
REAL_OPEN = ('[INFO] [1792268244.590437266] [controller_server]: EscapeCritic[FollowPath.EscapeCritic] '
             'critic-side probe (no avoid list, no hysteresis; NOT the escape heading): bearing 130 deg, '
             'clearance 2.00 m, horizon ray (7 horizon rays) from (0.45, 0.18); bearing to the critic '
             'goal 130 deg')
LONGEST = ('[INFO] [1792268244.7] [MPPIController]: SE escape waypoint 3 (stalled): from (1.00, 2.00) '
           'to (1.50, 2.50); bearing -45 deg, clearance 0.90 m, longest ray (no horizon ray), '
           'previous bearing kept; goal bearing 0 deg')


# --------------------------------------------------------------------------- #
# Log patterns
# --------------------------------------------------------------------------- #
def test_patterns_parse_real_controller_lines():
    text = '\n'.join([REAL_ENTER, REAL_WP1, REAL_BEGIN, REAL_OPEN, REAL_WP2, REAL_EXIT])
    p = an.parse_launch_log(text, T0)
    assert p['escape_enter'] == 1 and p['escape_exit'] == 1
    assert p['escape_hops'] == 2 and p['escape_injection_begin'] == 1
    assert p['escape_critic_probe_lines'] == 1
    assert 'escape_opening_lines' not in p        # the probe is not the escape heading
    ep = p['escape_episodes'][0]
    assert ep['duration_s'] == pytest.approx(1792268244.704772041 - 1792268244.589171131)
    assert ep['enter_xy'] == [0.45, 0.17]
    h1, h2 = p['escape_hop_list']
    assert (h1['hop'], h1['why'], h1['bearing_deg'], h1['clearance_m'], h1['horizon_ray']) == (
        1, 'entrapped', 130, 2.0, True)
    assert h1['from'] == [0.45, 0.18] and h1['to'] == [-0.73, 1.58]
    assert (h2['hop'], h2['why'], h2['bearing_deg']) == (2, 'waypoint reached', 10)


def test_longest_ray_and_previous_bearing_suffix_parse():
    p = an.parse_launch_log(LONGEST, T0)
    h = p['escape_hop_list'][0]
    assert p['escape_hops'] == 1 and h['horizon_ray'] is False and h['bearing_deg'] == -45


def test_patterns_agree_with_the_format_strings_of_the_controller_source():
    src = open(SRC).read()
    # The regular expressions are written for these format strings.
    assert '"SE escape %s at (%.2f, %.2f): progress_events=%zu arc=%.2f stall=%d"' in src
    assert 'SE escape waypoint %d (%s): from (%.2f, %.2f) to (%.2f, %.2f); bearing %.0f deg, ' in src
    assert '"clearance %.2f m, %s%s; goal bearing %.0f deg"' in src
    assert '"horizon ray" : "longest ray (no horizon ray)"' in src
    assert 'entrapped ? "ENTER" : "EXIT"' in src
    crit = open(CRIT).read()
    assert 'escape-cost injection %s (entrapped=%d)' in crit
    assert 'critic-side probe (no avoid list, no hysteresis; NOT the escape "\n          "heading): bearing %.0f deg, clearance %.2f m, %s' in crit
    assert 'log_begin ? "BEGIN"' in crit or '"BEGIN"' in crit


def test_patterns_do_not_match_near_misses():
    assert an.ENTER_RE.search(REAL_EXIT) is None and an.EXIT_RE.search(REAL_ENTER) is None
    assert an.WAYPOINT_RE.search(REAL_ENTER) is None
    assert an.parse_launch_log('SE escape ENTER at (a, b)', T0)['escape_enter'] == 0
    assert an.parse_launch_log('', T0)['escape_enter'] == 0
    assert an.INJECT_END_RE.search(REAL_BEGIN) is None


def test_episodes_pair_in_time_order_and_open_episodes_are_kept():
    def enter(t, x=0.0):
        return f'[INFO] [{t}] [MPPIController]: SE escape ENTER at ({x}, 0.0): progress_events=1 arc=1 stall=30'

    def exit_(t):
        return f'[INFO] [{t}] [MPPIController]: SE escape EXIT at (1.0, 0.0): progress_events=2 arc=2 stall=0'
    # exit line listed before its enter line: still paired by time
    text = '\n'.join([exit_(110.0), enter(100.0), enter(120.0), exit_(125.0), enter(130.0)])
    p = an.parse_launch_log(text, 90.0)
    eps = p['escape_episodes']
    assert [e['enter_s'] for e in eps] == [10.0, 30.0, 40.0]
    assert [e['duration_s'] for e in eps] == [10.0, 5.0, None]
    assert p['escape_open_at_end'] is True and p['escape_total_s'] == 15.0
    assert p['escape_enter'] == 3 and p['escape_exit'] == 2
    # two ENTER lines in a row: the first episode stays open, the next one starts
    p2 = an.parse_launch_log(enter(1.0) + '\n' + enter(2.0), 0.0)
    assert [e['exit_s'] for e in p2['escape_episodes']] == [None, None]


def test_recovery_and_failure_counters():
    text = '\n'.join([
        '[component_container_isolated-5] [INFO] [100.1] [behavior_server]: Running spin',
        '[component_container_isolated-5] [INFO] [101.1] [behavior_server]: Running backup',
        '[component_container_isolated-5] [INFO] [102.1] [behavior_server]: Running backup',
        '[component_container_isolated-5] [INFO] [103.1] [behavior_server]: Running wait',
        '[INFO] [104.0] [behavior_server]: Running spinning',                       # near miss
        '[INFO] [104.1] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap',
        '[INFO] [104.2] [global_costmap.global_costmap]: Received request to clear entirely the global_costmap',
        '[INFO] [104.3] [global_costmap.global_costmap]: Received request to clear entirely the global_costmap',
        '[ERROR] [105.0] [controller_server]: Failed to make progress',
        '[WARN] [106.0] [controller_server]: Optimizer fail to compute path',
        '[WARN] [107.0] [controller_server]: Optimizer fail to compute path',
        '[WARN] controller_server Control loop missed its desired rate of 10.0Hz',
        '[WARN] [108.0] [controller_server]: SafeEscapeController[FollowPath]: vx_min = 0.100 > 0 with se_enabled: x',
    ])
    p = an.parse_launch_log(text, 100.0)
    assert (p['bt_spin'], p['bt_backup'], p['bt_wait']) == (1, 2, 1)
    assert (p['bt_clear_local'], p['bt_clear_global']) == (1, 2)
    assert p['progress_failures'] == 1 and p['optimizer_failures'] == 2
    assert p['first_optimizer_failure_s'] == pytest.approx(6.0)
    assert p['missed_control_cycles'] == 1 and p['vx_min_warning'] is True


def test_excerpt_keeps_evidence_and_counts_the_rest():
    fails = '\n'.join(f'[WARN] [{i}.0] [controller_server]: Optimizer fail to compute path' for i in range(10))
    text = '\n'.join(['noise line', REAL_ENTER, fails, REAL_EXIT, 'other noise',
                      '[ERROR] [1.0] [controller_server]: Failed to make progress'])
    ex = an.excerpt(text)
    assert 'noise' not in ex.replace('# optimizer', '')
    assert ex.count('Optimizer fail') == 3                      # the first three only
    assert '# optimizer failures: 10;' in ex
    assert 'SE escape ENTER' in ex and 'SE escape EXIT' in ex and 'Failed to make progress' in ex


def test_escape_path_log_summary_ignores_probe_messages():
    lines = [
        {'kind': 'graph', 'tag': '2s', 'wall_s': 1.0, 'topic': '/FollowPath/se_escape_path', 'publishers': 0},
        {'kind': 'graph', 'tag': '10s', 'wall_s': 9.0, 'topic': '/FollowPath/se_escape_path', 'publishers': 1},
        {'kind': 'path', 'wall_s': 20.0, 'frame': 'odom', 'n_poses': 51},
        {'kind': 'path', 'wall_s': 21.0, 'frame': an.PROBE_FRAME, 'n_poses': 2},
        {'kind': 'path', 'wall_s': 22.0, 'frame': 'odom', 'n_poses': 30},
    ]
    s = an.parse_escape_path_log('\n'.join(json.dumps(x) for x in lines) + '\nnot json\n\n')
    assert s['escape_path_messages'] == 2 and s['escape_path_probe_messages'] == 1
    assert s['escape_path_publishers_seen'] == 1 and s['escape_path_first_wall'] == 20.0
    empty = an.parse_escape_path_log('')
    assert empty['escape_path_messages'] == 0 and empty['escape_path_publishers_seen'] is None


# --------------------------------------------------------------------------- #
# Synthetic trial directories
# --------------------------------------------------------------------------- #
def make_trial(root, name, scenario, config, traj, *, sub='trials', launch='', result='SUCCEEDED',
               watchdog=False, foreign=0.1, start=(-3.0, 0.0), goal=(3.0, 0.0), rep=0,
               escape_path='', complete=True, active=True, no_goal=False, drive_s=None, vx_min=-0.35):
    """Write one trial directory. ``traj`` is a list of (t_rel, x, y, yaw)."""
    d = os.path.join(root, sub, name)
    os.makedirs(d, exist_ok=True)
    t_end = drive_s if drive_s is not None else traj[-1][0]
    meta = {
        'trial_id': name, 'scenario': scenario, 'config': config, 'rep': rep, 'phase': 'A',
        'env': {'repo_commit': 'abc123'}, 'complete': complete, 'active': active,
        'start': list(start), 'goal': list(goal), 'launch_log': 'launch.log',
        'drive_wall_start': T0, 'drive_wall_end': T0 + t_end, 'setup_attempts': 1,
        'load1_at_start': 0.5, 'load_gate_waited_s': 0.0, 'params_yaml_sha256': 'f' * 64}
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    if not active or not complete:
        return d
    with open(os.path.join(d, 'params.yaml'), 'w') as f:
        f.write(f'controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_min: {vx_min}\n')
    send = ('[INFO] [%.6f] [basic_navigator]: Nav2 active. Sending goal (3.0, 0.0).' % T0)
    last = ('[INFO] [%.6f] [basic_navigator]: [%.1fs] dist=1.0m amcl=(0.0,0.0 yaw=0.0rad) '
            'odom=(0.0,0.0 yaw=0.0rad) nav=vx:0.1 wz:0.0 cmd=vx:0.1 wz:0.0' % (T0 + t_end, t_end))
    tail = ('SMOKE_RESULT=TaskResult.%s\n' % result) if not watchdog else 'TIMEOUT after 300 s\n'
    with open(os.path.join(d, 'drive.log'), 'w') as f:
        f.write(('' if no_goal else send + '\n') + ('' if no_goal else last + '\n') + tail)
    with open(os.path.join(d, 'launch.log'), 'w') as f:
        f.write(launch)
    with open(os.path.join(d, 'escape_path.jsonl'), 'w') as f:
        f.write(escape_path)
    with open(os.path.join(d, 'diagnostics.jsonl'), 'w') as f:
        f.write(json.dumps({'wall_s': T0, 'sim_s': 0, 'name': 'x', 'level': 0, 'message': 'ok',
                            'values': {'mppi_rollout_check': 'ok', 'cycles': '10',
                                       'escape_hops': '2', 'base_failures': '0'}}) + '\n')
    with open(os.path.join(d, 'telemetry.csv'), 'w') as f:
        f.write('wall_s,sim_s,gt_x,gt_y,gt_z,gt_roll,gt_pitch,gt_yaw,odom_x,odom_y,odom_yaw,'
                'nav_vx,nav_wz,cmd_vx,cmd_wz,load1\n')
        for t, x, y, yaw in traj:
            f.write(f'{T0 + t:.3f},{0.95 * t:.3f},{x},{y},0,0,0,{yaw},{x},{y},{yaw},0,0,0,0,0.5\n')
    with open(os.path.join(d, 'host_cpu.csv'), 'w') as f:
        f.write('wall_s,busy_cpu_s,own_cpu_s,load1,load5,ncpu\n')
        n = int(t_end / 5) + 4
        for i in range(-1, n):
            t = T0 + 5.0 * i
            f.write(f'{t:.3f},{(2.0 + foreign) * 5.0 * i:.3f},{2.0 * 5.0 * i:.3f},0.5,0.5,12\n')
    return d


def drive_to_goal(t_goal=60.0, via=()):
    """-3,0 -> via points -> 3,0 at 10 Hz, ending at the goal."""
    pts = [(-3.0, 0.0)] + list(via) + [(3.0, 0.0)]
    n_seg = len(pts) - 1
    traj, t = [], 0.0
    per = t_goal / n_seg
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        steps = int(per * 10)
        for i in range(steps):
            f = i / steps
            traj.append((t, x0 + f * (x1 - x0), y0 + f * (y1 - y0), 0.0))
            t += 0.1
    traj.append((t, 3.0, 0.0, 0.0))
    return traj


def stuck_in_pocket(t_end=300.0, x=0.2):
    traj = [(i * 0.1, min(-3.0 + 0.5 * i * 0.1, x), 0.0, 0.0) for i in range(int(t_end * 10) + 1)]
    return traj


def analyze_one(tmp_path, *args, **kw):
    d = make_trial(str(tmp_path), *args, **kw)
    boxes = an.world_boxes(an.WORLDS[args[1]])
    return an.analyze_trial(d, boxes, an.pilot.chassis_size())


def test_reached_trial_fields(tmp_path):
    t = analyze_one(tmp_path, 'correct__s0_open__C2__rep0', 's0_open', 'C2', drive_to_goal(60.0))
    assert t['outcome'] == 'REACHED' and t['success'] is True and t['reached'] is True
    assert t['contact'] is False
    assert t['final_goal_distance_m'] == pytest.approx(0.0)
    # first ground-truth sample within 0.5 m of the goal: x >= 2.5 on the straight line
    assert t['time_to_goal_wall_s'] == pytest.approx(60.0 * 5.5 / 6.0, abs=0.2)
    assert t['time_to_goal_sim_s'] == pytest.approx(0.95 * t['time_to_goal_wall_s'], abs=0.2)
    assert t['se_config'] is True and t['controller'] == 'F_se_full'
    assert t['pocket'] is False and t['enter_before_leaving_pocket'] is None
    assert t['reached_without_escape'] is None
    assert t['host_load']['valid'] is True
    assert t['rtf'] == pytest.approx(0.95, abs=0.01)
    assert t['params_sha256'] == 'f' * 64 and t['repo_commit'] == 'abc123'
    assert t['diagnostics_last']['escape_hops'] == '2'


def test_not_reached_outcomes(tmp_path):
    traj = stuck_in_pocket(60.0)
    t = analyze_one(tmp_path, 'a', 's1_utrap', 'C3', traj, result='ABORTED', drive_s=60.0)
    assert t['outcome'] == 'NOT_REACHED_ABORTED' and t['success'] is False
    assert t['se_config'] is False and t['enter_before_leaving_pocket'] is None
    t = analyze_one(tmp_path, 'b', 's1_utrap', 'C3', traj, watchdog=True, drive_s=60.0)
    assert t['outcome'] == 'NOT_REACHED_TIMEOUT'
    # Nav2 says SUCCEEDED but the robot is far from the goal: a false success, still not reached
    t = analyze_one(tmp_path, 'c', 's1_utrap', 'C1', traj, result='SUCCEEDED', drive_s=60.0)
    assert t['outcome'] == 'NOT_REACHED_ABORTED' and t['nav2_false_success'] is True
    assert t['time_to_goal_wall_s'] is None


def test_goal_tolerance_boundary(tmp_path):
    for dx, ok in ((0.499, True), (0.501, False)):
        traj = [(0.0, -3.0, 0.0, 0.0), (10.0, 3.0 - dx, 0.0, 0.0)]
        t = analyze_one(tmp_path, f'g{dx}', 's0_open', 'C1', traj)
        assert t['reached'] is ok, dx


def test_contact_marks_reached_with_contact_and_not_a_success(tmp_path):
    lx, ly = an.pilot.chassis_size()
    face = 0.85
    traj = [(0.0, -3.0, 0.0, 0.0), (5.0, face - lx / 2 - 0.004, 0.0, 0.0), (10.0, 3.0, 0.0, 0.0)]
    t = analyze_one(tmp_path, 'k', 's1_utrap', 'C2', traj)
    assert t['contact'] is True and t['outcome'] == 'REACHED_WITH_CONTACT' and t['success'] is False
    traj[1] = (5.0, face - lx / 2 - 0.02, 0.0, 0.0)               # 2 cm clearance: no contact
    t = analyze_one(tmp_path, 'k2', 's1_utrap', 'C2', traj)
    assert t['contact'] is False and t['outcome'] == 'REACHED'


def test_pocket_polygons_match_the_stated_outside_rule_for_s1():
    poly = an.POCKETS['s1_utrap']
    inside = lambda x, y: an.inside_polygon((x, y), poly)
    assert inside(0.2, 0.0) and inside(-0.99, 0.84) and inside(0.84, -0.84)
    assert not inside(-1.01, 0.0) and not inside(0.0, 0.86) and not inside(0.0, -0.86)
    assert not inside(-3.0, 0.0) and not inside(3.0, 0.0)


def test_s3_pocket_polygon_follows_the_wall_inner_faces():
    """The polygon edges lie on the inner faces of the three walls and the mouth line."""
    poly = an.POCKETS['s3_ctrap']
    boxes = dict(an.world_boxes(an.WORLDS['s3_ctrap']))
    # wall centre lines are at y = +-1.2 and x = 1.3 in the pocket frame, 0.3 thick: inner
    # faces at +-1.05 and 1.15 (an.S3_POCKET_FRAME). The rotated polygon corner (1.15, 1.05)
    # must touch the corner region of the walls c_back and c_top.
    corner = poly[2]
    assert an.pilot.rect_distance([corner, (corner[0] + 1e-6, corner[1]),
                                   (corner[0] + 1e-6, corner[1] + 1e-6), (corner[0], corner[1] + 1e-6)],
                                  boxes['c_back']) == pytest.approx(0.0, abs=1e-3)
    assert an.pilot.rect_distance([poly[1], (poly[1][0] + 1e-6, poly[1][1]),
                                   (poly[1][0] + 1e-6, poly[1][1] + 1e-6), (poly[1][0], poly[1][1] + 1e-6)],
                                  boxes['c_bot']) == pytest.approx(0.0, abs=1e-3)
    # the straight path from the start along y = 0 enters at the mouth and the start is outside
    assert not an.inside_polygon((-3.0, 0.0), poly)
    assert an.inside_polygon(_rot(0.5, 0.0), poly)
    assert an.inside_polygon(_rot(1.0, 0.0), poly)
    assert not an.inside_polygon(_rot(-0.4, 0.0), poly)
    assert not an.inside_polygon(_rot(0.5, 1.2), poly)       # inside the top wall
    assert not an.inside_polygon(_rot(1.3, 0.0), poly)       # inside the back wall


def _rot(x, y):
    return an._rotate((x, y), an.S3_PIVOT, an.S3_ROT_DEG)


def pocket_traj(enter_at, leave_at, end=100.0):
    """Start outside, in the s1 pocket from enter_at to leave_at, above the pocket
    (|y| > 0.85, clear of the wall) for 5 s, then at the goal."""
    traj = []
    for i in range(int(end * 10) + 1):
        t = i * 0.1
        if t < enter_at:
            traj.append((t, -3.0, 0.0, 0.0))
        elif leave_at is None or t < leave_at:
            traj.append((t, 0.2, 0.0, 0.0))
        elif t < leave_at + 5.0:
            traj.append((t, 0.2, 1.6, 0.0))
        else:
            traj.append((t, 3.0, 0.0, 0.0))
    return traj


def enter_line(t):
    return (f'[INFO] [{T0 + t:.6f}] [MPPIController]: SE escape ENTER at (0.2, 0.0): '
            'progress_events=1 arc=1.0 stall=30')


def test_enter_before_leaving_the_pocket(tmp_path):
    traj = pocket_traj(20.0, 50.0)
    t = analyze_one(tmp_path, 'e1', 's1_utrap', 'C4', traj, launch=enter_line(30.0))
    assert t['entered_pocket_s'] == pytest.approx(20.0, abs=0.11)
    assert t['left_pocket_s'] == pytest.approx(50.0, abs=0.11)
    assert t['enter_before_leaving_pocket'] is True
    assert t['reached_without_escape'] is False and t['success'] is True
    # ENTER after the chassis left: not before leaving
    t = analyze_one(tmp_path, 'e2', 's1_utrap', 'C4', traj, launch=enter_line(55.0))
    assert t['enter_before_leaving_pocket'] is False and t['reached_without_escape'] is True
    # no ENTER at all but reached: reached without escape
    t = analyze_one(tmp_path, 'e3', 's1_utrap', 'C4', traj, launch='')
    assert t['enter_before_leaving_pocket'] is False and t['reached_without_escape'] is True
    # exactly at the leaving sample counts as before
    t = analyze_one(tmp_path, 'e4', 's1_utrap', 'C4', traj, launch=enter_line(t['left_pocket_s']))
    assert t['enter_before_leaving_pocket'] is True
    # never left the pocket: any ENTER after entering counts; not reached
    stay = pocket_traj(20.0, None)
    t = analyze_one(tmp_path, 'e5', 's1_utrap', 'C4', stay, launch=enter_line(80.0), result='ABORTED')
    assert t['left_pocket_s'] is None and t['enter_before_leaving_pocket'] is True
    assert t['success'] is False and t['reached_without_escape'] is None
    # never entered the pocket: not applicable
    t = analyze_one(tmp_path, 'e6', 's1_utrap', 'C4', drive_to_goal(30.0, via=[(-2.0, 2.5)]))
    assert t['entered_pocket_s'] is None and t['enter_before_leaving_pocket'] is None


def test_stock_configurations_have_no_escape_flags(tmp_path):
    t = analyze_one(tmp_path, 's', 's1_utrap', 'C3', pocket_traj(20.0, 50.0), launch=enter_line(30.0))
    assert t['se_config'] is False
    assert t['enter_before_leaving_pocket'] is None and t['reached_without_escape'] is None


def test_host_load_validity_rule_marks_a_loaded_trial_invalid(tmp_path):
    t = analyze_one(tmp_path, 'h1', 's0_open', 'C1', drive_to_goal(60.0), foreign=1.0)
    assert t['outcome'] == 'REACHED'                          # exactly at the limit (<= 1.0)
    t = analyze_one(tmp_path, 'h2', 's0_open', 'C1', drive_to_goal(60.0), foreign=1.2)
    assert t['outcome'] == 'INVALID_HOST_LOAD' and t['outcome_measured'] == 'REACHED'
    assert t['success'] is False and 'foreign CPU' in t['invalid_reason']
    assert not an.is_valid_driven(t)


def test_invalid_and_setup_failure_records(tmp_path):
    root = str(tmp_path)
    boxes = an.world_boxes(an.WORLDS['s0_open'])
    chassis = an.pilot.chassis_size()
    d = make_trial(root, 'i1', 's0_open', 'C1', [(0.0, 0, 0, 0)], complete=False)
    assert an.analyze_trial(d, boxes, chassis)['outcome'] == 'INVALID'
    d = make_trial(root, 'i2', 's0_open', 'C1', [(0.0, 0, 0, 0)], active=False)
    assert an.analyze_trial(d, boxes, chassis)['outcome'] == 'SETUP_FAIL'
    t = analyze_one(tmp_path, 'i3', 's0_open', 'C1', drive_to_goal(20.0), no_goal=True)
    assert t['outcome'] == 'NO_GOAL_SENT'
    bad_start = drive_to_goal(20.0)
    bad_start = [(a, b + 1.0, c, d_) for a, b, c, d_ in bad_start]
    t = analyze_one(tmp_path, 'i4', 's0_open', 'C1', bad_start)
    assert t['outcome'] == 'INVALID' and 'from the start' in t['invalid_reason']


def test_escape_path_and_probe_fields_flow_into_the_record(tmp_path):
    ep = '\n'.join(json.dumps(x) for x in [
        {'kind': 'graph', 'tag': '10s', 'publishers': 1},
        {'kind': 'path', 'wall_s': T0 + 5, 'frame': 'odom', 'n_poses': 50},
        {'kind': 'path', 'wall_s': T0 + 6, 'frame': an.PROBE_FRAME, 'n_poses': 2}])
    t = analyze_one(tmp_path, 'p', 's0_open', 'C2', drive_to_goal(30.0), escape_path=ep)
    assert t['escape_path_messages'] == 1 and t['escape_path_probe_messages'] == 1
    assert t['escape_path_publishers_seen'] == 1


# --------------------------------------------------------------------------- #
# Statistics
# --------------------------------------------------------------------------- #
def test_wilson_interval_known_values():
    lo, hi = an.wilson(0, 6)
    assert lo == 0.0 and hi == pytest.approx(0.3903, abs=1e-3)
    lo, hi = an.wilson(6, 6)
    assert lo == pytest.approx(0.6097, abs=1e-3) and hi == 1.0
    lo, hi = an.wilson(3, 6)
    assert lo == pytest.approx(0.1876, abs=1e-3) and hi == pytest.approx(0.8124, abs=1e-3)
    assert an.wilson(0, 0) is None


def test_fisher_known_values_of_the_design():
    f = an.pilot.fisher_two_sided
    assert f(6, 0, 0, 6) == pytest.approx(0.002165, abs=1e-5)
    assert f(5, 1, 0, 6) == pytest.approx(0.01515, abs=1e-4)
    assert f(4, 2, 0, 6) == pytest.approx(0.06061, abs=1e-4)
    assert f(3, 3, 3, 3) == pytest.approx(1.0)


def test_mann_whitney_exact_and_edge_cases():
    r = an.mann_whitney([1.0, 2.0, 3.0], [4.0, 5.0, 6.0])
    assert r['method'] == 'exact' and r['U'] == 0 and r['p'] == pytest.approx(0.1)
    assert r['median1'] == 2.0 and r['median2'] == 5.0
    same = an.mann_whitney([1.0, 2.0], [1.0, 2.0])
    assert same['p'] == pytest.approx(1.0)
    assert an.mann_whitney([], [1.0]) is None and an.mann_whitney([1.0], []) is None
    big = an.mann_whitney(list(map(float, range(30))), list(map(float, range(15, 45))))
    assert big['method'] == 'normal' and 0.0 < big['p'] < 1.0


def rec(scenario, config, outcome, **kw):
    base = {'trial_id': f'{scenario}__{config}__{len(kw)}{outcome}', 'scenario': scenario,
            'config': config, 'outcome': outcome, 'success': outcome == 'REACHED'}
    base.update(kw)
    return base


def test_cell_stats_count_only_valid_trials():
    trials = [rec('s1_utrap', 'C4', 'REACHED', time_to_goal_wall_s=100.0, escape_enter=2),
              rec('s1_utrap', 'C4', 'REACHED_WITH_CONTACT'),
              rec('s1_utrap', 'C4', 'INVALID_HOST_LOAD'),
              rec('s1_utrap', 'C4', 'NOT_REACHED_TIMEOUT', escape_enter=1)]
    c = an.cell_stats(trials)
    assert (c['n_trials'], c['n_valid'], c['successes']) == (4, 3, 1)
    assert c['reached_with_contact'] == 1
    assert c['escape_enter_total'] == 3 and c['trials_with_escape_enter'] == 2
    assert c['time_to_goal_wall_s'] == [100.0]
    assert c['outcomes']['INVALID_HOST_LOAD'] == 1


def test_contrasts_use_valid_counts_and_report_none_for_empty_cells():
    cells = {('s1_utrap', 'C4'): an.cell_stats([rec('s1_utrap', 'C4', 'REACHED', time_to_goal_wall_s=90.0 + i)
                                                for i in range(6)]),
             ('s1_utrap', 'C3'): an.cell_stats([rec('s1_utrap', 'C3', 'NOT_REACHED_ABORTED') for _ in range(6)])}
    rows = {r['name']: r for r in an.contrasts(cells)}
    r = rows['C4 vs C3 on s1_utrap']
    assert (r['a_successes'], r['a_n'], r['b_successes'], r['b_n']) == (6, 6, 0, 6)
    assert r['fisher_p'] == pytest.approx(0.002165, abs=1e-5) and r['mann_whitney'] is None
    assert rows['C4 vs C3 on s3_ctrap']['fisher_p'] is None
    assert len(rows) == 5


def test_h3_sums_the_control_cells():
    cells = {('s0_open', 'C2'): an.cell_stats([rec('s0_open', 'C2', 'REACHED', escape_enter=0)] * 6),
             ('s0_open', 'C4'): an.cell_stats([rec('s0_open', 'C4', 'REACHED', escape_enter=1)] * 5 +
                                              [rec('s0_open', 'C4', 'NOT_REACHED_ABORTED')])}
    h = an.h3(cells)
    assert (h['reached'], h['n_valid'], h['escape_enter_total']) == (11, 12, 5)


# --------------------------------------------------------------------------- #
# Stopping rules
# --------------------------------------------------------------------------- #
def cell_trials(scenario, config, k, n=6):
    return [rec(scenario, config, 'REACHED' if i < k else 'NOT_REACHED_ABORTED') for i in range(n)]


def test_phase_a_gate_stops_at_one_of_six_or_fewer_only():
    for k, stop in ((0, True), (1, True), (2, False), (6, False)):
        stops = an.evaluate_stops(cell_trials('s1_utrap', 'C4', k))
        assert any(c == 'PHASE_A_GATE' for c, _ in stops) is stop, k
    # not decided before all six are in
    assert an.evaluate_stops(cell_trials('s1_utrap', 'C4', 0, n=5)) == []
    # an invalid attempt does not count as one of the six
    mixed = cell_trials('s1_utrap', 'C4', 0, n=5) + [rec('s1_utrap', 'C4', 'INVALID_HOST_LOAD')]
    assert an.evaluate_stops(mixed) == []


def test_stack_broken_rule_on_the_stock_controller_on_s0():
    for cfg in ('C1', 'C3'):
        assert any(c == 'STACK_BROKEN' for c, _ in an.evaluate_stops(cell_trials('s0_open', cfg, 4)))
        assert not an.evaluate_stops(cell_trials('s0_open', cfg, 5))
    assert not an.evaluate_stops(cell_trials('s0_open', 'C2', 0))      # SE cells are not this rule
    assert not an.evaluate_stops(cell_trials('s0_open', 'C1', 0, n=5))


def test_invalid_cap_and_setup_failure_stop():
    assert not an.evaluate_stops([], {'s1_utrap/C1': 6})
    assert any(c == 'INVALID_CAP' for c, _ in an.evaluate_stops([], {'s1_utrap/C1': 7}))
    assert any(c == 'SETUP_FAIL' for c, _ in an.evaluate_stops([rec('s0_open', 'C1', 'SETUP_FAIL')]))


# --------------------------------------------------------------------------- #
# Whole analysis
# --------------------------------------------------------------------------- #
def build_raw(tmp_path):
    root = str(tmp_path / 'raw')
    for rep in range(6):
        make_trial(root, f'correct__s1_utrap__C4__rep{rep}', 's1_utrap', 'C4',
                   pocket_traj(20.0, 50.0), launch=enter_line(30.0), rep=rep)
        make_trial(root, f'correct__s1_utrap__C3__rep{rep}', 's1_utrap', 'C3',
                   stuck_in_pocket(60.0), result='ABORTED', drive_s=60.0, rep=rep)
    make_trial(root, 'correct__s1_utrap__C4__rep0.invalid1', 's1_utrap', 'C4',
               pocket_traj(20.0, 50.0), foreign=2.0)
    make_trial(root, 'correct__s1_utrap_evidence__C2__rep0', 's1_utrap_evidence', 'C2',
               pocket_traj(0.0, 40.0), launch=enter_line(10.0), start=(0.0, 0.0))
    make_trial(root, 'correct__s0_open__C2__rep0', 's0_open', 'C2', drive_to_goal(60.0),
               sub='shakedown')
    return root


def test_build_separates_registered_superseded_evidence_and_shakedown(tmp_path):
    result = an.build(build_raw(tmp_path))
    assert len(result['trials']) == 13                              # 12 + 1 evidence
    assert [t['trial_id'] for t in result['superseded_attempts']] == ['correct__s1_utrap__C4__rep0.invalid1']
    assert [t['trial_id'] for t in result['shakedown']] == ['correct__s0_open__C2__rep0']
    cells = {(c['scenario'], c['config']): c for c in result['cells']}
    assert set(cells) == {('s1_utrap', 'C4'), ('s1_utrap', 'C3')}
    assert cells[('s1_utrap', 'C4')]['successes'] == 6 and cells[('s1_utrap', 'C3')]['successes'] == 0
    assert result['evidence_cells'][0]['scenario'] == 's1_utrap_evidence'
    row = next(r for r in result['contrasts'] if r['name'] == 'C4 vs C3 on s1_utrap')
    assert row['fisher_p'] == pytest.approx(0.002165, abs=1e-5)
    assert result['stops'] == []
    assert cells[('s1_utrap', 'C4')]['trials_with_escape_enter'] == 6


def test_registered_cells_exclude_the_shakedown_and_the_invalid_attempt(tmp_path):
    root = build_raw(tmp_path)
    recs = an.registered_records(root)
    assert all('.' not in t['trial_id'] for t in recs)
    assert not any(t['scenario'] == 's0_open' for t in recs)


def test_render_and_check_round_trip(tmp_path):
    root = build_raw(tmp_path)
    out = tmp_path / 'out'
    out.mkdir()
    import subprocess
    import sys
    cmd = [sys.executable, os.path.join(an.HERE, 'analyze.py'), '--raw', root, '--out', str(out)]
    subprocess.run(cmd + ['--pack'], check=True, capture_output=True)
    assert subprocess.run(cmd + ['--check'], capture_output=True).returncode == 0
    md = (out / 'RESULTS.md').read_text()
    assert 'correct__s1_utrap__C4__rep0' in md and 'Shakedown' in md and 'Fisher' in md
    # results.json.gz is deterministic and parses
    with gzip.open(out / 'results.json.gz', 'rt') as f:
        data = json.load(f)
    assert data['cells'][0]['scenario'] == 's1_utrap'
    assert len(list((out / 'logs').glob('*.log.gz'))) == 15      # 14 in trials/ (12 + invalid + evidence) + shakedown
    # a change in the raw data makes --check fail
    make_trial(root, 'correct__s0_open__C1__rep0', 's0_open', 'C1', drive_to_goal(30.0))
    assert subprocess.run(cmd + ['--check'], capture_output=True).returncode == 1
    # missing outputs also fail
    (out / 'RESULTS.md').unlink()
    assert subprocess.run(cmd + ['--check'], capture_output=True).returncode == 1


def test_gz_output_is_byte_identical_across_runs():
    a, b = an.gz_bytes('abc' * 100), an.gz_bytes('abc' * 100)
    assert a == b


def test_world_boxes_of_the_campaign_worlds():
    s0 = dict(an.world_boxes(an.WORLDS['s0_open']))
    s1 = dict(an.world_boxes(an.WORLDS['s1_utrap']))
    s3 = dict(an.world_boxes(an.WORLDS['s3_ctrap']))
    assert not any(n.startswith('u_') for n in s0) and not any(n.startswith('u_') for n in s3)
    assert {'u_back', 'u_top', 'u_bot'} <= set(s1)
    assert {'c_back', 'c_top', 'c_bot'} <= set(s3) and not any(n.startswith('c_') for n in s0)
    assert len(s0) == 4 and len(s3) == 7 and len(s1) == 7


def test_regexes_have_no_accidental_overlap_with_pilot_names():
    # the analyzer's ENTER pattern accepts the campaign-1 prefix and the new suffix alike
    old = '[INFO] [100.5] [MPPIController]: SE escape ENTER at (1.00, 2.00): progress_idx=0 stall=30'
    m = an.ENTER_RE.search(old)
    assert m and (m.group('x'), m.group('y')) == ('1.00', '2.00')
    assert re.search(r'\(', an.ENTER_RE.pattern)
