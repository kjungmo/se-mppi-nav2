"""Campaign 2 computes REACHED, contact and the distances exactly as campaign 1.

``analyze.analyze_trial`` of campaign 2 re-implements the outcome logic of
``results_pilot_rerun/analyze.py``; it does not call it. PREREG.md says the two agree. This test
fixes that claim: 300 random synthetic trials over s0, s1 and s3 (including trials that the
host-load rule invalidates) and boundary cases give the same outcome, ``reached``, ``contact``,
final distance and minimum wall distance in both. A later edit of either file that changes one of
them fails here.
"""

import json
import math
import os
import random

import pytest

import analyze as an
from test_c2_analyze import make_trial

PILOT = an.pilot
SCENARIOS = ('s0_open', 's1_utrap', 's3_ctrap')
COMPARED = ('outcome', 'reached', 'contact', 'final_goal_distance_m', 'min_wall_distance_m',
            'nearest_wall', 'nav2_false_success', 'drive_wall_s', 'final_gt_xy', 'rtf')


def pilot_meta(tdir):
    """Add the fields that the campaign-1 analyzer reads from meta.json."""
    path = os.path.join(tdir, 'meta.json')
    with open(path) as f:
        meta = json.load(f)
    meta.update(env_label='correct', param_set='V', override_guard=None,
                env={'package_builds': ['nav2-mppi-controller-1.3.13-np2py312_25'],
                     'check_mppi_rollout_exit': 0, 'repo_commit': 'abc123'})
    with open(path, 'w') as f:
        json.dump(meta, f)


def both(tmp_path, name, scenario, traj, **kw):
    d = make_trial(str(tmp_path), name, scenario, 'C2', traj, **kw)
    pilot_meta(d)
    boxes = an.world_boxes(an.WORLDS[scenario])
    chassis = PILOT.chassis_size()
    return an.analyze_trial(d, boxes, chassis), PILOT.analyze_trial(d, boxes, chassis)


def compare(c2, c1, label):
    assert c2['outcome'] == c1['outcome'], (label, c2['outcome'], c1['outcome'])
    for k in COMPARED:
        if k in c1 or k in c2:
            assert c2.get(k) == pytest.approx(c1.get(k)), (label, k, c2.get(k), c1.get(k))


def random_traj(rng, scenario):
    pts = [(-3.0, 0.0)]
    n_way = rng.choice([1, 2, 3, 4])
    for _ in range(n_way):
        pts.append((rng.uniform(-3.2, 3.2), rng.uniform(-2.2, 2.2)))
    end = rng.choice([(3.0, 0.0), (3.0, 0.0), (rng.uniform(1.0, 3.0), rng.uniform(-1.0, 1.0)),
                      (3.0 + rng.choice([-0.49, -0.51, 0.3]), 0.0), pts[-1]])
    pts.append(end)
    traj, t = [], 0.0
    yaw = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        steps = max(2, int(math.hypot(x1 - x0, y1 - y0) / rng.uniform(0.03, 0.08)))
        for i in range(steps):
            f = i / steps
            yaw = math.atan2(y1 - y0, x1 - x0)
            traj.append((t, x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw))
            t += 0.1
    traj.append((t, end[0], end[1], yaw))
    return traj


def test_300_random_synthetic_trials_agree_with_campaign_1(tmp_path):
    rng = random.Random(20261018)
    outcomes = {}
    for i in range(300):
        scenario = SCENARIOS[i % 3]
        traj = random_traj(rng, scenario)
        kw = dict(result=rng.choice(['SUCCEEDED', 'SUCCEEDED', 'ABORTED', 'CANCELED']),
                  watchdog=rng.random() < 0.15, foreign=rng.choice([0.1, 0.1, 0.1, 2.0, 4.5]),
                  drive_s=traj[-1][0] + rng.choice([0.0, 0.0, 5.0]))
        c2, c1 = both(tmp_path, f'rand{i}', scenario, traj, **kw)
        compare(c2, c1, f'rand{i} {scenario}')
        outcomes[c2['outcome']] = outcomes.get(c2['outcome'], 0) + 1
    # the random trials reach every outcome of the logic under test
    for needed in ('REACHED', 'REACHED_WITH_CONTACT', 'NOT_REACHED_ABORTED', 'NOT_REACHED_TIMEOUT',
                   'INVALID_HOST_LOAD'):
        assert outcomes.get(needed, 0) >= 5, outcomes


@pytest.mark.parametrize('dx,expected', [(0.499, True), (0.4999, True), (0.5001, False), (0.501, False)])
def test_goal_tolerance_boundary_agrees(tmp_path, dx, expected):
    traj = [(0.0, -3.0, 0.0, 0.0), (10.0, 3.0 - dx, 0.0, 0.0)]
    c2, c1 = both(tmp_path, f'g{dx}', 's0_open', traj)
    compare(c2, c1, f'goal {dx}')
    assert c2['reached'] is expected


@pytest.mark.parametrize('gap,contact', [(0.0095, True), (0.0099, True), (0.0101, False), (0.0105, False)])
def test_contact_boundary_agrees(tmp_path, gap, contact):
    lx, _ = PILOT.chassis_size()
    face = 0.85                                   # back wall face of the s1 testbed
    traj = [(0.0, -3.0, 0.0, 0.0), (5.0, face - lx / 2 - gap, 0.0, 0.0), (10.0, 3.0, 0.0, 0.0)]
    c2, c1 = both(tmp_path, f'c{gap}', 's1_utrap', traj)
    compare(c2, c1, f'contact {gap}')
    assert c2['contact'] is contact


def test_host_load_validity_agrees(tmp_path):
    traj = [(0.0, -3.0, 0.0, 0.0), (60.0, 3.0, 0.0, 0.0)]
    for foreign, valid in ((0.9, True), (1.1, False)):
        c2, c1 = both(tmp_path, f'h{foreign}', 's0_open', traj, foreign=foreign)
        compare(c2, c1, f'host {foreign}')
        assert (c2['outcome'] != 'INVALID_HOST_LOAD') is valid


def test_the_analyzers_share_their_tolerances():
    assert an.GOAL_TOL == PILOT.GOAL_TOL == 0.5
    assert an.CONTACT_TOL == PILOT.CONTACT_TOL == 0.01
    assert an.START_TOL == PILOT.START_TOL
    assert an.RTF_WINDOW_S == PILOT.RTF_WINDOW_S
