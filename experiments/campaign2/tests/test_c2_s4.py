"""Tests for the second held-out pocket (the L-shaped dead end, PREREG.md amendment 1,
LEAD_DECISIONS_2 E-2): its pocket polygon against the world file, its place outside the
development family, the extended decision rule and the refusal before Phase A."""

import math
import os
import re
import xml.etree.ElementTree as ET

import pytest

import analyze as an
import plan
import run_campaign2 as drv
from test_c2_analyze import cell_trials, rec
from test_c2_review_fixes import phase_a_records

S4 = 's4_ltrap'
MOUTH = (-0.6, 0.0)
YAW = math.radians(-15.0)
DEV_GEOMETRIES = os.path.join(plan.REPO, 'src', 'nav2_se_controller', 'test', 'deployment',
                              'UTRAP_DEV_GEOMETRIES.md')


def to_world(p):
    c, s = math.cos(YAW), math.sin(YAW)
    return (MOUTH[0] + c * p[0] - s * p[1], MOUTH[1] + s * p[0] + c * p[1])


def to_pocket(p):
    c, s = math.cos(-YAW), math.sin(-YAW)
    dx, dy = p[0] - MOUTH[0], p[1] - MOUTH[1]
    return (c * dx - s * dy, s * dx + c * dy)


def walls():
    """name -> (x, y, yaw, length, thickness) of the l_* boxes of the s4 world file."""
    out = {}
    for model in ET.parse(an.WORLDS[S4]).getroot().iter('model'):
        if not model.get('name').startswith('l_'):
            continue
        x, y, _, _, _, yaw = (float(v) for v in model.find('pose').text.split())
        size = [float(v) for v in model.find('./link/collision/geometry/box/size').text.split()]
        out[model.get('name')] = (x, y, yaw, size[0], size[1])
    return out


def in_a_wall(pt):
    return any(an.inside_polygon(pt, an.pilot.rect(*b)) for b in
               ((x, y, yaw, L, T) for x, y, yaw, L, T in walls().values()))


# --------------------------------------------------------------------------- #
# The pocket polygon
# --------------------------------------------------------------------------- #
def test_the_s4_polygon_is_not_convex_and_the_convex_test_is_kept_for_convex_ones():
    assert not an._convex(an.POCKETS[S4])
    assert an._convex(an.POCKETS['s1_utrap']) and an._convex(an.POCKETS['s3_ctrap'])


@pytest.mark.parametrize('pocket_xy,inside', [
    ((1.0, 0.0), True),                 # corridor
    ((1.8, 1.2), True),                 # leg
    ((1.8, 0.65), True),                # corridor and leg meet
    ((0.0, 0.0), True),                 # mouth plane: boundary counts as inside
    ((2.05, 0.0), True),                # back wall face: boundary on the right side
    ((1.8, 1.85), True),                # cap face
    ((1.55, 1.0), True),                # inner wall face of the leg
    ((1.0, 1.2), False),                # the notch beside the corridor: outside, behind the top wall
    ((1.0, 0.9), False),                # on the top wall
    ((-0.2, 0.0), False),               # in front of the mouth
    ((2.3, 0.0), False),                # behind the back wall
    ((1.8, 2.1), False),                # beyond the cap
])
def test_the_polygon_separates_corridor_and_leg_from_the_outside(pocket_xy, inside):
    assert an.inside_polygon(to_world(pocket_xy), an.POCKETS[S4]) is inside


def edge_distance(pt, poly):
    best = math.inf
    for a, b in zip(poly, poly[1:] + poly[:1]):
        ex, ey = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((pt[0] - a[0]) * ex + (pt[1] - a[1]) * ey) / (ex * ex + ey * ey)))
        best = min(best, math.hypot(pt[0] - a[0] - t * ex, pt[1] - a[1] - t * ey))
    return best


def test_the_polygon_is_the_free_space_the_walls_enclose():
    poly = an.POCKETS[S4]
    n_in = 0
    for i in range(-30, 90):
        for j in range(-30, 90):
            pw = to_world((i * 0.025, j * 0.025))
            if an.inside_polygon(pw, poly):
                n_in += 1
                # a point more than 0.03 m inside the polygon is free of walls
                if edge_distance((i * 0.025, j * 0.025), an.S4_POCKET_FRAME) > 0.03:
                    assert not in_a_wall(pw), (i * 0.025, j * 0.025)
    assert n_in > 1000
    # every edge but the mouth lies on a wall: a point 0.05 m outside the edge is inside a wall
    p = an.S4_POCKET_FRAME
    for k, (a, b) in enumerate(zip(p, p[1:] + p[:1])):
        if a == (0.0, 0.65) and b == (0.0, -0.65) or {a, b} == {(0.0, -0.65), (0.0, 0.65)}:
            continue                                                  # the mouth edge
        ex, ey = b[0] - a[0], b[1] - a[1]
        length = math.hypot(ex, ey)
        nx, ny = ey / length, -ex / length                            # outward normal of a CCW polygon
        for t in (0.25, 0.5, 0.75):
            q = (a[0] + t * ex + 0.05 * nx, a[1] + t * ey + 0.05 * ny)
            assert in_a_wall(to_world(q)), (a, b, q)
    # and the mouth is open: a point in front of the mouth is free
    assert not in_a_wall(to_world((-0.05, 0.0)))


def test_the_polygon_leaves_the_pocket_at_the_mouth_for_a_trial_that_drives_out():
    rows = [{'gt_x': to_world(p)[0], 'gt_y': to_world(p)[1], 'wall_s': float(i)}
            for i, p in enumerate([(-1.0, 0.0), (0.5, 0.0), (1.8, 0.2), (1.8, 1.2), (1.0, 0.0),
                                   (-0.3, 0.0), (-1.0, 0.0)])]
    assert an.pocket_times(rows, S4) == (1.0, 5.0)
    # a path that stays on the notch side never enters
    rows = [{'gt_x': to_world(p)[0], 'gt_y': to_world(p)[1], 'wall_s': float(i)}
            for i, p in enumerate([(-1.0, 1.2), (1.0, 1.2), (2.5, 1.2)])]
    assert an.pocket_times(rows, S4) == (None, None)


# --------------------------------------------------------------------------- #
# The world is outside the development family
# --------------------------------------------------------------------------- #
def family_values():
    with open(DEV_GEOMETRIES) as f:
        text = f.read()
    out = {}
    for key, label in (('d', 'depth'), ('w', 'width'), ('theta', 'rotation')):
        m = re.search(rf'\| {label} `[^`]*` \([^)]*\) \| ([^|]*) \|', text)
        out[key] = [float(v) for v in m.group(1).split(',')]
    return out


def s4_depth_and_width():
    """Inner depth of the corridor (mouth plane to the inner face of the back wall) and inner width."""
    w = walls()
    bx, by = to_pocket((w['l_back'][0], w['l_back'][1]))
    depth = bx - w['l_back'][4] / 2
    tx, ty = to_pocket((w['l_top'][0], w['l_top'][1]))
    bt = to_pocket((w['l_bot'][0], w['l_bot'][1]))
    width = (ty - w['l_top'][4] / 2) - (bt[1] + w['l_bot'][4] / 2)
    return depth, width


def test_the_family_table_is_read_as_the_documented_values():
    f = family_values()
    assert f['d'] == [1.1, 1.5, 1.8] and f['w'] == [1.3, 1.6, 2.2, 2.6]
    assert f['theta'] == [-25.0, 0.0, 15.0, 30.0]


def test_the_s4_corridor_is_deeper_than_every_development_pocket():
    depth, width = s4_depth_and_width()
    assert depth == pytest.approx(2.05, abs=1e-6)
    assert width == pytest.approx(1.3, abs=1e-6)
    f = family_values()
    assert depth > max(f['d'])
    assert depth - max(f['d']) == pytest.approx(0.25, abs=1e-6)
    # the width and the rotation lie inside the family's span; the depth and the bend do not
    assert min(f['w']) <= width <= max(f['w'])
    assert min(f['theta']) <= -15.0 <= max(f['theta'])
    # the leg has no counterpart: the family's pocket is one rectangle, this one has 5 walls
    assert len(walls()) == 5


def test_the_straight_line_enters_the_corridor_and_ends_on_the_back_wall():
    depth, _ = s4_depth_and_width()
    # y = 0 crosses the mouth plane at the mouth centre and meets the back wall's inner face at
    # pocket y = +depth * tan(15 deg), on the leg side, inside the corridor's inner half width of 0.65
    x_back = MOUTH[0] + depth / math.cos(YAW)
    px, py = to_pocket((x_back, 0.0))
    assert px == pytest.approx(depth, abs=1e-9)
    assert py == pytest.approx(depth * math.tan(math.radians(15.0)), abs=1e-9)
    assert 0 < py < 0.65
    assert x_back - MOUTH[0] == pytest.approx(2.1223, abs=1e-3)


# --------------------------------------------------------------------------- #
# Plan, refusal, decision
# --------------------------------------------------------------------------- #
def test_the_plan_runs_only_c3_and_c4_on_s4_in_block_b():
    s4 = [t for t in plan.plan('all') if t.scenario == S4]
    assert len(s4) == 12 and {t.config for t in s4} == {'C3', 'C4'}
    assert {t.phase for t in s4} == {'B'} and all(t.counted for t in s4)
    assert sorted((t.config, t.rep) for t in s4) == sorted(
        (c, r) for c in ('C3', 'C4') for r in range(6))
    assert len(plan.plan('all')) == 88 and len(plan.plan('B')) == 79


def test_every_s4_trial_waits_for_phase_a(monkeypatch, tmp_path):
    monkeypatch.setattr(an, 'registered_records', lambda raw: [])
    for phase in ('A', 'B'):
        assert 'not complete' in drv.phase_gate_refusal(str(tmp_path), plan.Trial(S4, 'C4', 0, phase))
    assert 'stay unseen' in an.s3_refusal(phase_a_records(0))
    monkeypatch.setattr(an, 'registered_records', lambda raw: phase_a_records(0))
    assert 'gate fired' in drv.phase_gate_refusal(str(tmp_path), plan.Trial(S4, 'C4', 0, 'B'))
    monkeypatch.setattr(an, 'registered_records', lambda raw: phase_a_records(5))
    assert drv.phase_gate_refusal(str(tmp_path), plan.Trial(S4, 'C4', 0, 'B')) is None


def cells_of(**counts):
    """{(scenario, config): cell_stats} from keyword counts like s1_C4=6 (reached of 6)."""
    recs = []
    for key, k in counts.items():
        scen = {'s1': 's1_utrap', 's3': 's3_ctrap', 's4': S4}[key.split('_')[0]]
        recs += cell_trials(scen, key.split('_')[1], k)
    groups = {}
    for t in recs:
        groups.setdefault((t['scenario'], t['config']), []).append(t)
    return {k: an.cell_stats(v) for k, v in groups.items()}


@pytest.mark.parametrize('k4,k3,yes', [
    (6, 0, True), (5, 0, True), (4, 0, False),          # p 0.0022, 0.0152, 0.0606
    (6, 1, True), (5, 1, False), (0, 0, False),
    (0, 4, False), (3, 3, False)])                       # more often than C3 is needed as well
def test_yes_on_a_pocket_needs_p_below_005_and_more_reached_than_c3(k4, k3, yes):
    d = an.decision(cells_of(s1_C4=k4, s1_C3=k3, s4_C4=k4, s4_C3=k3))
    assert d['s1']['yes'] is yes and d['s4']['yes'] is yes


def test_the_opposite_direction_is_not_yes_even_when_p_is_small():
    # C3 reached 6 of 6 and C4 0 of 6: p = 0.0022 but C4 did not reach more often
    d = an.decision(cells_of(s1_C4=0, s1_C3=6, s4_C4=0, s4_C3=6))
    assert d['s1']['fisher_p'] < 0.05 and not d['s1']['yes'] and not d['s4']['yes']


@pytest.mark.parametrize('s1,s4,s3,branch,note', [
    ((6, 0), (6, 0), (6, 0), 'demonstrated', False),
    ((6, 0), (5, 0), (0, 0), 'demonstrated', True),          # s3 'no' beside s1 and s4 'yes': reported
    ((6, 0), (0, 0), (6, 0), 's1_yes_s4_no', False),         # s3 'yes' does not rescue the transfer
    ((6, 0), (4, 0), (6, 0), 's1_yes_s4_no', False),         # p 0.0606 is 'no'
    ((0, 0), (6, 0), (6, 0), 'not_demonstrated', False),     # s4 'yes' does not rescue s1 'no'
    ((4, 0), (6, 0), (6, 0), 'not_demonstrated', False),
])
def test_the_branch_follows_s1_and_s4_and_s3_does_not_change_it(s1, s4, s3, branch, note):
    cells = cells_of(s1_C4=s1[0], s1_C3=s1[1], s4_C4=s4[0], s4_C3=s4[1], s3_C4=s3[0], s3_C3=s3[1])
    d = an.decision(cells)
    assert d['branch'] == branch and d['s3_note'] is note and d['decided'] is True


def test_the_decision_is_not_final_before_six_valid_trials_in_each_pocket_cell():
    cells = cells_of(s1_C4=6, s1_C3=0, s4_C4=6, s4_C3=0)
    cells[(S4, 'C3')] = an.cell_stats([rec(S4, 'C3', 'NOT_REACHED_ABORTED') for _ in range(5)])
    assert an.decision(cells)['decided'] is False
    assert an.decision({})['branch'] == 'not_demonstrated' and an.decision({})['decided'] is False


def test_contrasts_include_s4_and_the_transfer_contrast_to_s1():
    names = [c[0] for c in an.CONTRASTS]
    assert 'C4 vs C3 on s4_ltrap' in names
    assert 'C4 on s4_ltrap vs C4 on s1_utrap (transfer)' in names
    rows = {r['name']: r for r in an.contrasts(cells_of(s1_C4=6, s1_C3=0, s4_C4=5, s4_C3=0))}
    assert rows['C4 vs C3 on s4_ltrap']['fisher_p'] == pytest.approx(0.0152, abs=5e-5)
    assert rows['C4 on s4_ltrap vs C4 on s1_utrap (transfer)']['a_successes'] == 5


def test_the_rendered_results_name_the_branch():
    result = {'cells': [], 'evidence_cells': [], 'contrasts': [], 'trials': [], 'superseded_attempts': [],
              'shakedown': [], 'stops': [], 'h3_control': {'reached': 0, 'n_valid': 0, 'escape_enter_total': 0},
              'decision': an.decision(cells_of(s1_C4=6, s1_C3=0, s4_C4=2, s4_C3=0))}
    text = an.render(result)
    assert 'Branch: `s1_yes_s4_no`' in text and 's4_ltrap (outside the family)' in text


# --------------------------------------------------------------------------- #
# The pre-registration text against the plan and the decision function
# --------------------------------------------------------------------------- #
def prereg():
    from test_c2_review_fixes import flat, prereg_text
    return flat(prereg_text())


def test_the_prereg_states_the_plan_size_the_plan_has():
    text = prereg()
    counts = re.findall(r'\*\*(\d+) trials\*\*', text)
    assert len(counts) >= 2 and set(counts) == {str(len(plan.plan('all')))}, counts   # every mention
    assert f'**Phase B ({len(plan.plan("B"))} trials).**' in text
    assert f'**Phase A (gate, {len(plan.plan("A"))} trials).**' in text
    assert 'C1 and C2 do not run on s4' in text


def test_the_prereg_decision_rule_matches_the_function():
    text = prereg()
    assert f'is below {an.ALPHA}' in text
    assert 'Seven contrasts are computed' in text and len(an.CONTRASTS) == 7
    assert an.DECISION_POCKETS == ('s1_utrap', 's4_ltrap')
    assert 'only two of them (C4 against C3 on `s1_utrap`, and on `s4_ltrap`) take part' in text


def test_the_prereg_ablation_counts_are_the_measured_ones():
    text = prereg()
    for needle in ('167 of 168 runs pass', '16, 18 and 14 of 56', '48 of 168', 'commit `b7e012a`'):
        assert needle.lower() in text.lower(), needle
