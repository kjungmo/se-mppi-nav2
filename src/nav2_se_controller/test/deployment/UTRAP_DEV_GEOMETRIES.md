# Development geometries for the closed-loop U-pocket test

Written and committed BEFORE the first run of `test_closed_loop_utrap`
(decision D-3 of the SE-MPPI v2 plan). The generator that implements this rule
is `utrap_dev_geometries.hpp`. The rule must not change after a test run has
looked at the results: a change needs a dated amendment at the end of this file.

## Purpose

The closed-loop test needs pockets the controller was never tuned on. The live
scenario `s1_utrap` (a 1.7 m wide, 1.85 m deep pocket) is excluded. The held-out
campaign geometry did not appear in this repository outside `experiments/campaign2/` until the
campaign ended on 2026-10-22. Since then `test_heldout_guard` allows the world names only in
`experiments/campaign2/`, `docs/papers/` and `scripts/check_paper_numbers.py`, and fails if they
appear anywhere else (controller sources, the development family generator, the headless test
worlds, tuning configs, tests).

## Frames

* World frame `map`. The robot starts at S = (-2.6, 0.0) facing +x. The goal is
  G = (3.0, 0.0). The BT stand-in replans a straight line from the robot to G.
* Pocket frame: origin B = (0.9, 0) on the inner face of the back wall, axes
  rotated by `theta` about B. The pocket interior is the rectangle
  `u in [-d, 0]`, `|v| <= w/2`. The mouth is the side `u = -d`. The back wall
  is at `u = 0`.
* Walls are line segments with a half thickness h = 0.075 m:
  * back wall centre line `u = +h`, `v in [-(w/2 + 2h), +(w/2 + 2h)]`;
  * top side wall `v = +(w/2 + h)`, `u in [-d * ft, +2h]`;
  * bottom side wall `v = -(w/2 + h)`, `u in [-d * fb, +2h]`.

## Parameters of the family

| Parameter | Values |
|---|---|
| width `w` (m) | 1.3, 1.6, 2.2, 2.6 |
| depth `d` (m) | 1.1, 1.5, 1.8 |
| rotation `theta` (deg) | -25, 0, 15, 30 |
| side-wall length factors `(ft, fb)` | (1.0, 1.0) symmetric, (0.6, 1.0) asymmetric |

That is 4 x 3 x 4 x 2 = 96 combinations. A combination is a valid geometry only if
all of these hold (the generator computes them; nothing is tuned by hand):

1. V1 (the straight path enters through the mouth): `|d * tan(theta)| <= w/2 - 0.3`.
2. V2 (start and goal are in free space): S and G are at least 0.8 m from every wall surface.
3. V3 (the straight line S to G is blocked): it comes within 0.22 m of a wall surface.
4. V4 (not s1): not (`|w - 1.7| < 0.1` and `|d - 1.85| < 0.1`).

The generator finds 56 valid geometries (checked before any controller run). They are listed in lexicographic order of the name
`w<w*10>_d<d*10>_r<theta>_a<ft*10>` (for example `w22_d13_r30_a10`).

## Costmap drawn from a geometry

The test draws a 3 m x 3 m rolling window (60 x 60 cells of 0.05 m) centred on the
robot, like the live local costmap. With `s` the distance from a cell centre to the
nearest wall surface (centre-line distance minus h):

* `s <= 0`: cost 254;
* `0 < s <= 0.22` (the robot radius): cost 253;
* `0.22 < s < 0.45` (the inflation radius): linear from 252 down to 1;
* otherwise 0.

## Which geometries gate the test

* PRIMARY (each must pass 3 of 3 runs): `w22_d13_r30_a10` (symmetric, rotated 30
  degrees) and `w16_d15_r-15_a6` (asymmetric, top wall 60 percent). Both are fixed
  here, before any run. They are not points of the 96-point grid (d = 1.3 and
  theta = -15 are not grid values); both satisfy V1 to V4, and the test checks that.
* SWEEP (informational, run with `SE_UTRAP_SWEEP=1`, one run each, results reported
  and not gating): every 7th valid geometry in the list, starting at index 3.
* Control: the CURRENT controller (before the fixes) must fail the primary geometry
  `w22_d13_r30_a10`: the robot centre stays in the pocket for the whole 60 s.

## Pass criteria for one run

* The robot centre leaves the pocket rectangle within 60 s of simulated time.
* The robot centre is within 0.5 m of G within 150 s of simulated time.
* The run has no wall contact (the harness stops the robot from entering a wall's
  0.22 m robot radius and counts every blocked step as a contact).

## Amendments

2026-10-18, after the first run of the harness (a harness defect, no controller result used):
the run starts outside the pocket, so "leaves the pocket" counts only after the robot centre
has been inside the rectangle once. A run in which the robot never enters the pocket is a
failed run (the geometry did not test a trap). The criteria and geometries are unchanged.
