# Pre-registration: campaign 2, the live U-pocket test of the fixed SE-MPPI controller

**Status: FROZEN on 2026-10-18.** The controller, the parameter file, the worlds, the plan and the
analysis code are fixed at base commit `7e629d2` and listed with their sha256 in
`FREEZE_MANIFEST.json`. The freeze commit is the commit that holds this file and the manifest. It
carries the annotated tag `campaign2-controller` (`git rev-parse campaign2-controller^{commit}`); a
commit cannot name its own hash, so the tag is the pointer. The plan has **88 trials** (Phase A 9,
Phase B 79). No counted trial has run. One shakedown trial on `s0_open` ran and is recorded below; it
counts for nothing. Later edits to this file are dated amendments placed before the trials they
affect.

History of the draft: written on 2026-10-18 from `DESIGN_ADVICE.md` section 4 with the lead
decisions D-4 (a) to (c); revised the same day after the first Opus review (sections "Escape
parameters and rules", "Held-out status of s3", "Decision rule", "Provenance", "Attempts,
interruption and restart"); amended by the lead decisions E-1 to E-3 (`LEAD_DECISIONS_2.md`): the
extras are accepted and disclosed with the measured ablation, a second held-out pocket `s4_ltrap`
outside the development family is added, and the plan has 88 trials (see "Amendments", amendment 1).

The checklist before the freeze, with the state of each item, is at the end ("Before the freeze").

## Why

Campaign 1 (`experiments/results_pilot_rerun/`) is a clean negative result: 0 of 44 valid
trials reached the goal on a correct `nav2_mppi_controller` build at a real-time factor of
0.96. `results_pilot_rerun/ROOT_CAUSE.md` names four defects of the controller and its
parameter file:

1. `vx_min: 0.1`: the robot cannot stop or reverse.
2. The entrapment detector does not run when MPPI throws `NoValidControl`.
3. The progress index is re-anchored on every replan.
4. The escape selects a sideways opening and does not route out of the pocket.

The branch `semppi-v2/escape-fix` fixes all four (see `docs/` and the commit log). This
campaign asks whether the fixed controller escapes live.

## Hypotheses

- **H1 (mechanism).** On a pocket that is absent from the planner's map, SE-MPPI at the
  fixed commit with Nav2's recoveries disabled (C4) reaches the goal more often than stock
  MPPI with recoveries disabled (C3).
- **H2 (deployable value).** With Nav2's default recovery tree on both, SE-MPPI (C2) reaches
  the goal at least as often as stock MPPI (C1), in less time, with fewer recovery
  invocations. **H2 is descriptive.** "At least as often" cannot be tested at n = 6; the
  campaign reports counts, intervals and the Fisher p of the contrast, and decides nothing
  from them (see "Decision rule").
- **H3 (no harm).** On a trap-free course all four configurations reach the goal and
  SE-MPPI logs no `SE escape ENTER`. Descriptive: counts on `s0_open`.
- **H4 (transfer).** H1 holds on `s4_ltrap`, an L-shaped dead end that lies outside the
  development family (its corridor is deeper than any development pocket, 2.05 m against 1.85 m for `s1_utrap`, and it has a bent
  leg), was specified by the lead before anything ran on it, and was authored blind (see
  "The second held-out pocket"). **H4 is tested on `s4_ltrap` only. The claim of transfer rests on
  s4.** `s3_ctrap` is also run and reported. It was not driven live and not run headless, but it lies
  inside the family's range and has a near mirror twin there (see "Held-out status of s3"). H4
  does not rest on it.

## Fixed inputs

**Commit.** The tag `campaign2-controller` marks the commit that holds the four fixes, the
headless closed-loop test green on the development geometries, this file and
`FREEZE_MANIFEST.json` (the order is in "Before the freeze"). Every trial records the
repository commit, `git describe`, the sha256 of the freeze manifest, the commit of the tag,
the install prefixes of `nav2_se_controller` and `se_mppi_utrap`, the sha256 of the installed
controller libraries, the build stamp, and the output of `git status` for the tracked inputs in
its `meta.json`. No source change after trial 1. A fix restarts the campaign as attempt 2 with
its own raw root, and both attempts are reported ("Attempts, interruption and restart").

**Parameters.** `experiments/sim/nav2_se_campaign2.yaml` is set V of campaign 1
(`nav2_se_loopback.yaml` with `vx_min: -0.35`) plus these differences, which are listed in
the file header and checked by `tests/test_c2_plan.py`:

| Key | Campaign 1 | Campaign 2 |
|---|---|---|
| `FollowPath.vx_min` | 0.1 (set P), -0.35 (set V) | -0.35 |
| `controller_server.failure_tolerance` | -1.0 | 0.0 (Nav2 default) |
| `progress_checker.required_movement_radius` | 0.1 | 0.5 (Nav2 default) |
| `progress_checker.movement_time_allowance` | 120.0 | 10.0 (Nav2 default); 30.0 in C3 and C4 |
| `bt_navigator.default_nav_to_pose_bt_xml` | persistent tree | Nav2's default tree; replanning-only tree in C3 and C4 |
| `FollowPath.EscapeCritic.gap_weight` | 4.0 | 0.0 |
| escape parameters | compiled defaults, unwritten | written out, equal to the compiled defaults (next section) |

The MPPI block is unchanged: 10 Hz, 28 steps, `model_dt` 0.1, batch 1500, inflation
radius 0.45.

**`failure_tolerance` 0.0 and the detector update on a throw.** The fix of defect 2 updates the
entrapment detector when MPPI throws `NoValidControl`, and rethrows. With `failure_tolerance`
0.0, `controller_server` rethrows the exception, `computeControl` catches it and calls
`publishZeroVelocity()`, which calls `reset()` on every controller. The detector state is gone in
the same cycle, and in C3 and C4 the first throw ends the goal. **The detector update on a throw
therefore cannot act in this campaign**; it acts only with `failure_tolerance` -1, as in campaign
1. The headless test does not exercise it either: its pass rule requires zero throws
(`test_closed_loop_utrap.cpp`, `passes()`), and a run with a throw is reported as "C4 would
abort". All 168 runs with the additions (3 sweeps of 56 geometries) had zero throws and zero contacts and reached the goal; 167 of 168 passed (the one non-pass never entered the pocket). A trial in which a throw occurs
is a trial in which the escape did not prevent the all-collide state; it is counted as it ends.
The fix of defect 2 is tested by the unit tests with the detector driven through the throw path
(`ControllerNoValidControl`), not by this campaign.

**Environment.** The micromamba environment `semppi_rerun` (`nav2_mppi_controller` 1.3.13
`_25`). The driver refuses to run unless `check_mppi_rollout` prints OK. The package list is
`results_pilot_rerun/env_correct.txt`, unchanged.

**Refusal.** The driver and `plan.refuse_unsafe` refuse any parameter tree with
`se_enabled` and `vx_min > 0`. The controller also logs a warning at configure time.

**Frozen files.** `FREEZE_MANIFEST.json` holds the sha256 of the parameter file, the
behaviour trees, the worlds and scenarios, the plan, the driver, the provenance module, the
recorder, the host-load module, the analyzer and its tests, the campaign-1 analyzer that it
imports, and the controller sources. The driver refuses a counted trial when
`freeze_manifest.py --check` fails or the manifest is missing. Every trial also records the
sha256 of the parameter file, its tree and its world (decision D-4b).

## Escape parameters and rules

The escape that ships is the one in this section. It is the design of `DESIGN_ADVICE.md`
section 2 plus four additions that the design does not have. The additions are a departure from
decision D-1 ("exactly as advised") and are listed here, with the reason and the place where
each was developed, so that nobody reads this table as the advised design.

**Where the values come from.** The values of the rows marked "advice" were fixed by
`DESIGN_ADVICE.md` section 4.2 before any code existed. The additions were developed on two
things only: the headless closed-loop test on the development family
(`UTRAP_DEV_GEOMETRIES.md`; the additions were worked on until all 56 valid geometries passed,
so the family is a development set, not a test set), and static gap tests at the stall pose of
the live scenario `s1_utrap` (commits `6d0d9ce` and `5f17e19`; `test_opening_search.cpp`, the
cases built with `s1Window`). The use of the `s1_utrap` stall pose is allowed, because `s1_utrap`
is the development scenario, and is stated here because it is a use of a live scenario in the
design of a parameter. The Python mirror was not the source of any addition. Before this revision
the mirror did not have them; it now transcribes them (`prototype/se_mppi_v2.py`, flag
`escape_extras`), and `benchmark2d/tests/test_v2_cxx_parity.py` compares the mirror with the C++
through `parity_shim`. The architecture note `docs/architecture/2026-10_escape-v2.md` section 3
gives the reason for each addition.

| Key | Value | In DESIGN_ADVICE 4.2 | Origin |
|---|---|---|---|
| `se_enabled` | true | no (existing switch) | compiled default |
| `se_enforce_rollout_check` | true | no (existing guard of campaign 1) | compiled default |
| `se_progress_stall_window` | 30 cycles (3 s at 10 Hz) | advice | 4.2 |
| `se_progress_min_advance` | 0.10 m | advice | 4.2 |
| `se_progress_adopt_advance` | 0.5 m | advice | 4.2 |
| `se_escape_path_substitution` | true | the mechanism is advice; the switch is new | 4.2 and defect 4b; the switch is for ablation |
| `se_escape_waypoint_cap` | 2.5 m | advice | 4.2 |
| `se_escape_reselect_radius` | 0.4 m | advice | 4.2 |
| `se_escape_reselect_period` | 15 s | advice | 4.2 |
| `se_escape_hysteresis` | 0.5 rad | advice | 4.2 |
| `se_gap_num_rays` | 36 | advice | 4.2 |
| `se_gap_max_range` | 2.0 m | advice | 4.2 |
| `se_gap_min_clearance` | 0.6 m | the value is advice (section 1, the existing default); **its use on horizon candidates is not** (addition d) | closed-loop test |
| `se_escape_stall_time` | 3 s | **no (addition b)** | closed-loop test, a geometry where MPPI stood before a gap that the binary threshold called free (note section 3, item 2) |
| `se_escape_stall_distance` | 0.15 m | **no (addition b)** | same |
| `se_escape_avoid_radius` | 1.0 m | **no (addition a)** | static gap tests at the `s1_utrap` stall pose (`6d0d9ce`, `5f17e19`) and the closed-loop test |
| `se_escape_avoid_hop_radius` | 0.6 m | **no (addition a)** | closed-loop test, a rotated pocket where the selection swung between two places (note section 3, item 1) |
| `se_escape_avoid_hops` | 3 | **no (addition a)** | same |
| `se_goal_reached_tolerance` | 0.5 m (compiled default, not written in the YAML) | no (existing) | compiled default |
| `EscapeCritic.gap_weight` | 0 | advice | 4.2 |
| APF term | unchanged (`use_apf` true, `repulsion_weight` 2.0, `apf_influence_dist` 0.6) | advice (kept) | 4.2 |

Rules that are compiled constants and not parameters, all additions: at most 6 failed waypoints
are remembered (addition a); a candidate waypoint closer than 0.2 m is not taken (addition c); an
avoided place counts only when it is ahead of the ray (less than 90 degrees off), the robot is
farther than its radius from it, and it lies not more than the radius beyond the end of the ray
(addition a).

**The rules, in the order the controller applies them** (`opening_search.cpp`,
`safe_escape_controller.cpp`, `manageEscapePath`):

1. *Detection.* Progress is the arc length along a per-task anchor path (the first plan);
   a replan is a candidate whose signed advance along the chain of candidates reaches
   `se_progress_adopt_advance` before it becomes the anchor (`path_progress.cpp`). Entrapment is
   declared after `se_progress_stall_window` cycles without a progress event, and is suppressed
   within `se_goal_reached_tolerance` of the goal.
2. *One cycle of lag.* The escape is decided at the start of a cycle from the detector state at
   the end of the previous cycle. The stall pose is the pose at the cycle that declared
   entrapment.
3. *Opening selection.* Among the rays that reach the sensing horizon (`se_gap_max_range`, or the
   costmap edge) with a clearance of at least `se_gap_min_clearance` (addition d), and that do not
   lead to an avoided place (addition a), take the one closest to the goal bearing. If there is
   none, take the longest ray that does not lead to an avoided place, if it clears
   `se_gap_min_clearance`. If excluding rays leaves nothing, repeat without the avoid list.
   Hysteresis keeps the previous bearing unless the best ray is closer to the goal bearing by more
   than `se_escape_hysteresis`.
4. *Avoided places* (addition a): the stall pose (`se_escape_avoid_radius`), the failed waypoints
   of the episode and the last `se_escape_avoid_hops` hop origins (`se_escape_avoid_hop_radius`).
5. *Waypoint.* On the selected bearing at `min(clearance - robot_radius - 0.05, se_escape_waypoint_cap)`;
   taken only if that distance is at least 0.2 m (addition c). The path handed to MPPI runs from the
   robot to the waypoint.
6. *Re-selection*, checked in this order: the robot is within `se_escape_reselect_radius` of the
   waypoint (inclusive); `se_escape_reselect_period` has passed; the robot has not moved
   `se_escape_stall_distance` in `se_escape_stall_time` (addition b); the straight line to the
   waypoint is blocked. A waypoint that is not reached is added to the failed list (addition a).

**Departure from D-1, and the decision on it (E-1).** D-1 says "exactly as advised". Additions (a) to (d)
are a departure. The lead decided on 2026-10-18 (`LEAD_DECISIONS_2.md`, E-1) to accept them as the
shipped algorithm and to disclose them here, in the method section of the paper and in its limitations,
as additions found necessary on the development family. The mirror carries both arms (`escape_extras`
true and false; runner variants `v2` and `v2_advised`), and the 1,200-trial mirror re-run reports both.
Whatever the paper says about SE-MPPI is a statement about the algorithm with the additions.

**Measured ablation.** The additions are switched off with `se_escape_avoid_radius` 0.0,
`se_escape_avoid_hop_radius` 0.0 and `se_escape_stall_time` 1e6. The exact string is
`SE_UTRAP_PARAMS="se_escape_avoid_radius=0.0,se_escape_avoid_hop_radius=0.0,se_escape_stall_time=1e6"`.
The values must carry a decimal point: the rig reads `0` as an integer, and the parameter then fails with an invalid type. That leaves the rules
of `DESIGN_ADVICE.md`, except the 0.2 m waypoint floor, the 0.6 m horizon clearance and the failed-waypoint
list, which have no parameter. The test is the whole development family (56 valid geometries, one run each,
`SE_UTRAP_SWEEP=all`) with the pass rule of `UTRAP_DEV_GEOMETRIES.md` (enters the pocket, leaves within 60 s,
reaches the goal within 150 s, no contact, no `NoValidControl` throw). MPPI's noise cannot be seeded from
the test, so one sweep is one process and the sweeps are independent. Counts per sweep:

| Sweep (process) | Additions | Passed of 56 | Non-pass |
|---|---|---|---|
| implementer | on | 56 of 56 | none |
| reviewer | on | 55 of 56 | the run never entered the pocket (a shallow pocket the straight line passes) |
| reviewer | off | 16 of 56 | not itemised |
| implementer, earlier | off | 19 of 56 | not itemised |
| confirmation 2, `on1` | on | 55 of 56 | `w26_d11_r15_a6`: never entered the pocket; it reached the goal |
| confirmation 2, `on2` | on | 56 of 56 | none |
| confirmation 2, `on3` | on | 56 of 56 | none |
| confirmation 2, `off1` | off | 16 of 56 | 31 goal not reached within 150 s, 9 did not leave within 60 s |
| confirmation 2, `off2` | off | 18 of 56 | 29 goal not reached within 150 s, 9 did not leave within 60 s |
| confirmation 2, `off3b` | off | 14 of 56 | 33 goal not reached within 150 s, 9 did not leave within 60 s |

Confirmation 2 ran at commit `b7e012a` (the controller sources are those of `507173a`; only the guard
test differs), on the build in the job workspace, on a shared host. Raw logs:
`/mnt/d/wsl/se-mppi/v2_confirm2/sweep_*.log`. With the additions: 167 of 168 runs pass (all 168 reach
the goal; zero contacts, zero throws; mean 5.1 to 5.3 hops per run). Without them: 16, 18 and 14 of 56
(mean 16, 48 of 168), with zero contacts and zero throws, and a mean of 16.7 to 17.4 hops per run. A stall occurred twice, in either arm. The first was in the third sweep without the additions (`off3`): it stopped
producing output after 3 rows and used 100 percent of one core for 12 minutes; it was killed and the sweep was run again
as `off3b`. The geometry it stopped at (`w13_d11_r15_a6`) passed 3 of 3 alone. The reviewer then reproduced the stall in
the first sweep with the additions (`on1`) on `w22_d11_r0_a10`: the run finished normally (escape exit, controller
deactivated and cleaned up), then after the `local_costmap` destroy message the main thread spun at 100 percent for 13
minutes without printing the row, ignored SIGINT and needed SIGKILL. `sweep_off3.log` ends with the same signature. Both stalls
are in the teardown of the test rig (`Rig::~Rig`, `utrap_rig.hpp`, at `costmap_ros_.reset()` or `node_.reset()`; that
costmap was configured but never activated or cleaned up), after the run's row is complete. They do not depend on the
arm and do not occur in a control cycle; the counts are not affected and no controller change is needed. The log is kept
(`sweep_off3.log`). On the primary geometry
`w22_d13_r30_a10` the review found 0 of 3 runs without the additions (23 to 26 hops, closest approach
2.37 to 2.39 m) and the twin of s3, `w22_d15_r-25_a6`, did not pass without them in the sweeps (run alone, with the
additions off, it passed 3 of 3). The per-sweep counts are the reported quantity. A geometry's outcome
without the additions depends on its position in the sweep process (reviewer subset of 4 geometries,
3 processes per arm: 8 of 12 passed run alone against 4 of 12 in the sweeps; with the additions on,
12 of 12 in both).

The paper states the additions (a) to (d), this ablation and its non-seedable noise as a limit of the
evidence: the additions were worked on until the development family passed, so the 56 of 56 is a
development result, not a test result.

## Scenarios

| Id | World | Role |
|---|---|---|
| `s1_utrap` | `se_mppi_utrap/worlds/testbed.sdf` | development scenario (campaign 1) |
| `s3_ctrap` | `campaign2/worlds/s3_ctrap.sdf` | held out in the sense of "Held-out status of s3" (twin in the family); reported, not decision-bearing |
| `s4_ltrap` | `campaign2/worlds/s4_ltrap.sdf` | held out outside the family's range ("The second held-out pocket"); configurations C3 and C4 only; carries H4 |
| `s0_open` | `campaign2/worlds/s0_open.sdf` | trap-free control |
| `s1_utrap_evidence` | `testbed.sdf`, start inside the pocket | log artefact; 2 trials each for C2 and C4; not counted |

All scenarios share the start (-3, 0), the goal (3, 0) and the empty-box localization map.
The walls are in the Gazebo world and not in the map.

**s3 specification** (written before the controller tuning, realized by a separate agent
that did not see the tuning discussion, D-4a). In the pocket frame (open toward -x): back
wall 2.4 m long at x = 1.3; top side wall 1.6 m; bottom side wall 1.1 m (asymmetric);
thickness 0.3 m, height 1.0 m as in the template; the whole pocket rotated +25 degrees about
(0.5, 0). Wall positions are centre lines. The side walls sit at y = +/-1.2 on the ends of
the back wall, as the template puts its side walls at y = +/-1.0 for a 2.0 m back wall. The
generator is `make_worlds.py`; `verify_geometry.py` reads the geometry back from the SDF
file; `s3_ctrap_geometry.png` is the plot for review. No Gazebo run and no headless test
has used s3. A guard test (`test_heldout_guard.cpp`, `tests/test_c2_plan.py`) fails if the
world name appears in the repository outside `experiments/campaign2/`. The guard checks the
world name only. It cannot detect a development geometry that resembles s3.

### Held-out status of s3

s3 has a near mirror twin among the development geometries. The inner size of s3 is 2.1 m wide
and 1.45 m deep, its bottom side wall is 1.1 m of 1.6 m (0.69), and it is rotated +25 degrees.
The family members `w22_d15_r-25_a6` and `w26_d15_r-25_a6` (valid geometries; the shipped
algorithm reached the goal on them with 3 hops in 28.2 s and with 7 hops in 46.3 s) are 2.2 m or
2.6 m wide, 1.5 m deep, rotated -25 degrees, with the top wall at 0.6. The start, the goal and
the straight replans all lie on y = 0, so s3 reflected about y = 0 is `w22_d15_r-25_a6` to within
0.1 m in width, 0.05 m in depth and 0.09 in the wall ratio. The wall thickness differs (0.15 m in
the family, 0.30 m in s3) and the pivots differ. The controller faces nearly the same problem up
to the reflection. The additions of the previous section were worked on until this twin passed;
without them it failed in all three ablation sweeps, but run alone it passed 3 of 3, so its failure
without the additions is not robust.

What "held out" therefore means here, and all it means: s3 was not driven live, was not run in
the headless test, and its world file was written by an agent that did not see the tuning. It
does **not** mean that s3 lies outside the span of the development family, and the campaign does
not claim a geometry the design never saw. "s3 carries the claim" in the verbatim outcome wording
below is to be read with this paragraph: the claim on s3 is a claim for a pocket of the
development family's range that was not driven or tested during development.

The stronger design is a pocket outside the family's range, specified by the lead and authored
blind by a separate agent. Lead decision E-2 (2026-10-18) adds it as `s4_ltrap`, and keeps s3 with this
disclosure. H4 is now stated on s4 (see "The second held-out pocket" and amendment 1).

### The second held-out pocket

**Specification** (lead, written before anyone ran anything on it, `LEAD_DECISIONS_2.md` E-2).
An L-shaped dead end. In a pocket frame with the mouth facing -x: a corridor 1.6 m wide and 2.2 m
deep along +x (wall centre lines at y = +/-0.8 from x = 0 to x = 2.2), closed at x = 2.2, except that the
top wall turns: from (2.2, 0.8) a second leg 1.2 m long continues along +y to (2.2, 2.0), with its
inner wall from (1.4, 0.8) to (1.4, 2.0) and the cap from (1.4, 2.0) to (2.2, 2.0). The leg is closed
(a dead end, not a passage). The structure is translated so that the mouth centre is at (-0.6, 0) and
rotated by -15 degrees about the mouth centre. Start (-3, 0), goal (3, 0), testbed and empty map as in
s1. Wall thickness 0.3 m and height 1.0 m as in the template.

**Authorship.** An agent that saw only this specification wrote the world. It did not see the
controller, the family rule, the tuning or the campaign-1 analysis. The world is branch
`semppi-v2/campaign2-s4`, commit `8561495`, merged on this branch. The generator is
`make_s4_ltrap.py`, `verify_s4_ltrap.py` reads the geometry back from the SDF file, and
`s4_ltrap_geometry.png` and `s4_ltrap_verify.txt` are the plot and the output. No Gazebo run and no
headless test has used s4. `test_heldout_guard` and `tests/test_c2_plan.py` fail if the world name appears in
the repository outside `experiments/campaign2/`.

**Checks by the author** (computed, `s4_ltrap_verify.txt`). The straight start-goal line enters the
corridor at the mouth centre (-0.6, 0) and ends on the back wall at pocket y = +0.55, on the side of the leg and 0.10 m below the leg's opening (erratum: the author's printout in `verify_s4_ltrap.py` line 116 has the sign wrong and gave -0.55; its own `to_pocket` gives +0.549), inside the
inner half width of 0.65. The mouth is 1.6 m between wall centre lines and 1.3 m clear. The leg is 0.5 m
clear. A flood fill on the 5 cm grid with the mouth sealed encloses 3.145 m2 from both the corridor and
the leg, and the region reaches neither the goal, nor the start, nor the perimeter. With the mouth open
the goal is reachable only around the structure.

**Checks by the lead** (`tests/test_c2_s4.py`, read from the SDF file and from
`UTRAP_DEV_GEOMETRIES.md`; the family's maximum depth was not told to the author):

| Quantity | s4_ltrap | Development family | Outside the range? |
|---|---|---|---|
| Inner depth, mouth plane to back-wall face | 2.05 m (2.12 m along the start-goal line) | grid 1.1, 1.5, 1.8 m; gating primaries 1.3 and 1.5 m; the development scenario `s1_utrap` 1.85 m (maximum 1.85 m) | **yes**, by 0.20 m (11 percent) over `s1_utrap`, the deepest development pocket |
| Inner width | 1.3 m | 1.3, 1.6, 2.2, 2.6 m (primaries 2.2 and 1.6 m; `s1_utrap` 1.7 m) | no (at the lower end) |
| Rotation | -15 degrees | grid -25, 0, 15, 30 degrees; gating primaries 30 and -15 degrees | no (inside the span; -15 degrees is exactly the rotation of the gating primary `w16_d15_r-15_a6`) |
| Shape | corridor and a closed bent leg, 5 walls | one rectangle, 3 walls | **yes** (no counterpart) |
| Rule V1 of the family (`|d tan(theta)| <= w/2 - 0.3`) | 0.55 > 0.35 | holds for every member | **fails** (margin 0.10 m, not 0.3 m) |

So s4 is outside the family by its depth, by its bent leg and by the margin at which the straight
line enters the corridor. It is inside the family's span in width and rotation. The depth is
measured along the pocket axis, as the family defines `d`.

**Pocket polygons** (analysis only; `analyze.py`, `POCKETS`). s1: x from -1.0 to 0.85,
|y| <= 0.85. s3: the pocket frame rectangle x from -0.3 to 1.15, |y| <= 1.05 (inner faces of
the walls, mouth at the end of the longer side wall), rotated as the walls. s4: the non-convex L in the pocket frame (mouth plane at x 0), with
the vertices (0, -0.65), (2.05, -0.65), (2.05, 1.85), (1.55, 1.85), (1.55, 0.65), (0, 0.65). That is the
corridor x 0 to 2.05 with |y| <= 0.65 plus the leg x 1.55 to 2.05 up to y 1.85, all at the inner faces of
the walls. The frame is rotated by -15 degrees about the mouth centre, and the mouth centre is at
(-0.6, 0) in the world. The inside test is the even-odd rule. A trial leaves
the pocket at the first ground-truth sample outside the polygon after the first sample
inside it. For s1 this is the rule "x < -1.0 or |y| > 0.85" of the design.

## Configurations

| Id | Controller | Behaviour tree | `failure_tolerance` | Progress checker |
|---|---|---|---|---|
| C1 | stock MPPI (`A_stock`) | Nav2 default, `nav_to_pose_default_recovery.xml` | 0.0 | 0.5 m / 10 s |
| C2 | SE-MPPI (`F_se_full`) | as C1 | 0.0 | 0.5 m / 10 s |
| C3 | stock MPPI | replanning-only, `nav_to_pose_replanning_only.xml` | 0.0 | 0.5 m / 30 s |
| C4 | SE-MPPI | as C3 | 0.0 | 0.5 m / 30 s |

`nav_to_pose_default_recovery.xml` is a byte copy of Nav2's
`navigate_to_pose_w_replanning_and_recovery.xml` (checked by a test against the installed
file). The replanning-only tree removes every `RecoveryNode` and the recovery branch: the
first `FollowPath` failure ends the goal with `NOT_REACHED_ABORTED`. The 30 s allowance is
three times the default. It is justified before the trials by the 3 s detection window plus
a 180 degree turn, and it is the only non-default stall parameter. It applies to C3 and C4
equally. The persistent tree and `failure_tolerance -1` of campaign 1 are retired.

## Repetitions, order, host gate

Six repetitions per cell. The two-sided Fisher p of the repository's own `fisher_two_sided`
(`results_pilot_rerun/analyze.py`) for a SE cell against a stock cell that reached 0 times: at
n = 6, 6 of 6 gives 0.0022, 5 of 6 gives 0.0152 and 4 of 6 gives 0.0606; at n = 5, 5 of 5 gives
0.0079 and 4 of 5 gives 0.0476. So at n = 6 one failure still gives p = 0.015, and at n = 5 one
failure gives p = 0.048, which is at the threshold of 0.05. n = 6 is chosen for the margin and for
the time budget. (A test, `tests/test_c2_review_fixes.py`, recomputes these numbers.)

The plan has **88 trials** (`run_campaign2.py --list`; it was 76 before amendment 1, which adds the 12
trials of C3 and C4 on `s4_ltrap`):

- **Phase A (gate, 9 trials).** C4 x s1 x 6 and C3 x s1 x 3 (repetitions 0 to 2). Phase A
  trials count.
- **Phase B (79 trials).** The remaining cells of {C1..C4} x {s1, s3, s0} x 6 (63 trials:
  C3 x s1 repetitions 3 to 5 and all other cells), C3 and C4 x s4 x 6 (12 trials), plus 2 evidence
  trials each for C2 and C4 (4 trials). C1 and C2 do not run on s4.

**Block B, s3 and s4 wait for Phase A.** The driver refuses `--block B`, and refuses every `s3_ctrap` and
`s4_ltrap` trial and every Phase-B trial, until Phase A is complete (6 valid C4 x s1 and 3 valid C3 x s1
trials) and its gate has not fired (`analyze.s3_refusal`). The reason is D-7: if s3 or s4 were driven and
the gate then failed, the second design iteration would no longer have an unseen pocket.

The cell order rotates with the repetition (`plan.py`, `_rotate`: one place per repetition
in Phase A, two places in Phase B). Trial ids are `correct__<scenario>__<config>__rep<n>`.
Raw data go to `/mnt/d/wsl/se-mppi/campaign2/` (`trials/`, `shakedown/`) for attempt 1; only
`results.json.gz`, `RESULTS.md` and the log excerpts in `logs/` are committed.

**Host gate and validity** (campaign 1, amendments 5 and 8, unchanged; the module
`results_pilot_rerun/hostload.py` is imported, not copied): a trial starts only when the
1-minute load average is below 2.0 at two checks 30 s apart and the busy CPU over those
30 s is below 0.5 core. A trial is valid if the foreign CPU is at most 1.0 core on average
and at most 3.0 cores in every 30 s window. An invalid attempt is kept as `.invalidN`,
reported, and repeated. Time: failing trials take about 6.5 min, successful ones 2.5 to 3.5
min; 88 trials with half succeeding and 15% invalid repeats need 8 to 10 hours of quiet host
in two or three sessions.

## Provenance

The freeze manifest hashes repository sources. At run time the controller libraries come from the
colcon install prefix, and the behaviour trees, the testbed world, the launch file and the URDF of
`se_mppi_utrap` come from its installed share directory, which holds copies. Before every counted
trial the driver (`provenance.py`) checks and records:

1. Every file under the installed `worlds`, `urdf`, `launch`, `config` and `behavior_trees` of
   `se_mppi_utrap` is byte-identical to the repository file, and to the manifest hash when the file
   is frozen. Nothing else is installed there. The map files and the s3, s4 and s0 worlds are read
   from the repository by path and are covered by the manifest.
2. The sha256 of the installed `libse_mppi_core.so`, `libescape_critic.so`,
   `libmppi_rollout_check.so` and `libsafe_escape_controller.so`, in `meta.json`.
3. The build stamp `share/nav2_se_controller/build_stamp.txt`, written by CMake at every build:
   `git_describe` must be exactly `campaign2-controller` (the tag, not dirty) and `git_head` the
   commit of the tag.
4. The repository has the tag as an ancestor of HEAD, no frozen file changed since the tag, and no
   uncommitted change to a tracked file other than the generated analysis outputs
   (`RESULTS.md`, `results.json.gz`, `logs/`).

Any difference refuses the trial (exit code 2). The values are in `meta.json` under `provenance`.

## Attempts, interruption and restart

- A trial that stops on Ctrl-C or on an exception is marked `interrupted` and never `complete`.
  On resume the driver keeps its directory as `.abortedN` (reported, never deleted) and runs the
  trial again. The analyzer reports such a directory as `INVALID`.
- Every trial records the sha256 of `FREEZE_MANIFEST.json` and the commit of the tag. A trial
  directory with another hash or commit is foreign: the driver refuses to run, and the analyzer
  stops with `MIXED_ATTEMPTS`. Attempt 1 uses `/mnt/d/wsl/se-mppi/campaign2/`. A restart after a
  fix (D-7 or "A fix restarts the campaign") is attempt 2: run it with `--attempt 2` and analyze
  it with `analyze.py --attempt 2`; its raw root is `/mnt/d/wsl/se-mppi/campaign2/attempt2/`.
  Both attempts are reported.
- Finished trials are skipped on resume, a finished trial whose host-load verdict is invalid is
  kept as `.invalidN` and repeated.

## Success criterion and records

**Primary.** `REACHED`: the ground-truth position is within 0.5 m of the goal at the end of
the drive and the chassis never touched a wall (chassis-to-wall distance >= 0.01 m for every
ground-truth sample). The other outcomes are those of campaign 1: `REACHED_WITH_CONTACT`
(reported, not counted as a success), `NOT_REACHED_TIMEOUT`, `NOT_REACHED_ABORTED`,
`INVALID_HOST_LOAD`, `SETUP_FAIL`, `NO_GOAL_SENT`, `INVALID`. Nav2's own `SUCCEEDED` is
recorded but not used. The outcome logic of `analyze.py` is a re-implementation of the
campaign-1 logic; `tests/test_c2_equivalence.py` checks on 300 random synthetic trials that the
outcome, `reached`, `contact`, the final distance and the minimum wall distance equal those of the
campaign-1 `analyze_trial`.

**Per SE trial in a pocket scenario.** Whether `SE escape ENTER` was logged at or before the
first ground-truth sample outside the pocket polygon (`enter_before_leaving_pocket`). A
reached trial without it is "reached without escape" (`reached_without_escape`).

**Secondary** (`analyze.py`, one record per trial): time to goal in wall and simulated
seconds (first ground-truth sample within 0.5 m); ENTER and EXIT counts and the duration
of each escape episode; waypoint hops (`SE escape waypoint` lines, with bearing, clearance
and reason; **these lines are the controller's choice of heading**); injection BEGIN and END
counts; the critic's probe lines (`escape_critic_probe_lines`: the opening that the critic would
select without the avoid list and without hysteresis, toward the escape waypoint; **not** the
escape heading, labelled so in the log); number of messages on `/FollowPath/se_escape_path` and the
number of publishers the recorder saw; behaviour-tree recovery actions (`Running spin`,
`Running backup`, `Running wait`, local and global costmap clears); `Failed to make progress`
count; optimizer failures and the time of the first; missed control cycles; real-time factor
(whole drive and worst 10 s); load average and foreign CPU.

## Analysis

Counts per cell with Wilson 95% intervals. Two-sided Fisher exact tests for C4 against C3
(s1, s3 and s4 separately), C2 against C1 (s1 and s3 separately), and C4 on s3 and C4 on s4 each
against C4 on s1 (transfer). Seven contrasts in all (`analyze.CONTRASTS`). Mann-Whitney on wall time to goal over reached trials, descriptive (exact for up
to 20,000 arrangements, normal approximation otherwise). No pooling over scenarios. H3 is
the ENTER count on s0 (expected 0) and the reached count on s0 (expected 24 of 24).
The analysis script, `plan.py` and `tests/` are frozen with the tag (D-4b).
`python3 experiments/campaign2/analyze.py --check` fails if `results.json.gz` or
`RESULTS.md` differ from a fresh analysis of the raw data.

### Decision rule

Dated 2026-10-18, written before any counted trial. Extended on 2026-10-18 by amendment 1 (the rule for
s3 was replaced by the rule for s4). It selects the outcome branch below from the counts alone, so that
the branch cannot be chosen after the data are seen. `analyze.decision` computes it.

1. *Significance level.* 0.05, two-sided, per contrast, with no correction for multiple tests.
   Seven contrasts are computed. Each tests its own hypothesis, and only two of them (C4 against C3
   on `s1_utrap`, and on `s4_ltrap`) take part in the decision, each once.
2. *Yes on a pocket.* H1 (on `s1_utrap`) or H4 (on `s4_ltrap`) is "yes" on a pocket if and only if
   the two-sided Fisher p of C4 against C3 on that pocket is below 0.05 **and** C4 reached more
   often than C3 on it. Otherwise it is "no" on that pocket.
3. *Branches.* Demonstrated = s1 yes and s4 yes. "s1 yes, s4 no" = s1 yes and s4 no. Not
   demonstrated = s1 no, whatever s4 shows. (A "yes" on s4 with a "no" on s1 does not rescue the
   claim: s1 is the Phase-A gate and the development scenario, and the mechanism claim H1 is made
   there first. A "yes" on s3 does not rescue a "no" on s4.) The result is reported in full.
   **The claim of transfer rests on s4.**
4. *s3.* `s3_ctrap` is run and reported with the counts, the Fisher p and the disclosure of its
   twin. It does not change the branch. If s3 is "no" beside "yes" on s1 and s4, both are reported and
   the sentence for the branch says so (`decision.s3_note`).
5. *Descriptive only.* H2 (C2 against C1), H3 (the control), the two transfer contrasts (C4 on s3
   and C4 on s4, each against C4 on s1), all time comparisons and all Wilson intervals decide
   nothing. They are reported with their counts and p values, and the verbatim wording below is
   filled from the counts of the branch that rule 3 selects.
6. *Phase-A gate.* Unchanged: C4 on `s1_utrap` at 1 of 6 or fewer stops the campaign (D-7).

With six trials per cell, "yes" needs C4 at 5 of 6 or 6 of 6 when C3 is at 0 of 6 (p = 0.0152 and
0.0022), and C4 at 6 of 6 when C3 is at 1 of 6 (p = 0.0152); 4 of 6 against 0 of 6 is p = 0.0606 and
is "no". These p values come from the repository's own function.

## Stopping rules

`analyze.evaluate_stops` runs before every trial and after every invalid repeat. The driver
exits with code 3 and deletes nothing when one fires.

- **Phase-A gate.** C4 on s1 reaches 1 of 6 or fewer (after six valid trials). Stop; the work
  returns to the headless test (decision D-7).
- A controller crash (`CONTROLLER_CRASH`: a launched process exits with a code other than
  the teardown codes 0, -2, -15 and -9; the driver stops a trial with SIGINT, SIGTERM, SIGKILL,
  so a container killed by the driver shows -9; a kill by the kernel during a drive shows the
  same code and is not detected by this rule) or a refusal to configure (`SETUP_FAIL`: the
  stack was not active after two launches). **The crash rule also scans the superseded attempts**
  (`.invalidN`, `.abortedN`): a crash in an attempt that was repeated is still a crash.
- **Stack broken.** Stock MPPI (C1 or C3) fails on `s0_open`: the rule fires as soon as a stock
  cell has 2 or more valid failures, because with six trials per cell at most 4 of 6 can then be
  reached (the earlier form, "4 of 6 or fewer after the sixth trial", is certain at that point).
  The six trials of a cell are spread over all six repetitions of Phase B, so waiting for the sixth
  would detect a broken stack only after about 70 trials.
- More than 6 invalid attempts on one cell. The cell is reported incomplete.
- `MIXED_ATTEMPTS`: registered trials with more than one freeze manifest or tag commit.

## Outcome wording

Copied verbatim from `DESIGN_ADVICE.md` section 3, "Claims after campaign 2 (fixed before
the trials)". "Section 4.4" in the first item is the section of the paper. The branch is chosen by
the "Decision rule" above. Read the quote with amendment 1: "the held-out s3" and "s3 carries the claim"
mean `s4_ltrap` (outside the family), and `s3_ctrap` is reported beside it with "Held-out status of s3".
The count "k' of 6" in the first item is the count of C4 on s4. The stock count "0 of 6" is given per pocket: C3 on s1 and C3 on s4. The counts "m of 6" and "m' of 6" are C1 and C2 on s1 (C1 and C2 do not run on s4), and s3 is reported beside them.

> - *Demonstrated*: "SE-MPPI without recoveries reached the goal in k of 6 on s1
>   and k' of 6 on the held-out s3; stock without recoveries in 0 of 6; with
>   Nav2's default recovery tree, stock m of 6 and SE-MPPI m' of 6", Fisher exact
>   p per contrast, counts and Wilson intervals only, "on one simulated host". s1
>   is the development scenario; s3 carries the claim. Section 4.4 and the
>   abstract's "no live run has yet reached its goal" are replaced; the pilot's
>   host attribution is withdrawn (campaign 1: RTF 0.96).
> - *s1 yes, s3 no*: report both; the claim shrinks to "escapes the development
>   geometry live" and the paper says the mechanism is geometry-sensitive.
> - *Not demonstrated*: no working-escape claim. Hold the submission, or publish a
>   system report whose abstract says the plugin does not reach the goal live and
>   whose contribution is the mirror-to-controller gap and the four defects. I
>   advise holding unless a deadline forces the second.
>
> In every case `3_method.tex` eq. (detect) and the gap equation, Algorithm 1
> (substitution before the base call; exception handling), `app:mirror`,
> `NUMBERS.md` and `check_paper_numbers.py` change with the code. "Coordination is
> null" stands; campaign 2 does not test it.

Whatever the paper says about the algorithm, it names the additions (a) to (d) of "Escape
parameters and rules" with their measured ablation, the twin of s3 of "Held-out status of s3", and
that s4 is the pocket outside the family on which the claim of transfer rests.

**Not demonstrated** (D-7). If C4 on s1 reaches 1 of 6 or fewer at Phase A, the campaign
stops, the work returns to the headless test with the evidence, and Fable advises one more
design iteration and one more Phase A. If that also fails, the paper is held and the owner
is told.

## Amendments

Every amendment is dated and placed before the trials it affects. No counted trial has run.

**Amendment 1, 2026-10-18 (lead decisions E-1 to E-3, `LEAD_DECISIONS_2.md`).**

1. *Extras accepted (E-1).* The four additions (a) to (d) are the shipped algorithm. They are disclosed in
   this file ("Escape parameters and rules"), in the method section of the paper and in its
   limitations, with the measured ablation. The mirror carries both arms (`escape_extras` true and
   false, runner variants `v2` and `v2_advised`), and the 1,200-trial re-run reports both. This
   replaces item 1 of "Before the freeze" and ends the departure from D-1 as an open question.
2. *Second held-out pocket (E-2).* `s4_ltrap` is added. H4 is tested on it. C3 and C4 run on it, 6
   repetitions each (12 trials), so the plan has 88 trials (Phase A 9, Phase B 79). C1 and C2 do not run
   on s4. s3 stays, with its twin disclosure, reported and not decision-bearing. The decision rule is
   extended as above. This replaces item 2 of "Before the freeze".
3. *Frozen files.* `s4_ltrap.sdf` and `s4_ltrap.scenario.yaml` join the manifest. The analyzer gains the
   s4 pocket polygon (a non-convex L, even-odd test) and `analyze.decision`.
4. *Mirror variant (E-3).* `v2_advised` joins `configs.VARIANTS` (see the mirror README).

## Shakedown (not a registered trial)

One trial on `s0_open` with C2, id `correct__s0_open__C2__rep0`, raw data in
`/mnt/d/wsl/se-mppi/campaign2/shakedown/`. Its purpose is to show that the recorder
captures the new topic and that the analyzer reads the real log. See `SHAKEDOWN.md` for the
result. The trial uses the shakedown probe (`probe_escape_path.py`), a second publisher on
the escape-path topic, because the controller never enters escape on `s0_open`. The shakedown
ran on a build of `baeec71` plus an uncommitted probe edit; it is not evidence about a frozen
build, and the provenance checks apply to counted blocks only.

## Before the freeze

1. ~~Lead, decision on the additions.~~ Done: E-1 accepts them (amendment 1).
2. ~~Lead, decision on s3.~~ Done: E-2 keeps s3 with the disclosure and adds s4 (amendment 1).
3. ~~Review `s3_ctrap_geometry.png` and `s4_ltrap_geometry.png`.~~ Done 2026-10-18: both plots were
   viewed. s3 shows the three-wall pocket open to the start with the start-goal line entering the
   mouth chord and meeting the back wall. s4 shows the L-shaped dead end with its mouth facing the
   start and the goal behind the back wall; `s4_ltrap_verify.txt` agrees.
4. ~~Opus confirmation of amendment 1 (E-4).~~ Accepted by the lead for head `7e629d2`. Tests run on
   2026-10-18 before the freeze: `pytest experiments/campaign2/tests` 212 passed;
   `pytest experiments/benchmark2d/tests` 89 passed, 9 skipped (the C++ parity tests are skipped
   because no `parity_shim` build exists on this host); `scripts/check_paper_numbers.py` OK. Not run
   before the freeze: `colcon test` and the parity tests under `SE_REQUIRE_PARITY=1`. They need the
   workspace build of step 5.4 and run on that build before Phase A.
5. Freeze, in this order:
   1. Finish this file and the controller. Run `python3 experiments/campaign2/freeze_manifest.py
      --write`.
   2. Commit this file and `FREEZE_MANIFEST.json` together with every other change, so that one
      commit holds the final pre-registration and the manifest.
   3. Tag that commit: `git tag campaign2-controller`. The tag points to the commit that has the
      final PREREG and the manifest (the tag must not come first).
   4. Rebuild the workspace at the tag, so that the build stamp says `campaign2-controller`.
   5. The first counted trial records `git describe` (`campaign2-controller`) in `meta.json`.
6. Check that the shakedown host report in `SHAKEDOWN.md` matches the host the counted
   trials will run on. Run campaign 2 on an exclusive host (no other CPU work), Phase A first.
