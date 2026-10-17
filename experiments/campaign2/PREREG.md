# Pre-registration: campaign 2, the live U-pocket test of the fixed SE-MPPI controller

**Status: DRAFT.** Written on 2026-10-18 from `DESIGN_ADVICE.md` section 4 with the lead
decisions D-4 (a) to (c). It becomes the pre-registration when the lead commits it
together with the tag `campaign2-controller` and `FREEZE_MANIFEST.json`, before the first
trial of Phase A. No counted trial has run. One shakedown trial on `s0_open` ran and is
recorded below; it counts for nothing.

Open items for the lead before the freeze are listed at the end ("Before the freeze").

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
  invocations.
- **H3 (no harm).** On a trap-free course all four configurations reach the goal and
  SE-MPPI logs no `SE escape ENTER`.
- **H4 (transfer).** H1 holds on a geometry that was not used during development.

## Fixed inputs

**Commit.** The tag `campaign2-controller` marks the commit with the four fixes and the
headless closed-loop test green on the development geometries. Every trial records the
repository commit, `git describe`, the install prefixes of `nav2_se_controller` and
`se_mppi_utrap`, and the output of `git status` for the tracked inputs in its `meta.json`.
No source change after trial 1. A fix restarts the campaign, and both attempts are
reported.

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
| escape parameters | compiled defaults, unwritten | written out, equal to the compiled defaults (below) |

The MPPI block is unchanged: 10 Hz, 28 steps, `model_dt` 0.1, batch 1500, inflation
radius 0.45. Escape parameters, set from the mirror and the development-geometry test
before any live trial and not changed after one on `s1_utrap` or `s3_ctrap`:

| Parameter | Value |
|---|---|
| `se_progress_min_advance` | 0.10 m |
| `se_progress_adopt_advance` | 0.5 m |
| `se_progress_stall_window` | 30 cycles (3 s at 10 Hz) |
| `se_gap_num_rays` | 36 |
| `se_gap_max_range` | 2.0 m |
| `se_gap_min_clearance` | 0.6 m |
| opening selection | horizon ray nearest the goal bearing; longest ray if none reaches the horizon |
| `se_escape_hysteresis` | 0.5 rad |
| `se_escape_waypoint_cap` | 2.5 m |
| `se_escape_reselect_radius` | 0.4 m |
| blocked segment | re-select |
| `se_escape_reselect_period` | 15 s |
| `se_escape_stall_time` / `se_escape_stall_distance` | 3 s / 0.15 m |
| `EscapeCritic.gap_weight` | 0 |
| APF term | unchanged (`use_apf` true, `repulsion_weight` 2.0, `apf_influence_dist` 0.6) |

**Environment.** The micromamba environment `semppi_rerun` (`nav2_mppi_controller` 1.3.13
`_25`). The driver refuses to run unless `check_mppi_rollout` prints OK. The package list is
`results_pilot_rerun/env_correct.txt`, unchanged.

**Refusal.** The driver and `plan.refuse_unsafe` refuse any parameter tree with
`se_enabled` and `vx_min > 0`. The controller also logs a warning at configure time.

**Frozen files.** `FREEZE_MANIFEST.json` holds the sha256 of the parameter file, the
behaviour trees, the worlds and scenarios, the plan, the driver, the recorder, the
host-load module, the analyzer and its tests, the campaign-1 analyzer that it imports, and
the controller sources. The driver refuses a counted trial when
`freeze_manifest.py --check` fails or the manifest is missing. Every trial also records the
sha256 of the parameter file, its tree and its world (decision D-4b).

## Scenarios

| Id | World | Role |
|---|---|---|
| `s1_utrap` | `se_mppi_utrap/worlds/testbed.sdf` | development scenario (campaign 1) |
| `s3_ctrap` | `campaign2/worlds/s3_ctrap.sdf` | **held out**; carries the claim |
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
world name appears in the repository outside `experiments/campaign2/`.

**Pocket polygons** (analysis only; `analyze.py`, `POCKETS`). s1: x from -1.0 to 0.85,
|y| <= 0.85. s3: the pocket frame rectangle x from -0.3 to 1.15, |y| <= 1.05 (inner faces of
the walls, mouth at the end of the longer side wall), rotated as the walls. A trial leaves
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

Six repetitions per cell: 6 of 6 against 0 of 6 gives a two-sided Fisher p of 0.002, 5 of 6
against 0 of 6 gives 0.015, 4 of 6 against 0 of 6 gives 0.061. Six is the smallest n at which
one failure still leaves a significant contrast.

The plan has 76 trials (`run_campaign2.py --list`):

- **Phase A (gate, 9 trials).** C4 x s1 x 6 and C3 x s1 x 3 (repetitions 0 to 2). Phase A
  trials count.
- **Phase B (67 trials).** The remaining cells of {C1..C4} x {s1, s3, s0} x 6 (63 trials:
  C3 x s1 repetitions 3 to 5 and all other cells) plus 2 evidence trials each for C2 and C4
  (4 trials).

The cell order rotates with the repetition (`plan.py`, `_rotate`: one place per repetition
in Phase A, two places in Phase B). Trial ids are `correct__<scenario>__<config>__rep<n>`.
Raw data go to `/mnt/d/wsl/se-mppi/campaign2/` (`trials/`, `shakedown/`); only
`results.json.gz`, `RESULTS.md` and the log excerpts in `logs/` are committed.

**Host gate and validity** (campaign 1, amendments 5 and 8, unchanged; the module
`results_pilot_rerun/hostload.py` is imported, not copied): a trial starts only when the
1-minute load average is below 2.0 at two checks 30 s apart and the busy CPU over those
30 s is below 0.5 core. A trial is valid if the foreign CPU is at most 1.0 core on average
and at most 3.0 cores in every 30 s window. An invalid attempt is kept as `.invalidN`,
reported, and repeated. Time: failing trials take about 6.5 min, successful ones 2.5 to 3.5
min; 76 trials with half succeeding and 15% invalid repeats need 7 to 9 hours of quiet host
in two or three sessions. Interruptions resume at the first unfinished trial: finished
trials are skipped, an unfinished directory is kept as `.abortedN` and the trial runs again.

## Success criterion and records

**Primary.** `REACHED`: the ground-truth position is within 0.5 m of the goal at the end of
the drive and the chassis never touched a wall (chassis-to-wall distance >= 0.01 m for every
ground-truth sample). The other outcomes are those of campaign 1: `REACHED_WITH_CONTACT`
(reported, not counted as a success), `NOT_REACHED_TIMEOUT`, `NOT_REACHED_ABORTED`,
`INVALID_HOST_LOAD`, `SETUP_FAIL`, `NO_GOAL_SENT`, `INVALID`. Nav2's own `SUCCEEDED` is
recorded but not used.

**Per SE trial in a pocket scenario.** Whether `SE escape ENTER` was logged at or before the
first ground-truth sample outside the pocket polygon (`enter_before_leaving_pocket`). A
reached trial without it is "reached without escape" (`reached_without_escape`).

**Secondary** (`analyze.py`, one record per trial): time to goal in wall and simulated
seconds (first ground-truth sample within 0.5 m); ENTER and EXIT counts and the duration
of each escape episode; waypoint hops (`SE escape waypoint` lines, with bearing, clearance
and reason); injection BEGIN and END counts; number of messages on
`/FollowPath/se_escape_path` and the number of publishers the recorder saw; behaviour-tree
recovery actions (`Running spin`, `Running backup`, `Running wait`, local and global
costmap clears); `Failed to make progress` count; optimizer failures and the time of the
first; missed control cycles; real-time factor (whole drive and worst 10 s); load average
and foreign CPU.

## Analysis

Counts per cell with Wilson 95% intervals. Two-sided Fisher exact tests for C4 against C3
(s1 and s3 separately), C2 against C1 (s1 and s3 separately) and C4 on s3 against C4 on s1
(transfer). Mann-Whitney on wall time to goal over reached trials, descriptive (exact for up
to 20,000 arrangements, normal approximation otherwise). No pooling over scenarios. H3 is
the ENTER count on s0 (expected 0) and the reached count on s0 (expected 24 of 24).
The analysis script, `plan.py` and `tests/` are frozen with the tag (D-4b).
`python3 experiments/campaign2/analyze.py --check` fails if `results.json.gz` or
`RESULTS.md` differ from a fresh analysis of the raw data.

## Stopping rules

`analyze.evaluate_stops` runs before every trial and after every invalid repeat. The driver
exits with code 3 and deletes nothing when one fires.

- **Phase-A gate.** C4 on s1 reaches 1 of 6 or fewer (after six valid trials). Stop; the work
  returns to the headless test (decision D-7).
- A controller crash or refusal to configure (`SETUP_FAIL`).
- **Stack broken.** Stock MPPI (C1 or C3) reaches 4 of 6 or fewer on `s0_open`.
- More than 6 invalid attempts on one cell. The cell is reported incomplete.

## Outcome wording

Copied verbatim from `DESIGN_ADVICE.md` section 3, "Claims after campaign 2 (fixed before
the trials)". "Section 4.4" in the first item is the section of the paper.

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

**Not demonstrated** (D-7). If C4 on s1 reaches 1 of 6 or fewer at Phase A, the campaign
stops, the work returns to the headless test with the evidence, and Fable advises one more
design iteration and one more Phase A. If that also fails, the paper is held and the owner
is told.

## Amendments

None. Every amendment is dated and placed before the trials it affects.

## Shakedown (not a registered trial)

One trial on `s0_open` with C2, id `correct__s0_open__C2__rep0`, raw data in
`/mnt/d/wsl/se-mppi/campaign2/shakedown/`. Its purpose is to show that the recorder
captures the new topic and that the analyzer reads the real log. See `SHAKEDOWN.md` for the
result. The trial uses the shakedown probe (`probe_escape_path.py`), a second publisher on
the escape-path topic, because the controller never enters escape on `s0_open`.

## Before the freeze

1. Review `s3_ctrap_geometry.png` against the specification above. The worlds step did not
   look at the plot after rendering it.
2. Commit the controller commit and run `git tag campaign2-controller`.
3. Run `python3 experiments/campaign2/freeze_manifest.py --write` and commit the manifest
   with this file.
4. Check that the shakedown host report in `SHAKEDOWN.md` matches the host the counted
   trials will run on.
