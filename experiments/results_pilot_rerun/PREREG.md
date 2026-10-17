# Pre-registration: live U-trap re-run on a verified nav2_mppi_controller build

Written and committed on 2026-10-12, before any trial of this re-run was started.
The commit that adds this file contains no trial output.

## Why

The live pilot in `experiments/results_pilot/` (2026-07-03) did not reach its
goal (STUCK at 254.5 s, TIMEOUT at 299.8 s). The paper attributes this to the
host (WSL2, real-time factor about 0.3 to 0.6, transform smear). It is now known
that the pilot ran on `ros-jazzy-nav2-mppi-controller 1.3.11 np2py312h2ed9cc7_16`,
whose batch rollouts negate cos/sin for part of the heading range
(RoboStack/ros-jazzy issue 290), and its log shows the robot creeping at
`vx_min`. The pilot cannot separate the two candidate causes. This re-run can.

## Question

On the pilot's host, with the pilot's scenarios and parameter file, does the
controller reach the goal when `nav2_mppi_controller` rolls trajectories out
correctly? And does the affected build, at the same commit and on the same day,
reproduce the pilot's behaviour?

## Fixed design

**Host.** The pilot's host (WSL2 on the same machine). Gazebo runs headless with
software rendering (`LIBGL_ALWAYS_SOFTWARE=1`, `MESA_GL_VERSION_OVERRIDE=3.3`),
as the harness already sets for this host. Other jobs may run on the host; the
1-minute load average is recorded for every sample and no trial is excluded for
load.

**Environments.**

| Label | micromamba env | nav2_mppi_controller | `check_mppi_rollout` must print |
|---|---|---|---|
| `correct` | `semppi_rerun`, new, created by `ROS2_ENV_NAME=semppi_rerun bash scripts/setup_ros2_env.sh` | 1.3.13 `np2py312h405a742_25` | OK |
| `affected` | `ros2`, the pilot's environment, not modified | 1.3.11 `np2py312h2ed9cc7_16` | FAILED |

The driver refuses to run when the label and the check disagree. The full
package list of each environment is committed as `env_<label>.txt` before its
first trial. The code is this branch's commit at the time of the run (recorded
per trial in `meta.json`); `src/` is identical to `main` at 55fb78c.

**Scenarios.** `experiments/barn/s1_utrap.scenario.yaml` (start outside the
pocket, 6 m to the goal through a physical U that the planner's map does not
contain) and `experiments/barn/s1_utrap_evidence.scenario.yaml` (start inside
the pocket). Both are byte-identical to the pilot's.

**Configurations.** `F_se_full` (the pilot's) and `A_stock` (the suite's stock
MPPI baseline: same plugin with `se_enabled: false` and no `EscapeCritic`), from
`experiments/configs/ablations.yaml`, unchanged.

**Parameter sets.** The base file is `experiments/sim/nav2_se_loopback.yaml`. It
differs from the pilot's copy only in a comment (checked against history commit
6572d1d of the pre-squash repository). The pilot's values were tuned by hand in
June 2026 on runs whose `nav2_mppi_controller` build is not recorded, some of
them in answer to symptoms the defect produces. Three sets are run:

| Set | Content | Purpose |
|---|---|---|
| `P` | the file as it is | the pilot's parameters |
| `V` | `P` with `vx_min: -0.35` instead of `0.1` | undoes the one change made in direct answer to the reversing symptom ("forbid reverse"); -0.35 is also the released default |
| `O` | `P` with the MPPI block, controller rate and inflation of the first committed parameter file (history f83039c, 2026-06-08): `controller_frequency 20`, `time_steps 56`, `model_dt 0.05`, `batch_size 2000`, `vx_min -0.35`, `GoalAngleCritic` and `PathAngleCritic` `threshold_to_consider 0.5`, `inflation_radius 0.70` (local and global) | the un-retuned controller parameters; also tests the paper's "compute budget" lesson on a correct build |

AMCL, the behaviour tree, timeouts, transform tolerances, costmap layers and the
`EscapeCritic` block are those of `P` in every set. In the `affected`
environment the only extra change is `se_enforce_rollout_check: false`, without
which the controller refuses to configure there.

**Repetitions.** A live trial has no seed: the harness's `seed` is only a
label, Gazebo and the Nav2 executors are not deterministic in wall time. The
pilot ran each scenario once ("seed 0"). Repetitions here are indexed
`rep0, rep1, ...`.

| Block | Environment | Sets and repetitions | Trials |
|---|---|---|---|
| main | `correct` | `P` x 5, `V` x 3, `O` x 3; each repetition = 2 scenarios x 2 configurations | 44 |
| affected | `affected` | `P` x 1; 2 scenarios x 2 configurations | 4 |

Why these numbers: one trial takes up to about 7.5 minutes of wall time (launch,
300 s watchdog, teardown), so 48 trials fit in about 6 hours. Five repetitions
per cell on the pilot's parameters are enough to show whether reaching the goal
is typical or rare on this host (5 of 5 against 0 of 5 has a two-sided Fisher
exact p of 0.008; any weaker split is not distinguishable and will be reported
as such). They are not enough for a rate with a useful interval, and none will
be claimed. Three repetitions for `V` and `O` only show whether the result
depends on the hand tuning. The affected block is a contrast on the same day and
commit, one trial per cell as the pilot.

**Order.** `python3 experiments/results_pilot_rerun/run_rerun.py --block <b> --env-label <l> --list`
prints the order. Within a repetition the four cells rotate so that no cell
always runs first. The main block runs first, then the affected block.

**Timeouts.** As in `experiments/configs/suite.yaml`: 120 s to reach the active
state with one retry, 300 s of wall time for the drive.

## What is recorded per trial (`trials/<trial_id>/`)

`meta.json` (environment, package builds, `check_mppi_rollout` output, commit,
launch command line, timestamps), `params.yaml` (the exact parameter file),
`launch.log`, `drive.log` (the same telemetry lines as the pilot),
`telemetry.csv` at 10 Hz of wall time (wall time, `/clock`, ground-truth pose of
the robot from the simulator, odometry, `/cmd_vel_nav`, `/cmd_vel`, load
average), `diagnostics.jsonl` (the controller's `/diagnostics`).

## Definitions (fixed now; computed by `analyze.py` from the raw files only)

- **Goal sent** `t0`: the wall time stamp of the `Sending goal` line in `drive.log`.
- **Drive end** `t1`: the wall time stamp of the last telemetry line in `drive.log`.
- **Reached the goal**: the ground-truth position at `t1` is within 0.5 m of the
  goal (the harness's `goal_tol`). Nav2's own result (`SMOKE_RESULT`) is reported
  next to it. The harness's `reached_goal` field is not used: it compares
  odometry-frame positions with the map-frame goal, which differ by the spawn
  offset in `s1_utrap`.
- **Contact**: the smallest distance, over the drive, between the chassis
  rectangle (0.28 m x 0.30 m at the ground-truth pose) and any wall box of
  `testbed.sdf` is below 0.01 m. The smallest distance and the largest absolute
  pitch and roll are reported. The pilot's "collision-free" came from a field
  that is always false when no obstacle list is given, so the pilot has no
  contact measurement to compare with.
- **Outcome**: `REACHED` (reached, no contact), `REACHED_WITH_CONTACT`,
  `NOT_REACHED_TIMEOUT` (the 300 s watchdog ended the drive),
  `NOT_REACHED_ABORTED` (Nav2 ended the drive earlier without reaching),
  `SETUP_FAIL` (not active after two attempts), `INVALID` (raw files missing or
  no ground truth in them).
- **Time to goal**: `t1 - t0` in wall seconds and the `/clock` difference over
  the same interval in simulated seconds, for `REACHED*` trials.
- **Real-time factor**: simulated seconds / wall seconds over `[t0, t1]`; also
  the smallest value over 10 s windows.
- **Missed control cycles**: number of `Control loop missed its desired rate`
  lines in `launch.log`.
- **First 5 s**: mean, minimum and maximum of the controller's `vx`
  (`nav=vx:` in `drive.log`) over the first 5 s after `t0`, and the share of
  those samples within 0.01 m/s of `vx_min`. For the affected build this is the
  signature (creeping at `vx_min` when `vx_min` >= 0).
- **Escape mechanism**: number of `SE escape ENTER` lines and of
  `escape-cost injection BEGIN` lines in `launch.log`, with the position of the
  first `ENTER`.
- **Guard state**: the `mppi_rollout_check` value on `/diagnostics`.

## Reporting rules

1. Every trial that is started is reported, with its outcome, including
   `SETUP_FAIL`, `INVALID` and trials interrupted by the driver or the host
   (kept as `<trial_id>.abortedN`). No trial is replaced or re-run to improve a
   count. If the driver or host stops a block, the block resumes at the first
   unfinished trial and the interrupted attempt stays in the table.
2. Shakedown runs made to get the driver, the bridge and the recorder working
   are not part of the registered trials. They are written to
   `trials_shakedown/`, committed, and listed separately with their outcomes.
   Once the first registered trial has started, the driver, the recorder, the
   parameter sets and these definitions do not change; if one must change, the
   change and the trials before and after it are stated.
3. Numbers quoted in the paper are produced by `analyze.py` and checked by
   `scripts/check_paper_numbers.py`.

## Which outcome leads to which wording

- **Correct build reaches the goal in `s1_utrap` with `F_se_full` and `P`, and
  the log shows `SE escape ENTER` before the robot leaves the pocket, in at
  least one trial.** The paper reports a live demonstration with the exact
  counts for every cell (n, environment, real-time factor), as descriptive
  small-n evidence. "Escape demonstrated end to end" is claimed for the trials
  that show it and for nothing wider. A difference between `F_se_full` and
  `A_stock` is stated only as counts with the Fisher exact p; no rate, no
  "significant" unless p < 0.05, and no claim beyond this scene and host.
- **Correct build reaches the goal but without an escape transition**, or
  `A_stock` reaches it as often: the paper says the live stack reaches the goal
  on a correct build and that the escape layer's live benefit is not shown.
- **Correct build does not reach the goal either.** The paper says so with the
  counts. The attribution is limited to what the trials show: missed control
  cycles and the real-time factor if they are present, "cause not established"
  otherwise.
- **Attribution of the July pilot.** If the `affected` trials reproduce the
  pilot's signature (first 5 s at `vx_min`, goal not reached) while `correct`
  trials with `P` reach the goal, the paper states that the pilot's outcome is
  reproduced by the defective binary and not by the host on a correct one, and
  withdraws the host attribution. If the `affected` trials reach the goal, or
  the `correct` ones fail in the same way, the paper states that the cause of
  the pilot's outcome remains not established. In every case the defect is
  disclosed and the measured real-time factor is reported as a fact.
- **"Compute budget" lesson.** Kept only if the `O` trials show missed control
  cycles and a worse outcome than `P`; otherwise it is reworded to what was
  measured or removed.
- **If Gazebo cannot run at all in the new environment**, what was tried is
  documented and the paper takes the "no re-run" wording: the pilot ran on a
  defective binary, the host attribution is withdrawn, and every sentence that
  leaned on the pilot is narrowed.

## Amendments made after the shakedown, before the first registered trial

Committed on 2026-10-12 together with the three shakedown runs
(`trials_shakedown/shake1` to `shake3`), before the first registered trial was
started.

1. **Build fix** (`src/nav2_se_controller/CMakeLists.txt`, one line and a
   comment: `add_link_options("LINKER:--as-needed")`). In the new environment
   the controller did not configure: `Could not create class of type
   mppi::critics::ConstraintCritic` (`shake1/launch.log.gz`). Today's
   `cxx-compiler 2.0.0` no longer brings the conda toolchain activation, so
   `LDFLAGS` is empty, the plugin libraries keep a `DT_NEEDED` entry for
   `libmppi_critics.so`, and the stock critics register with the wrong class
   loader. With the option the package asks for `--as-needed` itself. No C++
   source changed. So `src/` is `main` at 55fb78c plus this build-system line,
   in both environments; the sentence "`src/` is identical to `main`" above is
   corrected by this item.
2. **Active check.** The harness's `log_is_active` also accepts the
   localization manager's line and reported the failed stack of `shake1` as
   active. The driver now requires the navigation lifecycle manager's
   `Managed nodes are active` line and stops waiting on `Aborting bringup`.
3. **Load gate.** Other jobs on the host pushed the 1-minute load average above
   20 during `shake1`. Before each trial the driver now waits until the load
   average is below 6.0 (12 logical CPUs), for at most 20 minutes, then starts
   anyway; the load at the start, the wait and whether the gate passed are in
   `meta.json`, and the load is in every telemetry row. This schedules trials;
   it excludes none.
4. **Ground truth.** The bridge drops entity names, so the recorder takes the
   first pose of `/world/testbed/dynamic_pose/info` (the model; its links
   follow). `analyze.py` marks a trial `INVALID` unless its first ground-truth
   sample lies within 0.05 m of the scenario's start.

**Shakedown runs** (all `correct`, `s1_utrap`, `F_se_full`, `P`; not registered
trials; kept and listed with the results):

| Run | What happened | Used for |
|---|---|---|
| `shake1` | the controller did not configure (item 1); the driver wrongly saw the stack as active; no ground truth (topic remap ignored) | items 1 and 2 |
| `shake2` | ran the full 300 s; no ground truth recorded (names dropped, item 4); by AMCL the robot ended inside the pocket | item 4 |
| `shake3` | Nav2 reported `SUCCEEDED` after 26 s while the ground truth has the robot at x = 0.71 m, against the back wall of the U and 2.3 m from the goal: the wheels slipped, odometry kept counting and AMCL followed it | shows why "reached" is defined on ground truth; nothing changed |

These three outcomes were seen before the registered trials. No definition,
parameter set, count or wording rule above was changed because of them.

## Amendment 5: host load is the confound, so it is gated (replaces item 3)

Committed on 2026-10-12 before any trial that counts.

**What happened.** The main block was started under the load gate of item 3
(1-minute load average below 6.0). Two trials ran
(`correct__s1_utrap__F_se_full__P__rep0`, `correct__s1_utrap__A_stock__P__rep0`)
while other jobs of the same work session were compiling and running benchmarks
on the host: the load average during them was 5.1 to 10.8 and 5.7 to 32.9. The
block was then stopped (the third trial was still waiting at the gate and never
launched). The paper blamed the host for the pilot, so a trial on a loaded host
cannot answer the question. **These two trials are exploratory only.** They are
kept in `trials_exploratory_loaded/`, listed in the results with their load, and
used for no statement. Their outcomes were seen (both ended inside the pocket
after 300 s). The registered blocks start again from the first trial under the
rules below; the design, sets, counts, order, definitions and wording rules are
unchanged.

**Admission (before every trial).** The driver waits, without a time limit,
until the 1-minute load average is below 2.0 at two checks 30 s apart. One
trial's own stack (Gazebo, the Nav2 container, bridge, recorder) uses about 4
cores of the 12 while it runs, so the gate also spaces the trials. Recorded per
trial: `/proc/loadavg` at the start, the number of CPUs, and the eight busiest
processes immediately before and after (`meta.json`: `load1_at_start`,
`load5_at_start`, `ncpu`, `top_before`, `top_after`).

**Validity (during every trial).** A sampler writes, every 5 s, the host's busy
CPU time (`/proc/stat`), the CPU time of the trial's own processes (those that
carry the trial's environment tag) and the load average (`host_cpu.csv`).
Foreign CPU is the difference, in cores. A trial is **valid** if, over the
drive, foreign CPU is at most 1.0 core on average and at most 3.0 cores in every
30 s window. The thresholds are set now, without data: one core of twelve
leaves the trial's stack eleven, and a 30 s burst of three cores still leaves
nine against a need of about four. The rule deliberately uses only load that
comes from outside the trial. The real-time factor and the missed-control-cycle
warnings are **not** part of it: they are outcomes (the `O` set exists to see
whether the controller misses its rate), and a rule that discarded trials with
a low real-time factor would hide exactly what is to be measured. They are
reported for every trial, valid or not.

**Invalid trials.** A trial that fails the rule is kept as
`<trial_id>.invalidN`, reported with its measured outcome and load, not counted,
and repeated after the gate, at most twice. If the third attempt is invalid too,
the cell is reported as incomplete. The same gate and rule apply to the
`affected` block, so the affected-versus-correct contrast is on equal load.

**Idle calibration (one run, before the registered trials).** When the host is
quiet, one unregistered run (`trials_calibration/`, `correct`, `s1_utrap`,
`F_se_full`, `P`) is made under the gate to check that an otherwise idle host
passes the validity rule and to record the own-stack CPU use and the real-time
factor of an unloaded trial. If the idle host does not pass (background services
of the machine), the thresholds are raised to the measured idle level plus the
margins above, and that is stated here before the first registered trial. The
calibration run is listed with the results and counts for nothing.

**Reporting.** The paper and the submission notes state the measured load
conditions of the live runs (load average, foreign CPU, real-time factor) as
facts, and keep "single WSL2 host" among the limitations.

## Amendment 6: calibration result and block order

Committed on 2026-10-12 before the first registered trial.

**Calibration.** Two calibration attempts ran (`trials_calibration/`). The first
was invalid by the rule of amendment 5 (foreign CPU 1.94 cores on average, 3.23
in the worst 30 s window; another job was compiling). The second was valid
(0.76 and 1.90) although the host was not fully idle, so a quiet host passes the
rule and the thresholds stay as written. One trial's own stack used 4.1 cores
and the real-time factor was 0.96. Both attempts ended inside the pocket after
300 s with no escape transition logged; this outcome was seen before the
registered trials and changes nothing above.

**Order.** The quiet window on the host is limited, so the blocks run in this
order: the `P` part of the main block (20 trials), the affected block (4), then
the `V` and `O` parts (24). Within each part the order is the one registered.
Counts, sets, definitions and wording rules are unchanged. If the window closes
before the `V` and `O` parts finish, the trials not run are reported as not run.

## Amendment 7: interruption on 2026-10-12, continuation on 2026-10-14

Committed on 2026-10-14 before any further trial.

**What happened.** On 2026-10-12 the `P` part of the main block finished (20
valid trials; one attempt invalid by host load and repeated). The affected
block had three valid trials when the work session that owned the driver ended
at about 18:46 KST and the driver stopped with it. That is an operational
interruption, not a result. The fourth affected cell
(`affected__s1_utrap_evidence__A_stock__P__rep0`) had one attempt invalid by host
load (kept as `.invalid1`) and one attempt cut off before its drive (kept as
`.aborted1` when the driver resumes). The `V` and `O` parts had not started. No
Gazebo or Nav2 process was left behind.

**Continuation.** The remaining registered trials run on 2026-10-14 on the same
host under the same gate and validity rule, in the registered order: the fourth
affected cell (1 trial), then the `V` and `O` parts (24). Nothing completed is
repeated and nothing is replaced.

**Environment changes detected since 2026-10-12.** The WSL2 kernel is now
`6.18.54.1-microsoft-standard-WSL2` (it was `6.18.40.1`); each trial records its
kernel in `meta.json`. Both micromamba environments are unchanged (their package
lists are identical to the committed `env_correct.txt` and `env_affected.txt`),
and the two workspaces are the builds of 2026-10-12. So the `P` part and three
of the four affected trials ran on the older kernel, and the last affected trial
and the `V` and `O` parts run on the newer one; comparisons across that boundary
carry this caveat.

**Descriptive fields added to the analysis.** After the first registered trials
`analyze.py` gained fields that no outcome definition uses: the number of
`Optimizer fail to compute path` warnings and the time of the first one, the
number of `Failed to make progress` errors, and the time from which the robot
stood still (less than 2 cm of ground-truth motion in 5 s). They were added
because the trials showed the robot stopping at the back wall while MPPI's
optimizer failed every cycle. The registered definitions are unchanged.

**Outcomes seen so far** (stated so that the reader knows what was known when
the continuation started): none of the 20 valid `P` trials on the correct build
reached the goal.

## Amendment 8: four attempts invalidated by a known constant job (2026-10-14)

Committed on 2026-10-14 at about 10:05 KST, before any further trial.

**What happened.** The continuation started at 09:25 KST while one
single-threaded job of another work session (`hitrate_probe`, nice 19) was
running on the host. It used 0.8 to 1.0 core all the time, which together with
the host's baseline put foreign CPU at 1.05 to 1.32 cores on average, above the
limit of 1.0. Four attempts ran and all four were invalid by the unchanged rule:
one of `affected__s1_utrap_evidence__A_stock__P__rep0` (kept as `.invalid2`) and
three of `correct__s1_utrap__F_se_full__V__rep0` (kept as `.invalid1` to
`.invalid3`). The driver was stopped at 09:58 KST. No valid trial was added.

**What the invalid attempts showed** (seen, reported in the results, counted for
nothing): the affected attempt ended at the back wall like the valid affected
trial of the same scenario; the three `V` attempts did not reach the goal, with
no optimizer failure and 25 to 27 escape transitions each.

**Rule changes, all made before any further trial.**

1. *Stricter admission.* The load-average gate lets a one-core job through. A
   trial now also starts only if the host's busy CPU over the 30 s before the
   start is below 0.5 core (`busy_cores_before_start` in `meta.json`). The
   validity rule during the trial is unchanged (1.0 core mean, 3.0 cores in any
   30 s window).
2. *Repetition cap.* The cap of two repetitions was meant to stop a trial that
   cannot be made valid. Here the cause was one identified, constant job, so the
   attempts it invalidated are not counted against the cap: a cell is now given
   up only after six load-invalid attempts in total. This is a change of a
   registered rule after seeing data, made so that a scheduling accident does
   not remove a cell; every invalid attempt stays in the record.
3. *Driver fix.* After a restart the driver tried to rename a second invalid
   attempt to a name that already existed, failed, and the affected block ended
   early (the `V` block then started). Invalid attempts now get the next free
   suffix. No trial outcome was affected.

**Process priority.** On 2026-10-14 the trial's own processes run at nice 5 (the
session that starts them is niced). The niceness on 2026-10-12 was not recorded.
With eleven cores free this should not matter, but it is a difference between
the two days that cannot be excluded.

**Order from here.** The fourth affected cell, then the `V` and `O` parts, as
registered.

## Amendment 9: second interruption (2026-10-14), continuation on 2026-10-18

Committed on 2026-10-18 at about 03:15 KST, before any further trial.

**What happened.** On 2026-10-14 the driver stopped with its work session at
about 12:27 KST, an operational interruption, not a result. By then the
affected block was complete (4 of 4 valid), the `V` part was complete (12 of 12
valid) and the `O` part had 7 valid trials. The eighth `O` trial,
`correct__s1_utrap__F_se_full__O__rep1`, was cut off before its record was
closed; it is kept as `.aborted1` and repeated. The four trials of `O` rep2 had
not started. No Gazebo or Nav2 process was left behind.

**Continuation.** The 5 remaining registered trials run on 2026-10-18 on the
same host under the same gate and validity rule (amendments 5 and 8) and in the
registered order. Nothing completed is repeated or replaced.

**Environment check.** Kernel `6.18.54.1-microsoft-standard-WSL2`, as on
2026-10-14. Both micromamba environments are identical to the committed package
lists. The workspaces are the builds of 2026-10-12. Nothing changed.

**Outcomes seen so far.** No valid trial of any set has reached the goal
(`RESULTS.md` at the commit of this amendment).
