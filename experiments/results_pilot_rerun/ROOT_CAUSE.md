# Why the live U-trap trials do not reach the goal

Evidence-backed analysis of the registered live re-run (`PREREG.md`,
`RESULTS.md`). Written so that the owner can decide what to change. Nothing here
was changed in the controller: all registered trials ran on one commit.

Paths are relative to the repository root. `C` = `src/nav2_se_controller`.
Times are seconds after the goal was sent. Line numbers refer to the commit the
trials ran on. Upstream Nav2 sources are the `jazzy` branch of
ros-navigation/navigation2, fetched on 2026-10-14 (the installed binary carries
the same log strings; its exact source revision was not compared).

## 0. Summary

| Set | What changed | Trials (valid) | Reached | What happens |
|---|---|---|---|---|
| `P` (pilot's parameters, `vx_min: 0.1`) | — | 20 | 0 | robot cannot stop, creeps into the back wall, MPPI's optimizer then fails every cycle and the controller outputs zero for the rest of the run |
| `V` (`vx_min: -0.35`) | reversing allowed | 12 | 0 | robot stops 0.3–0.45 m before the back wall; stock MPPI stands still there (a true local minimum, no optimizer failure); SE-MPPI enters escape 9–25 times per run but never leaves the pocket |
| `O` (first committed MPPI block) | see `PREREG.md` | 12 | see §6 | see §6 |
| affected binary, `P` | the pilot's build | 4 | 0 | in `s1_utrap` reproduces the pilot: creeps at `vx_min` from the start, never enters the pocket |

Four defects, in the order in which they act:

1. **`vx_min: 0.1` (set `P`).** A value tuned in June on runs whose MPPI build
   was not recorded, with symptoms of the defective binary. On a correct
   binary it forbids stopping and reversing, so the robot drives into the wall.
2. **The detector does not run when MPPI throws** (`P`). The optimizer throws
   `NoValidControl` every cycle once all rollouts collide;
   `SafeEscapeController` updates its detector only after the base call
   returns, so it never sees the stall.
3. **The progress index is re-anchored on every replan** (`V`, and the one
   `P` trial with an escape). Nav2's behaviour tree replans at 1 Hz from the
   robot's pose; a replan to the same goal re-anchors the furthest index on the
   new path, so one path cell of motion along the new path (2.5 cm) counts as
   progress and ends the escape. 70 of 70 closed escape episodes in `V` ended at
   `progress_idx` 1 or 2, median 3.1 s after they began.
4. **Even long escape episodes do not route out of the pocket** (`V`): episodes
   of 24 to 83 s occurred in 5 of the 6 F trials (§4); the robot stayed
   within 0.34 m of its stopping point. The cause is not established (§4).

None of these exists in the Python 2D mirror (§3), which is why the mirror's
88–90% does not carry over.

## 1. Failure sequence on the correct build with the pilot's parameters (`P`)

Representative trial `correct__s1_utrap__A_stock__P__rep0` (ground truth `gt`,
controller output `nav`, after smoother and collision monitor `cmd`;
`telemetry.csv`):

```
  9.1 gt=(0.224,-0.006) nav=(0.100, 0.015)
 12.1 gt=(0.514, 0.006) nav=(0.100, 0.139)
 13.1 gt=(0.607, 0.026) nav=(0.100, 0.318)
 13.6 gt=(0.643, 0.040) nav=(0.086,-0.026)
 14.1 gt=(0.665, 0.051) nav=(0.000, 0.000)   <- and so until 300 s
```

`launch.log`: `13.4 [controller_server] Optimizer reset` (twice), then
`13.4 [controller_server] Optimizer fail to compute path`, repeated about every
0.1 s for the rest of the run (2,862 times in this trial; 2,816–2,941 in the 18
valid `P` trials that ran the full 300 s; the two that Nav2 ended early, as a
false success, have 20 and 1,252).

* **Commands before the failure.** From about 8 s the controller commands
  exactly `vx = 0.100`, the lower bound `vx_min` of set `P`. MPPI's sampled
  controls are clipped to `[vx_min, vx_max]`, so with `vx_min > 0` no rollout
  can stop or back off; the best it can do is creep and turn.
* **Why every rollout collides.** The back wall's face is at x = 0.85 m. The
  robot stops with its centre at x = 0.60–0.71 m (ground truth), 0.14–0.25 m
  from the face, inside the inscribed radius of the local costmap (`robot_radius:
  0.22`; `CostCritic consider_footprint: false`, so the critic checks the centre
  cell). The chassis touches the wall (chassis-to-wall distance below 1 cm in 14 of the 20 valid `P`
  trials). From that pose every rollout starts in a lethal cell.
* **Why the optimizer throws.** Upstream `cost_critic.cpp` sets
  `data.fail_flag = all_trajectories_collide` (line 218). `Optimizer::fallback`
  (optimizer.cpp 203–220) resets the optimizer and, after
  `retry_attempt_limit` consecutive failures, throws
  `nav2_core::NoValidControl("Optimizer fail to compute path")`. Two
  `Optimizer reset` lines per failure match one retry.
* **What `controller_server` does.** With `failure_tolerance: -1.0` (set in
  `experiments/sim/nav2_se_loopback.yaml` line 105 "never abort FollowPath on
  transient plugin failures"), `controller_server.cpp` 675–692 logs the message
  as a warning and publishes zero velocity, every cycle, without aborting the
  action. This is the stream of warnings in the log.
* **What the recovery layer does.** The only abort is the progress checker's
  `Failed to make progress`, which the pilot's file delays to
  `movement_time_allowance: 120.0` (line 115; Nav2's default is 10 s). In
  `correct__s1_utrap__A_stock__P__rep0` it fired at 137.6 s and 262.0 s; each
  time the inner `RecoveryNode` of the persistent behaviour tree cleared the
  local costmap and restarted `FollowPath`, which failed again at once. The
  spin, wait and back-up recoveries of the outer round robin were never reached
  within the 300 s watchdog.
* **Host.** Real-time factor 0.96–0.97 in every valid trial, at most two
  `Control loop missed its desired rate` warnings per trial, foreign CPU below
  1 core. The host does not explain any of this.

So with `P` the stock controller and SE-MPPI fail in the same way, and the
failure is fixed before any escape logic could act. Set `P` itself is the
first cause: its `vx_min`, `failure_tolerance` and `movement_time_allowance`
are the June workarounds (`git log` of the parameter file in the pre-squash
history: commits fd70c22 "forbid reverse", 8e540f3 "vx_min=0.1 (zero-lock
fix)", 87cd1df "persistent BT + load-tolerant timeouts").

## 2. The code path, and what the detector would have seen

`C/src/safe_escape_controller.cpp`:

* 506–507: `geometry_msgs::msg::TwistStamped cmd =
  MPPIController::computeVelocityCommands(robot_pose, robot_speed, goal_checker);`
  — not inside a `try`; the only `try` in the function (535–550) guards a TF
  lookup.
* 562–571: the nearest path index, the furthest index and
  `detector_.update(furthest_progress_)` run after that call.
* 585–586: the flag is published to the critic's registry.

An exception from line 506 therefore leaves the function before line 571: the
stall counter does not move, entrapment is never declared, the critic never
injects escape costs, and the coordinator never raises the gain.

Unit test (scratch, `test_root_cause.cpp`, built against this commit; not part
of the branch): `RootCause.WhenTheBaseControllerThrowsTheDetectorNeverAdvances`
fills the costmap with lethal cells, steps the controller 60 times at a fixed
pose with a stall window of 5, and checks that every cycle throws and that the
controller is not entrapped; the control test on free ground declares
entrapment. Results in §2.1 below.

**Offline replay** (`replay_detector.py`, committed): the recorded ground-truth
positions fed through a transcription of the detector, assuming it ran every
cycle. With the live replanning (a new straight path from the robot every
1 s), the replayed detector declares entrapment at 16.8 s in
`correct__s1_utrap__A_stock__P__rep0` and `..._F_se_full__P__rep0`, about 3 s
after the robot stopped, i.e. had the detector run, the escape would have been
switched on. For `correct__s1_utrap__F_se_full__P__rep3`, the one `P` trial in
which the escape was logged live (ENTER 16.7 s, EXIT 17.9 s), the replay gives
ON 17.0 s, OFF 18.5 s: the reconstruction matches the log.

### 2.1 Scratch unit test results

See the end of this file (filled in after the build).

## 3. Why the 2D Python mirror does not show this

The mirror (`experiments/prototype/se_mppi_proto.py`,
`experiments/benchmark2d/rollout.py`) differs from the C++ path exactly where
the live trials fail:

| | Mirror | C++ controller in Nav2 |
|---|---|---|
| All rollouts in collision | adds `1e4` per colliding sample to the cost (`se_mppi_proto.py` 276–280) and still returns the softmax average; there is no failure state and nothing throws | `CostCritic` sets `fail_flag`; the optimizer throws `NoValidControl`; `controller_server` outputs zero |
| Order of detection and planning | `detector.update` runs first, every step, then `mppi.step` (`rollout.py` 74–75, 95) | detector runs after the base optimizer and is skipped when it throws |
| Progress signal | best distance to the goal so far; progress only if it improves by 1 mm (`se_mppi_proto.py` 74–88). A detour away from or around the trap never counts as progress, so entrapment stays on for the whole detour | furthest index on a path that the behaviour tree replaces every second with one that starts at the robot; one path cell of motion along the new path counts as progress |
| Gap use | the selected gap bearing becomes a temporary goal 2.5 m away that replaces the real goal (`rollout.py` 84–90); the path terms vanish | a bearing cost of weight 4 is added to Nav2's critics, which still include path-alignment (weight 14), path-following (5) and goal (5) terms pulling towards the goal through the wall |
| Lower speed bound | `v ∈ [-0.35, 0.5]` always | `vx_min` from the parameter file (0.1 in set `P`) |
| Sensing | true obstacle geometry, no range limit, no inflation | 1.5 m lidar, 5 Hz costmap, inscribed inflation |
| Replanning | none | NavFn every 1 s, through the U (absent from its map) |

The mirror's headline result therefore rests on a detector that cannot be
cleared by sideways motion and on an escape that replaces the goal. The C++
controller has neither property. This is the gap between the paper's
quantitative result and the deployed controller, and it is a design gap, not a
simulator artefact.

## 4. The escape episodes that did happen

* `P`: one of 10 F trials (`correct__s1_utrap__F_se_full__P__rep3`). ENTER at
  16.7 s at odom (3.51, 0.50) with `progress_idx=0 stall=30` while the robot was
  turning away from the wall; injection BEGIN 16.8 s; the optimizer failed from
  17.0 s; EXIT at 17.9 s with `progress_idx=1` after a replan; from 19 s the
  optimizer failed every cycle and the robot stood still.
* `V`: 79 ENTER events in 6 F trials (9 to 25 per trial); 70 closed episodes,
  median 3.1 s, all closed by an EXIT at `progress_idx` 1 or 2 (defect 3).
  Seven episodes lasted 24–83 s, and in those the robot still stayed in the
  pocket (largest ground-truth x reached 0.29–0.34 m; it never turned back to the
  mouth at x = −1). Why the long episodes do not route out is not established
  from the logs. Candidate contributors, none tested: the path critics outweigh
  the gap bearing cost; the gap search casts rays only 2 m and treats unknown
  cells as blocked, so the mouth behind the robot may not be offered as a gap;
  the selected bearing is the open bearing closest to the goal bearing, which in
  a pocket can be a sideways bearing that leads nowhere.

## 5. Candidate fixes (not implemented)

| Fix | What it changes | Risk |
|---|---|---|
| A. Catch `NoValidControl` in `SafeEscapeController::computeVelocityCommands`, still update the detector, count a failed cycle as no progress, then re-throw (or return a stop) | defect 2; failure no longer hides a stall | a robot blocked by a moving obstacle for 3 s would also enter escape; must not swallow the exception silently (Nav2's failure tolerance and BT rely on it) |
| B. Treat persistent optimizer failure as entrapment evidence and produce the escape command from the escape layer itself (e.g. back off along the free bearing) when the base optimizer has no valid control | defect 2 and the stuck-at-wall state | a second controller path that bypasses MPPI's critics; needs its own collision check; a large change |
| C. Measure progress against the goal (best distance, as the mirror does) or keep a fixed reference path per task instead of re-anchoring on replans | defect 3 | a route that must first move away from the goal is indistinguishable from entrapment (that is what the detector is for); distance to goal is meaningless on winding routes, so a path-arc-length measure on a fixed path is safer |
| D. Use the escape bearing as a temporary goal or weaken the path critics while entrapped (what the mirror does) | defect 4, possibly | fights Nav2's design (path critics are what keep MPPI on the plan); needs tuning |
| E. Parameters: `vx_min` ≤ 0, Nav2's default `movement_time_allowance` and `failure_tolerance` | defect 1, and lets Nav2's own recoveries act | none for correctness; the June values were workarounds |
| F. Rely on Nav2: controller failure → behaviour-tree recovery (spin, back-up) | uses the stack's own stall handling | that is exactly the baseline the paper says its in-controller detector improves on; if the controller works only through the BT, the contribution changes |

**What experiment would show a fix works.** A second, separately
pre-registered campaign on the same host and gate: the same two scenarios, the
same configurations, the fixed controller at one commit, set `V` (or `E`) as the
primary parameter set, 5 repetitions per cell (20 trials, about 2.5 h of quiet
host), plus a stock-MPPI-with-Nav2-recoveries baseline (another 10 trials,
about 1.2 h). Success criterion fixed in advance: ground-truth arrival within
0.5 m, no contact, escape transition logged before the robot leaves the pocket.

## 6. Set `O` (first committed MPPI block)

Filled in after the `O` part finished (below).
