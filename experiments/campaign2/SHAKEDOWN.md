# Shakedown of campaign 2 (2026-10-18, not counted)

One trial on `s0_open` with C2 (SE-MPPI, Nav2 default tree), trial id
`correct__s0_open__C2__rep0`, phase `shakedown`. It is not a registered trial and enters no
statistic. Raw data: `/mnt/d/wsl/se-mppi/campaign2/shakedown/` (the driver's own log is
`driver_final.log` there). Committed: `results.json.gz`, `RESULTS.md` (section "Shakedown
runs") and `logs/shakedown__*.log.gz` (launch-log excerpts with the sha256 of the full log).

## Question

Does the recorder capture the new topic (`/FollowPath/se_escape_path`) and do the analyzer
and the driver read a real log? The controller never enters escape on `s0_open`, so it
publishes nothing on that topic, and its log has no `SE escape` line. The shakedown therefore
shows three things, and not a fourth:

1. The topic exists with one publisher (the controller's escape-path publisher) while the
   stack runs, and the recorder sees it: `graph` records in `escape_path.jsonl`.
2. The recorder's subscription receives messages on that topic and writes them (first and
   last position, last yaw, frame, count): three messages from a probe
   (`probe_escape_path.py`, frame `shakedown_probe`), a second publisher on the same topic
   name that starts after the subscriptions match. The analyzer counts them apart.
3. The analyzer and the driver run on a real log: outcome REACHED, time to goal, host-load
   verdict, counters, excerpt.

Not shown: a real escape-path message of the controller, and a real `SE escape ENTER`,
waypoint or injection line from a live run. The patterns for those lines are tested against
the format strings in `safe_escape_controller.cpp` and `escape_critic.cpp`, and against real
lines of the headless closed-loop run (`tests/test_c2_analyze.py`). The first live evidence
comes from the first Phase-A trial or from an evidence trial.

## Result of the final attempt

| Item | Value |
|---|---|
| Outcome | `REACHED`, Nav2 `SUCCEEDED`, final distance to the goal 0.24 m, no contact (min wall distance 0.81 m) |
| Time to goal | 11.9 s wall, 11.5 s simulated; real-time factor 0.97 |
| Escape lines | `SE escape ENTER` 0, EXIT 0, waypoint 0, injection 0 (expected on `s0_open`) |
| Controller line in the launch log | `SafeEscapeController[FollowPath] configured: se_enabled=1 robot_radius=0.234 alpha_base=2.00 alpha_escape=6.00` |
| `escape_path.jsonl` | `graph` at 2 s: 1 publisher; 3 `path` records from the probe (n_poses 2, first (0.0,0.0), last (1.0,0.0), last yaw 0.0, wall 1792274324.90 to 1792274326.11); `graph` at 10 s: 1 publisher; probe log: `subscriptions matched: 1` |
| Optimizer failures, `Failed to make progress`, BT recovery actions, missed cycles | 0, 0, 0, 0 |
| Environment | `check_mppi_rollout` OK, `semppi_rerun`, workspace built from the committed head `baeec71` plus the uncommitted probe edit (`probe_escape_path.py`: wait for the subscription match) |

## Host

The shakedown needed a quiet host (load average below 2.0 at two checks, busy CPU below 0.5
core, validity: foreign CPU at most 1.0 core on average). At the start of the work the load
average was 1.9 to 3.1 and one job of another work session (`hitrate_probe`, one core
at 99%) ran for more than 30 minutes, with compiles of other sessions (`cc1plus`, `colcon`)
on and off. The gate waited 1867 s for the first start and later 30 s to 210 s per attempt.

For the final attempt: load average at the start 0.58 (1 min) and 1.17 (5 min), busy CPU
before the start 0.32 core, waited 30 s at the gate. The busiest processes before the start:
`python3` 17.5% (another session's script), `claude` daemons 6.9% and 3.2%, `fusion_node_main`
3.6% (another session's test). During the drive: foreign CPU 0.485 core on average and in the
worst 30 s window (limits 1.0 and 3.0), own CPU 3.8 cores, load average 0.97 to 2.6; valid.

## Attempts (all kept, none counted)

| Directory | What happened |
|---|---|
| `...rep0.invalid1` | Foreign CPU 1.78 cores, invalid by the host rule. The drive also failed (see the next row). |
| `...rep0.envfail1` | Valid by the host rule (0.95 core), but `smoke_drive.py`, the recorder and the probe failed on import: `AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT'`. Cause: the workspace had been built from a shell that had `/opt/ros/noetic` in its environment, so `install/setup.bash` chained that ROS 1 prefix. Fixed by rebuilding the workspace through `act.sh` (the clean environment of campaign 1); `grep -c noetic install/setup.sh` is 0 now. |
| `...rep0.probe_nomatch1` | REACHED in 11.9 s, valid by the host rule (0.69 core; a compile of another session started during the drive), but the probe published before the recorder's subscription was matched, so `escape_path.jsonl` holds no `path` record. Fixed: the probe waits up to 60 s for the match. |
| `...rep0` | The result above. |

A defect of the first valid attempt that the shakedown found in the harness, not in the
controller: a workspace must be built in the clean environment of `act.sh`. The campaign
command in `README.md` assumes that environment.
