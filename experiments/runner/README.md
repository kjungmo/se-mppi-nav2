# SE-MPPI evaluation harness (L11)

Reproducible benchmark runner that fills the SE-MPPI paper's *pending*
quantitative tables (success / collision / time / clearance / compute). Design:
[`docs/architecture/2026-06_evaluation-harness-design.md`](../../docs/architecture/2026-06_evaluation-harness-design.md);
metrics & ablation definitions: [`…_se-mppi-evaluation-protocol.md`](../../docs/architecture/2026-06_se-mppi-evaluation-protocol.md).

## Layout

```
experiments/
  configs/
    ablations.yaml     # A–F + F-variants as param overlays over a base nav2 yaml
    suite.yaml         # tiers, seeds, timeouts, results dir
  runner/
    config.py          # resolve base + overlay -> launch params (protocol §3)
    gridmap.py         # occupancy load + eroded reachability (start/goal guard)
    scenario.py        # Scenario model + loading + validate()
    metrics.py         # per-trial metrics + outcome classification (protocol §4)
    cleanup.py         # clean-restart between trials (handoff §2-A)
    trial.py           # single-trial state machine; Launcher protocol + Fake/Ros
    run_suite.py       # scenario × config × seed orchestrator (resume-able)
    tests/             # offline unit tests (no ROS)
    ros_launcher.py    # live ROS2/Gazebo Launcher (pure helpers + imperative shell)
    hunav.py           # HuNavSim (T3 social) scenario loader  [H-5]
    dynabarn.py        # DynaBARN (T2 dynamic) scenario loader [H-5]
  analysis/
    stats.py           # McNemar / Mann–Whitney U / Cliff's δ / Holm (protocol §6)
    aggregate.py       # raw JSON -> long CSV + per-(tier,config) summary tables
    plots.py           # comparison figures incl. the E-vs-F contrast (protocol §8) [H-6]
    tests/
  barn/ dynabarn/ hunav/   # scenario yamls per tier (+ an example fixture in each)
  results/             # raw trial JSON + summary CSV + figures/ (gitignored)
```

## One command (turnkey)

The whole matrix runs from a single entry point that reads `configs/suite.yaml`
and selects the launcher. The default **fake** launcher is the offline dry-run
(no ROS): it executes the full scenario × config × seed orchestration and writes
real per-trial JSON, so the pipeline is exercised end-to-end in CI.

```bash
# Offline dry-run (no ROS): exercises the full pipeline, writes results JSON.
python3 -m experiments.runner.run_suite --launcher fake

# Live sweep on a ROS2/Gazebo GPU workstation (the real benchmark):
python3 -m experiments.runner.run_suite --launcher ros
```

Useful overrides: `--tiers barn dynabarn hunav`, `--seeds 0 1 2 ...`,
`--results-dir <dir>`, `--no-resume`. Then aggregate + plot:

```bash
python3 -c "from experiments.analysis import plots; \
            plots.plot_from_results('experiments/results')"   # writes results/figures/*.png
```

## What runs offline (here) vs. on a GPU workstation

The **pure core is fully unit-tested offline** — config resolution, metrics,
classification, reachability, statistics, aggregation, plots, the dynamic-tier
loaders, and the trial/suite orchestration (via a `FakeLauncher`). Run it:

```bash
python3 -m pytest experiments/ -q
```

The **live launch** (`trial.RosLauncher`, implemented in `ros_launcher.py`) needs
a working ROS2 + Gazebo with hardware rendering, which the web container lacks
(design §9). Its **pure helpers are unit-tested offline** (argv/params/env
construction, the "active" log matcher, the smoke_drive telemetry parser, the
no-ROS fail-fast); only the imperative shell (`ros2 launch` + Gazebo + the drive)
runs on the workstation. `--launcher ros` selects it; on a no-ROS machine it
fails fast with a clear message rather than hanging. Two layers remain
workstation-only: **(a)** ground-truth dynamic-obstacle topic sampling for
collision/clearance on the DynaBARN/HuNav tiers (kinematic metrics come straight
from odom telemetry), and **(b)** the **ROS1→ROS2 bridge** for BARN/DynaBARN
worlds (the loader flags it via `scenario.meta['requires_ros1_bridge']`).

## Quick demo (synthetic launcher)

```python
from experiments.runner import config, run_suite, scenario
from experiments.runner.trial import DriveResult, FakeLauncher

suite = config.load_ablations('experiments/configs/ablations.yaml')
scens = scenario.discover('experiments', tier='barn')   # includes example_utrap
fake = lambda: FakeLauncher(DriveResult(samples=[...]))  # or a callable per scenario
report = run_suite.run_suite(fake, suite, scens, seeds=range(20),
                             results_dir='experiments/results')
```

Then aggregate:

```python
from experiments.analysis import aggregate
res = aggregate.aggregate('experiments/results',
                          out_csv='experiments/results/summary.csv')
for row in res['summary']:
    print(row['tier'], row['config'], row['success_rate'])
```

## Ablation matrix (protocol §3)

`A_stock`, `B_escape_alwayson`, `C_escape_detect`, `D_cbf_only`,
`E_escape_cbf_indep`, `F_se_full`, `F_minus_gap`, `F_proxy`, `F_static`.
The load-bearing contrast is **E vs F** (independent vs coordinated α) — see
`analysis/stats.mcnemar` for the paired success-rate test.

## Scenario file format

```yaml
name: example_utrap            # optional (defaults to filename)
tier: barn                     # barn | dynabarn | hunav
map: example_utrap.yaml        # path relative to this file
start: [2.5, 3.5]              # [x, y] or [x, y, yaw]
goal:  [2.5, 0.7]
world: my_world.sdf            # optional (gz world for dynamic/social tiers)
agents: [...]                  # optional dynamic-agent specs (DynaBARN/HuNav)
optimal_length: 3.1            # optional; defaults to straight-line for SPL/BARN
```

`scenario.validate()` rejects a goal that is not in the start's eroded free
component, so an impossible task is recorded SETUP_FAIL rather than counted as a
navigation failure (design §7).
