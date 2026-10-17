# experiments/campaign2

Assets of campaign 2: the live test of the fixed SE-MPPI controller on a development
pocket (`s1_utrap`), a held-out pocket (`s3_ctrap`) and a trap-free control (`s0_open`).
Design and rules: `PREREG.md` (draft until the tag). Campaign 1 is in
`../results_pilot_rerun/`.

| File | Role |
|---|---|
| `PREREG.md` | hypotheses, fixed inputs, plan, definitions, stopping rules, outcome wording |
| `plan.py` | configurations C1 to C4, the 76-trial plan, parameter overlays, stopping thresholds |
| `run_campaign2.py` | driver (derived from `../results_pilot_rerun/run_rerun.py`) |
| `recorder.py`, `probe_escape_path.py` | 10 Hz recorder with ground truth and the escape-path topic; shakedown probe |
| `analyze.py` | every number from the raw files; writes `results.json.gz`, `RESULTS.md`, `logs/*.log.gz` |
| `freeze_manifest.py` | sha256 manifest of the frozen files (`FREEZE_MANIFEST.json`, written before the tag and committed with `PREREG.md`) |
| `provenance.py` | what is installed is what is frozen: installed trees and libraries, build stamp, git state, attempt identity |
| `make_worlds.py`, `verify_geometry.py`, `worlds/`, `*.scenario.yaml`, `s3_ctrap_geometry.png` | the two new worlds and scenarios |
| `tests/` | `python -m pytest experiments/campaign2/tests` |
| `SHAKEDOWN.md` | the one shakedown trial (not counted) |

The parameter file is `../sim/nav2_se_campaign2.yaml`. The behaviour trees are in
`../se_mppi_utrap/behavior_trees/`. Raw data go to `/mnt/d/wsl/se-mppi/campaign2/`.

## Commands

Run inside the `semppi_rerun` environment with the workspace (`nav2_se_controller`,
`se_mppi_utrap`) sourced, from the repository root.

```bash
python3 experiments/campaign2/run_campaign2.py --list             # the 76-trial plan
python3 experiments/campaign2/run_campaign2.py --block shakedown  # one trial, not counted
python3 experiments/campaign2/freeze_manifest.py --write          # before the tag; commit it with PREREG.md, then tag
python3 experiments/campaign2/run_campaign2.py --block A          # Phase A (gate)
python3 experiments/campaign2/run_campaign2.py --block B          # Phase B (refused until Phase A is complete and its gate has not fired)
python3 experiments/campaign2/run_campaign2.py --block all        # A, gate, B; resumes
python3 experiments/campaign2/run_campaign2.py --block all --attempt 2   # a restart after a fix: its own raw root
python3 experiments/campaign2/analyze.py --pack                   # results.json.gz, RESULTS.md, logs/
python3 experiments/campaign2/analyze.py --check                  # fails if they are stale
python -m pytest experiments/campaign2/tests
```

A run needs a quiet host: the driver waits for the campaign-1 gate (load average below 2.0
at two checks, busy CPU below 0.5 core) before every trial. The driver exits with code 3 when
a stopping rule fires, and with code 2 when it refuses to run.

A counted trial is refused (exit code 2) when the installed behaviour trees, worlds, launch files
or controller build are not the frozen ones, when the build stamp does not name the tag
`campaign2-controller`, when the raw root holds trials of another attempt (another freeze manifest
or tag commit), or when it would drive `s3_ctrap` or run block B before Phase A is complete.
`PREREG.md` ("Provenance", "Attempts, interruption and restart") says why.
