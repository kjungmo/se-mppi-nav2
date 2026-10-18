# SE-MPPI v2 mirror: smoke result (not a benchmark result)

Status: SMOKE ONLY. 30 scenarios per configuration, 180 trials. The full
1,200-trial re-run is a separate task (command at the end). Do not cite these
counts as results; small n, no test, no interval.

Code: branch `semppi-v2/mirror-port` (from `paper/submission-audit`). Python:
micromamba env `ros2` (numpy 2.4.6, osqp 1.1.3), the environment of the
committed `results_2d`. Host: shared, load average 3 to 13 during the runs, so
the timings are exploratory only. The counts were produced twice (once before
and once after the change of the candidate advance from "best so far" to
"current displacement", see the test `test_backing_off_and_returning_earns_no_advance`);
the outcome counts of the table and of both diagnostics were identical.

## What ran

| file | content |
|---|---|
| `trials.csv` | v2 variant, 6 configs x 30 scenarios: utrap seeds 0-9, narrowdyn seeds 0-9, clutter seeds 0-4, dynamic seeds 0-4; `--max-steps 400`, seed = scenario seed |
| `SMOKE_TABLE.md` | outcome counts per family and config, v2 against the committed paper-v1 rows for the same (family, seed, config) |
| `FLAG_ABLATION.md` | `C_escape` with each of progress / opening rule / hopping on or off (utrap and narrowdyn seeds 0-9) |
| `HORIZON_SWEEP.md` | `C_escape` with all v2 switches on and the ray range / window varied |

Commands (from the repository root):

```bash
python3 -m experiments.benchmark2d.runner --variant v2 --families utrap narrowdyn -n 10 \
    --max-steps 400 --workers 4 --out experiments/results_2d_v2_smoke/trials.csv
python3 -m experiments.benchmark2d.runner --variant v2 --families clutter dynamic -n 5 \
    --max-steps 400 --workers 4 --out experiments/results_2d_v2_smoke/trials.csv
python3 -m experiments.benchmark2d.compare_v1_v2 experiments/results_2d_v2_smoke/trials.csv
python3 -m experiments.benchmark2d.v2_flag_ablation -n 10 --workers 4
python3 -m experiments.benchmark2d.v2_horizon_sweep -n 10 --workers 4
```

## Counts (success / collision / timeout)

The escape layer acts in C_escape, E_indep and F_full. A_stock, D_cbf and
Fminus_nogap do not use the gap search; their counts equal v1. The
zero-command mode fired in one step in each of D_cbf, E_indep and F_full on the
dynamic family, and nowhere else.

| config | n | v1 success | v2 success |
|---|---|---|---|
| A_stock | 30 | 7 | 7 |
| C_escape | 30 | 24 | 18 |
| D_cbf | 30 | 6 | 6 |
| E_indep | 30 | 20 | 17 |
| F_full | 30 | 20 | 17 |
| Fminus_nogap | 30 | 7 | 7 |

U-trap, `C_escape`: v1 9/0/1, v2 6/0/4 (10 seeds). Narrow-dynamic, `C_escape`:
v1 8/2/0, v2 4/2/4 (10 seeds). Clutter and dynamic: no change except dynamic
`C_escape`, `E_indep`, `F_full`, where one timeout became a success in each.

## Reading of the flag ablation (U-trap, C_escape, 10 seeds, success)

| progress | opening rule | hop | utrap | narrowdyn |
|---|---|---|---|---|
| goal_distance | closest_viable | off | 7 | 7 |
| anchor | closest_viable | off | 10 | 8 |
| goal_distance | horizon | off | 2 | 2 |
| anchor | horizon | off | 3 | 3 |
| anchor | horizon | on | 6 | 4 |

- The anchor-path progress source does not lower the mirror's score (it raises
  it from 7 to 10 with the v1 opening rule).
- The horizon opening rule with the C++ values (36 rays, 2.0 m range, 1.5 m
  half window) lowers the U-trap score. Cause seen in `utrap` seed 3
  (`C_escape`): after the robot leaves through the mouth, hop 3 is chosen from
  outside the pocket at (0.86, -0.60) with bearing 50 deg. That ray reaches the
  horizon because the back wall lies farther than 2.0 m from there, so the
  waypoint sends the robot back into the pocket (hops 1-3 at steps 108, 142,
  208; the robot is back inside the pocket by step 250). The same effect in the live controller needs
  a pocket deeper than the sensing horizon; s1 is 1.7 m x 1.85 m.
- A longer horizon helps narrow-dynamic (4 to 8 of 10) but not the U-trap
  (6 to 7 of 10); three U-trap seeds still time out at a 6 m horizon. Cause not
  yet traced.
- The stall window also changes between the sets (15 cycles in v1, 30 in v2),
  so row 1 is not the v1 score (9 of 10).

## Full re-run (not run)

```bash
# quiet host, micromamba env ros2 (numpy 2.4.6), from the repository root
python3 -m experiments.benchmark2d.runner --variant v2 \
    --families utrap clutter dynamic narrowdyn -n 50 --workers 6 --max-steps 400 \
    --out experiments/results_2d_v2/trials.csv
python3 -m experiments.benchmark2d.report \
    --trials experiments/results_2d_v2/trials.csv --out-dir experiments/results_2d_v2
```

The second arm is the design as advised (`escape_extras` false), a file of its own. LEAD_DECISIONS_2
E-1 asks for both arms in the 1,200-trial re-run:

```bash
python3 -m experiments.benchmark2d.runner --variant v2_advised \
    --families utrap clutter dynamic narrowdyn -n 50 --workers 6 --max-steps 400 \
    --out experiments/results_2d_v2_advised/trials.csv
python3 -m experiments.benchmark2d.report \
    --trials experiments/results_2d_v2_advised/trials.csv --out-dir experiments/results_2d_v2_advised
```

The runner refuses to write either arm to `experiments/results_2d/trials.csv`, and refuses to put
the two arms in one file (the `variant` column).

Expected runtime: about 15 minutes at 4 workers on the shared host (this
smoke: 180 trials in 140 s at load 3 to 7; U-trap and narrow-dynamic trials 1.1 s, clutter and
dynamic trials 0.5 s each, so 600 x 1.1 + 600 x 0.5 s of wall time). A quiet
host with 6 workers should take 10 to 15 minutes. The trial results are
deterministic: same seed, same world, same noise. The v1 equivalent took
2.4 s per trial in the identity regeneration (about 49 minutes for 1,200
trials at 4 workers). Likely cause: v1 recomputes a 72-ray, 3.0 m cast at every
escape step; v2 casts only when it picks a waypoint. Not profiled.
