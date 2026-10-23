# SE-MPPI v2 mirror: full 1,200-trial re-run (both arms)

Code: commit 742a3df16d2d9f0adb617b9c020dc64962b2dfe1 (branch semppi-v2/escape-fix).
Python: micromamba env `ros2` (numpy 2.4.6, osqp 1.1.3). Workers: 6. `--max-steps 400`, `-n 50`,
families utrap clutter dynamic narrowdyn, 6 configs, 1,200 trials per arm.

| arm | directory | trials |
|---|---|---|
| `v2` (shipped) | `experiments/results_2d_v2/` | `trials.csv`, `tables.md`, `stats.json`, `summary.csv`, `COMPARE_V1.md` |
| `v2_advised` (design as advised) | `experiments/results_2d_v2_advised/` | same files |

Commands: the "Full re-run" commands of `SMOKE.md`, then
`python3 -m experiments.benchmark2d.compare_v1_v2 <arm>/trials.csv > <arm>/COMPARE_V1.md`.

Determinism (identity regeneration): `runner --variant v1` with the same families, `-n 50`,
`--max-steps 400` rewrote 1,200 rows; every row equals a row of the committed
`experiments/results_2d/trials.csv` (sorted `diff` empty; row order differs because the pool returns scenarios in completion order, so `cmp` fails at line 32). The v1 benchmark therefore regenerates exactly in this
environment.
