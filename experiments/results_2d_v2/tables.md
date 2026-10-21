<!-- AUTO-GENERATED tables (experiments/benchmark2d/report.py) -->

### Clutter (random static field)

**Clutter (random static field)** — N = 50 scenarios per config (paired; identical scenarios and MPPI noise across configs).

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |
| C · escape only | 72% [58%, 83%] | 0% | 28% | 19.97 ± 2.67 | 6.57 ± 0.48 | 0.35 ± 0.02 |
| D · CBF only | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |
| E · escape+CBF (indep.) | 72% [58%, 83%] | 0% | 28% | 19.98 ± 2.67 | 6.57 ± 0.48 | 0.35 ± 0.02 |
| F · SE-MPPI (coord.) | 72% [58%, 83%] | 0% | 28% | 19.98 ± 2.67 | 6.57 ± 0.48 | 0.35 ± 0.02 |
| F⁻ · F, gap off | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — Clutter (random static field) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/10 | 0.00195 (0.0312) \* | +0.28 (0.899) | +0.28 (0.899) | -0.18 (1) |
| C_escape→F | 0/0 | 1 (1) | +0.00 (1) | -0.00 (1) | +0.00 (1) |
| D_cbf→F | 0/10 | 0.00195 (0.0312) \* | +0.28 (0.899) | +0.28 (0.899) | -0.18 (1) |
| E→F (**key**) | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | +0.00 (1) |

*(McNemar b = baseline-success & F-fail, c = baseline-fail & F-success; Cliff's δ is F relative to the baseline, sign as listed; \* = Holm-adjusted p < 0.05.)*


CBF slack usage — Clutter (random static field) (per-trial `slack_max`; a trial counts as "slack > 0" if the QP ever relaxed a barrier row):

| Config | Trials w/ slack > 0 | Mean slack_max | Max slack_max |
|---|---|---|---|
| A · stock MPPI | 0/50 (0%) | 0.0000 | 0.0000 |
| C · escape only | 0/50 (0%) | 0.0000 | 0.0000 |
| D · CBF only | 0/50 (0%) | 0.0000 | 0.0000 |
| E · escape+CBF (indep.) | 0/50 (0%) | 0.0000 | 0.0000 |
| F · SE-MPPI (coord.) | 0/50 (0%) | 0.0000 | 0.0000 |
| F⁻ · F, gap off | 0/50 (0%) | 0.0000 | 0.0000 |

*(Configs without a CBF (A, C) are trivially zero; on families without movers the CBF sees no obstacles and is likewise zero.)*

### Dynamic (crossing movers)

**Dynamic (crossing movers)** — N = 50 scenarios per config (paired; identical scenarios and MPPI noise across configs).

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 80% [67%, 89%] | 20% | 0% | 22.48 ± 1.44 | 7.99 ± 0.55 | 0.12 ± 0.03 |
| C · escape only | 80% [67%, 89%] | 20% | 0% | 22.54 ± 1.47 | 7.96 ± 0.52 | 0.12 ± 0.03 |
| D · CBF only | 82% [69%, 90%] | 16% | 2% | 22.62 ± 1.53 | 8.02 ± 0.65 | 0.13 ± 0.03 |
| E · escape+CBF (indep.) | 84% [71%, 92%] | 16% | 0% | 22.82 ± 1.56 | 8.07 ± 0.62 | 0.13 ± 0.03 |
| F · SE-MPPI (coord.) | 84% [71%, 92%] | 16% | 0% | 22.85 ± 1.59 | 8.07 ± 0.62 | 0.13 ± 0.03 |
| F⁻ · F, gap off | 84% [71%, 92%] | 16% | 0% | 23.03 ± 1.69 | 8.14 ± 0.68 | 0.13 ± 0.03 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — Dynamic (crossing movers) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/2 | 0.5 (1) | +0.01 (1) | -0.02 (1) | +0.03 (1) |
| C_escape→F | 0/2 | 0.5 (1) | +0.01 (1) | -0.02 (1) | +0.03 (1) |
| D_cbf→F | 0/1 | 1 (1) | +0.02 (1) | +0.03 (1) | -0.01 (1) |
| E→F (**key**) | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | +0.00 (1) |

*(McNemar b = baseline-success & F-fail, c = baseline-fail & F-success; Cliff's δ is F relative to the baseline, sign as listed; \* = Holm-adjusted p < 0.05.)*


CBF slack usage — Dynamic (crossing movers) (per-trial `slack_max`; a trial counts as "slack > 0" if the QP ever relaxed a barrier row):

| Config | Trials w/ slack > 0 | Mean slack_max | Max slack_max |
|---|---|---|---|
| A · stock MPPI | 0/50 (0%) | 0.0000 | 0.0000 |
| C · escape only | 0/50 (0%) | 0.0000 | 0.0000 |
| D · CBF only | 47/50 (94%) | 0.0016 | 0.0144 |
| E · escape+CBF (indep.) | 48/50 (96%) | 0.0016 | 0.0144 |
| F · SE-MPPI (coord.) | 48/50 (96%) | 0.0017 | 0.0144 |
| F⁻ · F, gap off | 47/50 (94%) | 0.0015 | 0.0144 |

*(Configs without a CBF (A, C) are trivially zero; on families without movers the CBF sees no obstacles and is likewise zero.)*

### Narrow-dynamic (U-trap + movers on the escape route)

**Narrow-dynamic (U-trap + movers on the escape route)** — N = 50 scenarios per config (paired; identical scenarios and MPPI noise across configs).

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 0% [0%, 7%] | 18% | 82% | — | — | 0.22 ± 0.04 |
| C · escape only | 72% [58%, 83%] | 18% | 10% | 32.68 ± 1.40 | 8.43 ± 0.47 | 0.22 ± 0.04 |
| D · CBF only | 0% [0%, 7%] | 32% | 68% | — | — | 0.19 ± 0.04 |
| E · escape+CBF (indep.) | 60% [46%, 72%] | 32% | 8% | 32.24 ± 1.51 | 8.19 ± 0.46 | 0.19 ± 0.04 |
| F · SE-MPPI (coord.) | 60% [46%, 72%] | 32% | 8% | 32.24 ± 1.51 | 8.19 ± 0.46 | 0.19 ± 0.04 |
| F⁻ · F, gap off | 0% [0%, 7%] | 32% | 68% | — | — | 0.19 ± 0.04 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — Narrow-dynamic (U-trap + movers on the escape route) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/30 | 1.19e-07 (1.91e-06) \* | +0.00 (1) | +0.00 (1) | -0.08 (1) |
| C_escape→F | 6/0 | 0.0312 (0.438) | -0.06 (1) | -0.09 (1) | -0.04 (1) |
| D_cbf→F | 0/30 | 1.19e-07 (1.91e-06) \* | +0.00 (1) | +0.00 (1) | -0.04 (1) |
| E→F (**key**) | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | +0.00 (1) |

*(McNemar b = baseline-success & F-fail, c = baseline-fail & F-success; Cliff's δ is F relative to the baseline, sign as listed; \* = Holm-adjusted p < 0.05.)*


CBF slack usage — Narrow-dynamic (U-trap + movers on the escape route) (per-trial `slack_max`; a trial counts as "slack > 0" if the QP ever relaxed a barrier row):

| Config | Trials w/ slack > 0 | Mean slack_max | Max slack_max |
|---|---|---|---|
| A · stock MPPI | 0/50 (0%) | 0.0000 | 0.0000 |
| C · escape only | 0/50 (0%) | 0.0000 | 0.0000 |
| D · CBF only | 26/50 (52%) | 0.0187 | 0.4687 |
| E · escape+CBF (indep.) | 35/50 (70%) | 0.0187 | 0.4687 |
| F · SE-MPPI (coord.) | 35/50 (70%) | 0.0248 | 0.4687 |
| F⁻ · F, gap off | 26/50 (52%) | 0.0248 | 0.4687 |

*(Configs without a CBF (A, C) are trivially zero; on families without movers the CBF sees no obstacles and is likewise zero.)*

### U-trap (static local minima)

**U-trap (static local minima)** — N = 50 scenarios per config (paired; identical scenarios and MPPI noise across configs).

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |
| C · escape only | 80% [67%, 89%] | 0% | 20% | 31.54 ± 1.35 | 8.06 ± 0.35 | 0.32 ± 0.00 |
| D · CBF only | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |
| E · escape+CBF (indep.) | 80% [67%, 89%] | 0% | 20% | 31.54 ± 1.35 | 8.06 ± 0.35 | 0.32 ± 0.00 |
| F · SE-MPPI (coord.) | 80% [67%, 89%] | 0% | 20% | 31.54 ± 1.35 | 8.06 ± 0.35 | 0.32 ± 0.00 |
| F⁻ · F, gap off | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — U-trap (static local minima) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/40 | 6.98e-10 (1.12e-08) \* | +0.00 (1) | +0.00 (1) | -0.36 (0.0234) \* |
| C_escape→F | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | -0.00 (1) |
| D_cbf→F | 0/40 | 6.98e-10 (1.12e-08) \* | +0.00 (1) | +0.00 (1) | -0.37 (0.0234) \* |
| E→F (**key**) | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | +0.00 (1) |

*(McNemar b = baseline-success & F-fail, c = baseline-fail & F-success; Cliff's δ is F relative to the baseline, sign as listed; \* = Holm-adjusted p < 0.05.)*


CBF slack usage — U-trap (static local minima) (per-trial `slack_max`; a trial counts as "slack > 0" if the QP ever relaxed a barrier row):

| Config | Trials w/ slack > 0 | Mean slack_max | Max slack_max |
|---|---|---|---|
| A · stock MPPI | 0/50 (0%) | 0.0000 | 0.0000 |
| C · escape only | 0/50 (0%) | 0.0000 | 0.0000 |
| D · CBF only | 0/50 (0%) | 0.0000 | 0.0000 |
| E · escape+CBF (indep.) | 0/50 (0%) | 0.0000 | 0.0000 |
| F · SE-MPPI (coord.) | 0/50 (0%) | 0.0000 | 0.0000 |
| F⁻ · F, gap off | 0/50 (0%) | 0.0000 | 0.0000 |

*(Configs without a CBF (A, C) are trivially zero; on families without movers the CBF sees no obstacles and is likewise zero.)*