<!-- AUTO-GENERATED tables (experiments/benchmark2d/report.py) -->

### Clutter (random static field)

**Clutter (random static field)** — N = 50 scenarios per config (paired; identical scenarios and MPPI noise across configs).

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |
| C · escape only | 62% [48%, 74%] | 0% | 38% | 17.75 ± 2.19 | 6.14 ± 0.32 | 0.35 ± 0.02 |
| D · CBF only | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |
| E · escape+CBF (indep.) | 62% [48%, 74%] | 0% | 38% | 17.76 ± 2.20 | 6.14 ± 0.32 | 0.35 ± 0.02 |
| F · SE-MPPI (coord.) | 62% [48%, 74%] | 0% | 38% | 17.76 ± 2.20 | 6.14 ± 0.32 | 0.35 ± 0.02 |
| F⁻ · F, gap off | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — Clutter (random static field) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/5 | 0.0625 (1) | +0.16 (1) | +0.16 (1) | -0.04 (1) |
| C_escape→F | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | +0.00 (1) |
| D_cbf→F | 0/5 | 0.0625 (1) | +0.16 (1) | +0.16 (1) | -0.04 (1) |
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
| C · escape only | 80% [67%, 89%] | 20% | 0% | 22.48 ± 1.44 | 7.92 ± 0.49 | 0.13 ± 0.03 |
| D · CBF only | 82% [69%, 90%] | 16% | 2% | 22.62 ± 1.53 | 8.02 ± 0.65 | 0.13 ± 0.03 |
| E · escape+CBF (indep.) | 84% [71%, 92%] | 16% | 0% | 22.67 ± 1.50 | 8.06 ± 0.62 | 0.13 ± 0.03 |
| F · SE-MPPI (coord.) | 84% [71%, 92%] | 16% | 0% | 22.60 ± 1.47 | 8.05 ± 0.62 | 0.13 ± 0.03 |
| F⁻ · F, gap off | 84% [71%, 92%] | 16% | 0% | 23.03 ± 1.69 | 8.14 ± 0.68 | 0.13 ± 0.03 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — Dynamic (crossing movers) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/2 | 0.5 (1) | -0.01 (1) | -0.03 (1) | +0.03 (1) |
| C_escape→F | 0/2 | 0.5 (1) | -0.00 (1) | -0.03 (1) | +0.03 (1) |
| D_cbf→F | 0/1 | 1 (1) | -0.00 (1) | +0.03 (1) | -0.01 (1) |
| E→F (**key**) | 0/0 | 1 (1) | -0.00 (1) | -0.00 (1) | +0.00 (1) |

*(McNemar b = baseline-success & F-fail, c = baseline-fail & F-success; Cliff's δ is F relative to the baseline, sign as listed; \* = Holm-adjusted p < 0.05.)*


CBF slack usage — Dynamic (crossing movers) (per-trial `slack_max`; a trial counts as "slack > 0" if the QP ever relaxed a barrier row):

| Config | Trials w/ slack > 0 | Mean slack_max | Max slack_max |
|---|---|---|---|
| A · stock MPPI | 0/50 (0%) | 0.0000 | 0.0000 |
| C · escape only | 0/50 (0%) | 0.0000 | 0.0000 |
| D · CBF only | 47/50 (94%) | 0.0016 | 0.0144 |
| E · escape+CBF (indep.) | 48/50 (96%) | 0.0015 | 0.0144 |
| F · SE-MPPI (coord.) | 48/50 (96%) | 0.0016 | 0.0144 |
| F⁻ · F, gap off | 47/50 (94%) | 0.0015 | 0.0144 |

*(Configs without a CBF (A, C) are trivially zero; on families without movers the CBF sees no obstacles and is likewise zero.)*

### Narrow-dynamic (U-trap + movers on the escape route)

**Narrow-dynamic (U-trap + movers on the escape route)** — N = 50 scenarios per config (paired; identical scenarios and MPPI noise across configs).

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 0% [0%, 7%] | 18% | 82% | — | — | 0.22 ± 0.04 |
| C · escape only | 56% [42%, 69%] | 18% | 26% | 31.41 ± 1.70 | 8.27 ± 0.56 | 0.22 ± 0.04 |
| D · CBF only | 0% [0%, 7%] | 32% | 68% | — | — | 0.19 ± 0.04 |
| E · escape+CBF (indep.) | 44% [31%, 58%] | 32% | 24% | 30.51 ± 1.83 | 7.74 ± 0.41 | 0.19 ± 0.04 |
| F · SE-MPPI (coord.) | 44% [31%, 58%] | 32% | 24% | 30.50 ± 1.83 | 7.74 ± 0.41 | 0.19 ± 0.04 |
| F⁻ · F, gap off | 0% [0%, 7%] | 32% | 68% | — | — | 0.19 ± 0.04 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — Narrow-dynamic (U-trap + movers on the escape route) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/22 | 4.77e-07 (7.63e-06) \* | +0.00 (1) | +0.00 (1) | -0.07 (1) |
| C_escape→F | 6/0 | 0.0312 (0.438) | -0.11 (1) | -0.17 (1) | -0.04 (1) |
| D_cbf→F | 0/22 | 4.77e-07 (7.63e-06) \* | +0.00 (1) | +0.00 (1) | -0.03 (1) |
| E→F (**key**) | 0/0 | 1 (1) | -0.00 (1) | -0.00 (1) | +0.00 (1) |

*(McNemar b = baseline-success & F-fail, c = baseline-fail & F-success; Cliff's δ is F relative to the baseline, sign as listed; \* = Holm-adjusted p < 0.05.)*


CBF slack usage — Narrow-dynamic (U-trap + movers on the escape route) (per-trial `slack_max`; a trial counts as "slack > 0" if the QP ever relaxed a barrier row):

| Config | Trials w/ slack > 0 | Mean slack_max | Max slack_max |
|---|---|---|---|
| A · stock MPPI | 0/50 (0%) | 0.0000 | 0.0000 |
| C · escape only | 0/50 (0%) | 0.0000 | 0.0000 |
| D · CBF only | 26/50 (52%) | 0.0187 | 0.4687 |
| E · escape+CBF (indep.) | 36/50 (72%) | 0.0187 | 0.4687 |
| F · SE-MPPI (coord.) | 35/50 (70%) | 0.0248 | 0.4687 |
| F⁻ · F, gap off | 26/50 (52%) | 0.0248 | 0.4687 |

*(Configs without a CBF (A, C) are trivially zero; on families without movers the CBF sees no obstacles and is likewise zero.)*

### U-trap (static local minima)

**U-trap (static local minima)** — N = 50 scenarios per config (paired; identical scenarios and MPPI noise across configs).

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |
| C · escape only | 58% [44%, 71%] | 0% | 42% | 30.93 ± 1.75 | 7.70 ± 0.32 | 0.32 ± 0.00 |
| D · CBF only | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |
| E · escape+CBF (indep.) | 58% [44%, 71%] | 0% | 42% | 30.93 ± 1.75 | 7.70 ± 0.32 | 0.32 ± 0.00 |
| F · SE-MPPI (coord.) | 58% [44%, 71%] | 0% | 42% | 30.93 ± 1.75 | 7.70 ± 0.32 | 0.32 ± 0.00 |
| F⁻ · F, gap off | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |

*(Time-to-goal and path length are over successful trials only; min clearance over all trials.)*


F vs baselines — U-trap (static local minima) (Holm-corrected within this family; family size 16):

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/29 | 2e-07 (3.2e-06) \* | +0.00 (1) | +0.00 (1) | -0.40 (0.00912) \* |
| C_escape→F | 0/0 | 1 (1) | +0.00 (1) | -0.00 (1) | -0.00 (1) |
| D_cbf→F | 0/29 | 2e-07 (3.2e-06) \* | +0.00 (1) | +0.00 (1) | -0.40 (0.00912) \* |
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