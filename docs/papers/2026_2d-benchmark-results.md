# SE-MPPI standalone 2D quantitative benchmark (paper §VI-C)

> **Status:** measured. This document holds the randomized-scenario benchmark
> that fills §VI-C of `2026_se-mppi-paper-draft.md`. All numbers are produced by
> `experiments/benchmark2d/` from the measured per-trial CSV
> (`experiments/results_2d/trials.csv`); none are transcribed by hand.
> Regenerate with:
>
> ```bash
> export MAMBA_ROOT_PREFIX=$HOME/micromamba
> micromamba run -n ros2 python3 -m experiments.benchmark2d.runner \
>     --families utrap clutter dynamic narrowdyn -n <N> --workers 6 --max-steps 400
> micromamba run -n ros2 python3 -m experiments.benchmark2d.report      # tables + stats
> micromamba run -n ros2 python3 -c "from experiments.benchmark2d import figures, aggregate; \
>     r=aggregate.aggregate('experiments/results_2d/trials.csv','experiments/results_2d'); \
>     figures.plot_all(r['summary'], r['stats'], 'experiments/results_2d/figures')"
> ```

## 1. What this benchmark is (and is not)

This is a **standalone 2D benchmark that mirrors the controller math one-to-one**.
It reuses the exact `se_mppi_proto` primitives validated in §VI-A (the same
MPPI sampler, APF, gap raycast, look-ahead DCBF-QP via OSQP, α coordination with
TTC override, and monotone-progress entrapment detector) and drives them across
*randomized* scenarios so that success/collision/clearance can be reported with
confidence intervals and paired significance tests. It is **not** the live Nav2
controller and carries the same scope limits as §VI-A: a 2D unicycle, no global
planner (the escape layer uses a free-space gap subgoal in lieu of Smac routing),
and a look-ahead-*point* safety certificate rather than full-body.

The live Gazebo full-matrix was excluded on this host deliberately: at RTF
0.3–0.6 under CPU contention it measures the host, not the algorithm. This 2D
benchmark is the honest quantitative surface; §VI-B reports the live integration
qualitatively.

One intentional, documented difference from the §VI-A `run_validation.run`
harness: **the CBF and the TTC estimate see only the dynamic obstacles** (nonzero
velocity), not the static walls. This matches the real Nav2 controller
(§IV-D and the §VI-B deployment lesson — feeding static walls into the CBF makes
the barrier infeasible and freezes the robot; static geometry is handled by the
sampling cost and the escape layer). The proto formulas themselves are unchanged
and `run_validation.py` is left untouched, so the §VI-A numbers stay reproducible.

## 2. Scenario families (seed-deterministic)

Each family is a pure function of an integer seed (same seed ⇒ byte-identical
world), so every config is evaluated on an *identical* scenario set — the paired
design McNemar requires. Generators: `experiments/benchmark2d/scenarios.py`.

- **`utrap`** — a finite U/C pocket (randomized width, side-wall depths, back-wall
  position, whole-pocket rotation, and goal-behind offset). The opening faces the
  robot and the goal sits *behind* the closed back wall, so greedy goal-descent
  drives the robot into a local minimum; escape requires rounding a side wall.
- **`clutter`** — a random field of 5–9 circular obstacles between start and goal.
  A footprint-clear start→goal path is **guaranteed** (checked by an
  occupancy-grid flood-fill at generation time; blocking obstacles are dropped
  until a path exists).
- **`dynamic`** — light static background plus 1–2 constant-velocity movers, each
  placed so it reaches a point on the robot's straight-line path at about the time
  the robot does (near-perpendicular crossing, pedestrian-like 0.35–0.58 m·s⁻¹,
  radius 0.24–0.36 m, timing jitter ±1.1 s). This is the coincidence that makes a
  *reactive* (current-position-only) planner cut the margin thin while a
  *velocity-aware* CBF anticipates.
- **`narrowdyn`** — the same U-trap pocket as `utrap` (identical static geometry
  per seed) **plus** 1–2 movers timed to sweep the pocket mouth during the escape
  window. This is the *narrow-AND-dynamic* coincidence the coordination
  contribution targets: the robot must execute an escape maneuver while a dynamic
  obstacle threatens the very route the escape uses.

## 3. Ablation configs (§VI / experiments/README A–F → 2D proto toggles)

Configs: `experiments/benchmark2d/configs.py`. Each is the exact toggle dict the
validated loop consumes.

| Config | escape | gap | CBF | α coord. | Role |
|---|---|---|---|---|---|
| **A · stock** | ✗ | ✗ | ✗ | ✗ | Nav2 stock-MPPI analogue |
| **C · escape** | ✓ | ✓ | ✗ | ✗ | escape detect-switch only |
| **D · CBF** | ✗ | ✗ | ✓ | ✗ | safety filter only |
| **E · indep.** | ✓ | ✓ | ✓ | ✗ | escape+CBF, α_escape = α_base |
| **F · SE-MPPI** | ✓ | ✓ | ✓ | ✓ | full coordination (α 2→6 + TTC override) |
| **F⁻ · no-gap** | ✓ | ✗ | ✓ | ✓ | F with gap search off |

The **load-bearing contrast is E vs F**: identical mechanisms, differing only in
whether the CBF gain is coordinated with the escape phase — this isolates the
coordination contribution (C2-coord).

## 4. Metrics and statistics

Per trial: success (reached without a footprint collision), collision, timeout,
time-to-goal, path length, minimum clearance, and — on the dynamic families — the
CBF slack and α history. Outcome classification follows
`experiments/runner/metrics` (COLLISION > SUCCESS > TIMEOUT).

- **Success** — Wilson 95% CI; F vs each baseline by **McNemar** on paired
  outcomes (primary test).
- **Continuous** — **Mann–Whitney U** with **Cliff's δ** effect size; time-to-goal
  and path length over the successful subset, min clearance over all trials.
- **Multiple comparisons** — **Holm–Bonferroni** within each family's test family.

Statistics reuse the unit-tested `experiments/analysis/stats`
(McNemar / Mann–Whitney / Cliff's δ / Holm) and the Wilson interval in
`experiments/analysis/aggregate`.

## 5. Scale and runtime

4 families × 50 seeds × 6 configs = **1,200 trials** (seeds 0–49 per family,
paired across configs). Control budget 400 steps × 0.1 s = 40 s sim time per
trial (matching `run_validation`'s budget; a run that exhausts it is a TIMEOUT).
Wall time ≈ 50 minutes with 6 worker processes on a 12-core host under heavy
external load; the runner appends per-scenario and resumes, so interrupted runs
lose nothing. Raw data: `experiments/results_2d/trials.csv`; summary:
`summary.csv`; statistics: `stats.json`; tables below: `tables.md`
(all auto-generated).

## 6. Results

The four blocks below reproduce the auto-generated
`experiments/results_2d/tables.md` (reordered utrap → clutter → dynamic →
narrowdyn; numbers verbatim). Continuous cells are mean ± 95% CI (normal
approximation); success carries a Wilson 95% CI; \* marks Holm-adjusted
p < 0.05. In the statistics tables, b/c are McNemar discordant counts
(b = baseline succeeded & F failed, c = baseline failed & F succeeded) and δ is
Cliff's delta of F relative to the baseline.

### U-trap (static local minima) — N = 50/config

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |
| C · escape only | 90% [79%, 96%] | 0% | 10% | 31.34 ± 1.36 | 8.12 ± 0.33 | 0.32 ± 0.00 |
| D · CBF only | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |
| E · escape+CBF (indep.) | 88% [76%, 94%] | 0% | 12% | 31.20 ± 1.36 | 8.09 ± 0.32 | 0.32 ± 0.00 |
| F · SE-MPPI (coord.) | 88% [76%, 94%] | 0% | 12% | 31.20 ± 1.36 | 8.09 ± 0.32 | 0.32 ± 0.00 |
| F⁻ · F, gap off | 0% [0%, 7%] | 0% | 100% | — | — | 0.33 ± 0.00 |

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/44 | 9.02e-11 (1.44e-09) \* | +0.00 (1) | +0.00 (1) | −0.44 (0.00195) \* |
| C_escape→F | 1/0 | 1 (1) | −0.02 (1) | −0.01 (1) | −0.00 (1) |
| D_cbf→F | 0/44 | 9.02e-11 (1.44e-09) \* | +0.00 (1) | +0.00 (1) | −0.44 (0.00195) \* |
| E→F (**key**) | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | +0.00 (1) |

### Clutter (random static field) — N = 50/config

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |
| C · escape only | 62% [48%, 74%] | 0% | 38% | 17.67 ± 2.11 | 6.14 ± 0.32 | 0.35 ± 0.02 |
| D · CBF only | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |
| E · escape+CBF (indep.) | 62% [48%, 74%] | 0% | 38% | 17.67 ± 2.11 | 6.14 ± 0.32 | 0.35 ± 0.02 |
| F · SE-MPPI (coord.) | 62% [48%, 74%] | 0% | 38% | 17.67 ± 2.11 | 6.14 ± 0.32 | 0.35 ± 0.02 |
| F⁻ · F, gap off | 52% [39%, 65%] | 0% | 48% | 15.23 ± 0.49 | 5.81 ± 0.18 | 0.36 ± 0.02 |

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/5 | 0.0625 (1) | +0.16 (1) | +0.16 (1) | −0.03 (1) |
| C_escape→F | 0/0 | 1 (1) | +0.00 (1) | −0.00 (1) | +0.00 (1) |
| D_cbf→F | 0/5 | 0.0625 (1) | +0.16 (1) | +0.16 (1) | −0.03 (1) |
| E→F (**key**) | 0/0 | 1 (1) | +0.00 (1) | +0.00 (1) | +0.00 (1) |

### Dynamic (crossing movers) — N = 50/config

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 80% [67%, 89%] | 20% | 0% | 22.48 ± 1.44 | 7.99 ± 0.55 | 0.12 ± 0.03 |
| C · escape only | 76% [63%, 86%] | 20% | 4% | 21.87 ± 1.36 | 7.75 ± 0.53 | 0.13 ± 0.03 |
| D · CBF only | 82% [69%, 90%] | 16% | 2% | 22.74 ± 1.52 | 8.02 ± 0.65 | 0.13 ± 0.03 |
| E · escape+CBF (indep.) | 78% [65%, 87%] | 18% | 4% | 21.77 ± 1.25 | 7.56 ± 0.47 | 0.13 ± 0.03 |
| F · SE-MPPI (coord.) | 78% [65%, 87%] | 18% | 4% | 21.78 ± 1.25 | 7.57 ± 0.47 | 0.13 ± 0.03 |
| F⁻ · F, gap off | 84% [71%, 92%] | 16% | 0% | 23.01 ± 1.66 | 8.14 ± 0.68 | 0.13 ± 0.03 |

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 3/2 | 1 (1) | −0.08 (1) | −0.15 (1) | +0.02 (1) |
| C_escape→F | 1/2 | 1 (1) | −0.00 (1) | −0.06 (1) | +0.02 (1) |
| D_cbf→F | 3/1 | 0.625 (1) | −0.09 (1) | −0.09 (1) | −0.01 (1) |
| E→F (**key**) | 0/0 | 1 (1) | −0.01 (1) | +0.01 (1) | +0.01 (1) |

### Narrow-dynamic (U-trap + movers on the escape route) — N = 50/config

| Config | Success (95% CI) | Collision | Timeout | Time-to-goal (s) | Path length (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| A · stock MPPI | 0% [0%, 7%] | 18% | 82% | — | — | 0.22 ± 0.04 |
| C · escape only | 78% [65%, 87%] | 18% | 4% | 30.52 ± 1.30 | 8.33 ± 0.44 | 0.21 ± 0.04 |
| D · CBF only | 0% [0%, 7%] | 32% | 68% | — | — | 0.19 ± 0.04 |
| E · escape+CBF (indep.) | 64% [50%, 76%] | 32% | 4% | 30.55 ± 1.50 | 8.24 ± 0.55 | 0.18 ± 0.04 |
| F · SE-MPPI (coord.) | 62% [48%, 74%] | 32% | 6% | 30.36 ± 1.51 | 8.11 ± 0.50 | 0.18 ± 0.04 |
| F⁻ · F, gap off | 0% [0%, 7%] | 32% | 68% | — | — | 0.19 ± 0.04 |

| Contrast | Success b/c | McNemar p (adj) | Time δ (p adj) | Path δ (p adj) | Clearance δ (p adj) |
|---|---|---|---|---|---|
| A_stock→F | 0/31 | 7.12e-08 (1.14e-06) \* | +0.00 (1) | +0.00 (1) | −0.11 (1) |
| C_escape→F | 8/0 | 0.00781 (0.109) | −0.03 (1) | −0.12 (1) | −0.06 (1) |
| D_cbf→F | 0/31 | 7.12e-08 (1.14e-06) \* | +0.00 (1) | +0.00 (1) | −0.05 (1) |
| E→F (**key**) | 1/0 | 1 (1) | −0.03 (1) | −0.03 (1) | +0.00 (1) |

## 7. Interpretation

**The escape layer is the measured contribution; gap search is load-bearing.**
On both trap families the effect is decisive and survives Holm correction:
success goes from 0% (stock A, CBF-only D, and no-gap F⁻ all time out in the
pocket) to 88–90% (utrap, 44/50 discordant pairs, adj. p = 1.4×10⁻⁹) and 62–78%
(narrow-dynamic, 31/50 discordant, adj. p = 1.1×10⁻⁶). F⁻ scoring 0% on both
trap families shows the detect-and-switch *plus the gap subgoal* is what
escapes; APF and α modulation without a routed opening do not. On clutter the
escape adds 10 pp (52%→62%, raw p = 0.0625, not significant after Holm) at the
cost of ~2.4 s longer average time-to-goal — the detector occasionally fires on
slow progress and detours.

**The key contrast — E (independent) vs F (coordinated) — is null in this
benchmark.** Across all 200 paired scenarios the discordant counts are 0/0,
0/0, 0/0, and 1/0 (McNemar p = 1 everywhere), and every continuous effect size
is |δ| ≤ 0.03. The α raise does engage exactly as designed (α_max = 6 during
escape phases, TTC override active), but the trials show the barrier constraint
almost never *binds* during these escapes — the same phenomenon §VI-A's
mechanism plot shows as "α: 2→6 with slack ≈ 0". When the constraint is slack,
the QP returns the nominal command regardless of α, so E and F issue nearly
identical controls. We therefore report honestly: **in this 2D benchmark the
coordination produces no measurable outcome difference over independent
escape+CBF.** The coordination's demonstrated value remains mechanism-level
(the escape maneuver is admitted *while staying certified-safe*, §VI-A); any
outcome-level benefit would have to come from regimes where the barrier
actively binds during escape (faster/persistent movers, tighter margins), which
this generator's pedestrian-like movers do not produce — a scope statement, not
evidence of benefit.

**Two negative findings about the CBF worth carrying into §VII.** First,
against body-grazing crossers (dynamic family) the look-ahead-point filter does
not measurably reduce collisions (20% → 16–18%, n.s.): the certificate protects
the look-ahead point, and the grazes occur at the robot body — the known
relative-degree/full-body limitation. Second, in the cramped narrow-dynamic
pocket the CBF configurations *raise* the collision rate over their no-CBF
counterparts (18% → 32%; F vs C: b/c = 8/0, raw p = 0.008, Holm-adj. 0.109) and
cost ~16 pp success (78% → 62%): when the QP goes infeasible the filter's fallback brakes
the robot (v = 0), and a mover sweeping the pocket then hits a robot that the
unfiltered sampler would have moved out of the way. Braking is not a safe
fallback when cornered — this is measured, directionally consistent, though not
significant after multiple-comparison correction at N = 50.

## 8. Figures

- `experiments/results_2d/figures/success_bars.png` — per-family success /
  collision / timeout across A–F (Wilson-95% error bars on success).
- `experiments/results_2d/figures/e_vs_f.png` — the coordination contrast
  (E vs F) per family, annotated with the McNemar p-value.
- `experiments/results_2d/figures/traj_montage.png` — representative
  trajectories (stock A, top vs coordinated F, bottom) on one seed per family
  (utrap s0, clutter s3, dynamic s37, narrowdyn s0), regenerated live from the
  seed so the drawing is exactly the measured run: A stalls at the pocket mouth
  / collides with the crosser; F rounds the pocket and dodges.
