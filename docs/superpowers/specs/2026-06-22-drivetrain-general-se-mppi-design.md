# Design Spec — Drivetrain-General SE-MPPI (two-paper split)

> **Date:** 2026-06-22
> **Status:** design approved (brainstorming); pre-implementation-plan.
> **Author/driver:** kangjmo (via brainstorming session)
> **Origin:** Assessing whether the SE-MPPI repo/paper can be "upgraded via" the
> research loop `~/kangj/loops/amr-path-planning-with-2d-3d-lidar-and-vision-across-drive-types`
> (35 sources, saturated). Finding: the loop is a broad AMR *path-planning + perception*
> corpus, largely disjoint from paper 1's MPPI/CBF/escape literature; it does **not**
> drive a rewrite of paper 1. The loop's drive-kinematics + modality veins instead
> justify a **new companion paper** ("Drivetrain-General SE-MPPI"), and the broadening
> is best done as a **two-paper split** rather than by widening paper 1.
> **Citation reliability:** external numbers self-reported / search-derived unless
> marked verified; confirm against primary PDFs before camera-ready (repo convention).

---

## 1. Decision summary

The user wanted to "broaden SE-MPPI's scope" using the loop, across **both axes**
(drive types + sensor modalities), at **theory-general / demo-narrow** depth. During
design we recognized that the §2 method abstraction exposes a clean seam: the
*coordination contribution is drivetrain-independent*, while the drivetrain-specific
content lives entirely in the plant map and admissible control set. We therefore
**split into two papers** rather than widen paper 1:

- **Paper 1 — unchanged, narrow, finish it.** Keeps the verified four-way novelty
  (Nav2-native · escape · dynamic CBF · α-coordination) on differential drive. Only
  remaining work: run the already-specified §VI-C benchmark. Add one forward-pointer
  sentence to a companion paper.
- **Paper 2 — new.** "Drivetrain-General SE-MPPI." Carries the omni/Ackermann method,
  the modality-agnostic-interface story, and *measured* cross-drivetrain evaluation.
  Its standalone novelty hook is a **certified-safe escape under nonholonomic
  (Ackermann) curvature constraints**, not mere per-drivetrain instantiation.

Approach chosen over: (B) one broadened co-headline paper, (C-narrow) split where
paper 2 is *only* the nonholonomic escape result.

---

## 2. Paper 1 — disposition (minimal change)

No repositioning. Title, scope, and the four-way novelty stand as in
`docs/papers/2026_se-mppi-paper-draft.md`.

- **Remaining work:** the §VI-C large-scale benchmark (BARN / DynaBARN / HuNavSim),
  exactly as already specified (currently *pending*). No new claims.
- **One edit:** §VIII Conclusion gains a sentence noting that drivetrain
  generalization (omni/Ackermann) and modality-agnostic operation are addressed in a
  companion paper (paper 2 below), alongside the existing SE-Predict / Multi-SE-MPPI
  pointers.
- **Why unchanged:** paper 1's entire defensible value is the *conservative, verified*
  novelty; widening it would dilute that and invite "Nav2 is already
  drivetrain/modality-agnostic" scrutiny. The split protects the asset.

---

## 3. Paper 2 — Drivetrain-General SE-MPPI (full design)

### 3.1 Title & framing
**Working title:** *"Drivetrain-General Safe-Escape MPPI: Certified-Safe Coordinated
Escape Across Differential, Omnidirectional, and Ackermann AMRs."*

**Relationship to paper 1.** P2 *inherits* the coordination thesis and the
forward-invariance proposition (cites P1; recalls — does not re-derive — the α
schedule, the slack-QP, and the proof). Central question: *does certified-safe
coordinated escape survive across wheeled-mobile-robot (WMR) mobility classes, and
what new machinery does each class demand?*

### 3.2 Thesis
The coordination core is drivetrain-independent, but the plant `(J, U)` is not — and
the **nonholonomic (Ackermann) case breaks naive escape**: rounding a non-convex
(U-shaped) trap may require curvature-feasible, possibly *reversing* maneuvers that
must still preserve forward invariance (h ≥ 0). Repairing escape for that case, while
keeping the certificate, is the new science.

### 3.3 The unifying abstraction
Every per-obstacle barrier takes one form; the drivetrain enters *only* through the
plant map `J(θ)` and the admissible control set `U`:

```
h_o   = ||d||^2 - R^2
ḣ_o   = 2 dᵀ ( J(θ) ũ - v_o )
CBF:    ḣ_o + α h_o ≥ 0           (linear in the drivetrain control ũ)
```

The slack-QP, the coordinator's α schedule, and the forward-invariance proposition
are **drivetrain-independent** (they need only α > 0 and a feasible QP). Hence
paper 1's core contribution (C2) transfers verbatim; generalization lives in `(J, U)`.

| Drivetrain | Control ũ | J / safe set U | Escape geometry | P2 status |
|---|---|---|---|---|
| Differential (unicycle) | (v, ω) | look-ahead point P=(x,y)+L(cosθ,sinθ); J=[[cosθ,−L sinθ],[sinθ, L cosθ]]; **box** U | in-place rotation OK; gap-attraction as in P1 | **measured (from P1)** |
| Omnidirectional (mecanum) | (vx, vy, ω) | **body position is relative-degree 1 directly** (no look-ahead): J=R(θ) acting on (vx,vy); ω decoupled from positional barrier | holonomic — *sidestep* toward a gap without reorienting; U-traps far easier | implement + measure |
| Ackermann (car-like) | (v, δ), ω = v·tan δ / L_wb | front-axle look-ahead; U is a **curvature cone** \|ω/v\| ≤ κ_max (not a box) | nonholonomic, no in-place turn; gap subgoals must be curvature-reachable; tight traps may need a **reverse maneuver** | implement + measure (headline, D2) |

### 3.4 Contributions
- **D1 — Generalization.** The `(J, U)` abstraction unifying differential /
  omnidirectional / Ackermann under one coordinated escape+CBF QP; forward invariance
  shown drivetrain-independent (recalled from P1).
- **D2 — Certified-safe nonholonomic escape (headline novelty).** (i) Gap subgoals
  restricted to a curvature-reachable set (Reeds-Shepp/Dubins feasibility); (ii) a
  reverse-permitting detect-and-switch escape; (iii) a proof that forward invariance
  survives **control-set / gear switching** — extending P1's piecewise-constant-α
  chaining to piecewise-`U` switching. *Proof strategy:* on each segment a feasible QP
  maintains ḣ ≥ −α h with α > 0 ⇒ h non-decreasing across zero; each switch (curvature
  set change, forward↔reverse) starts from h ≥ 0 and preserves it; chain the segments.
  **(To be formalized — supplementary; marked pending until written, per repo
  convention.)**
- **D3 — Holonomic exploitation.** Omni escape exploits lateral mobility (sidestep
  gap-attraction) with ω decoupled from the positional CBF; quantify the escape-time /
  success advantage over the diff baseline in the same traps.
- **D4 — Modality-agnostic interface.** The controller is unchanged across costmap
  layers built from 2D LiDAR, 3D LiDAR/depth, and vision; *argued* at the costmap
  interface and *spot-measured* on one or more layers (final count is an open decision
  — see §6 Q4).
- **D5 — Cross-drivetrain benchmark.** Measured evaluation across all three
  drivetrains on the shared harness (this is the eval that did not fit paper 1's
  demo-narrow scope and is paper 2's job).

### 3.5 Method outline (P2 §IV)
- §IV-A — recall coordination core (cite P1) + the `(J, U)` abstraction (§3.3).
- §IV-B — **Omni:** body-frame relative-degree-1 barrier; ω decoupled; holonomic
  gap-attraction (lateral subgoal).
- §IV-C — **Ackermann (D2):** front-axle look-ahead; curvature cone added to the QP;
  curvature-reachable gap-subgoal filter; reverse gear in the escape critic; the
  switching forward-invariance proof.
- §IV-D — **Modality interface:** escape APF (Dijkstra distance on the 2-D costmap)
  and gap raycast operate on the costmap regardless of source layer; honest caveat
  that modality changes the *tracker's input quality* (3D→2D projection, vision
  false positives), not the controller interface.

### 3.6 Modality-agnosticism — argued, not built
Both mechanisms attach to the **Nav2 costmap** and the **costmap-cell tracker**, never
to raw sensors. Layers that realize each modality: 2D LiDAR → ObstacleLayer; 3D
LiDAR/depth → VoxelLayer / spatio_temporal_voxel_layer (STVL) / nvblox costmap layer;
vision → BEV/semantic layer (BEVNav-style). SE-MPPI is unchanged across these; only
the upstream layer differs. **This respects the repo's "perception lives elsewhere"
rule — we consume layers, we do not build them.** Claim is *measured on LiDAR, argued
(and spot-measured on one or more layers — count per §6 Q4) for 3D/vision*.

### 3.7 Implementation changes (real code; beyond demo-narrow)
- Refactor `src/nav2_se_controller/.../cbf_safety_filter` to assemble `(J, U)` from a
  **drivetrain enum**; the differential path stays byte-for-byte as the P1-measured
  instance.
- Add **omni** path (body-frame barrier, decoupled ω) and **Ackermann** path
  (front-axle look-ahead, curvature-cone constraint in the QP).
- Add **reverse-maneuver support** to the escape critic + curvature-reachable
  gap-subgoal filter (Reeds-Shepp/Dubins feasibility check).
- Drivetrain config plumbing; **omni + Ackermann Gazebo robot models** for sim eval.
- No perception code added (modality stays an interface argument + config of existing
  Nav2 layers).

### 3.8 Experiments (P2 §VI)
- Cross-drivetrain on BARN / DynaBARN with diff/omni/Ackermann sim models; metrics as
  in P1 (success, collision, time, path length, min clearance, per-cycle compute).
- Modality on ≥2 costmap layers (e.g., 2D ObstacleLayer vs VoxelLayer/STVL).
- **Key ablation:** curvature-naive vs curvature-feasible escape (isolates D2).
- Reuse paper 1's evaluation harness; honest measured-vs-argued markers throughout.

### 3.9 Where the research loop plugs in (related-work grounding for P2)
This is the concrete "upgrade via the loop" payoff — the loop's drive-kinematics and
modality veins feed paper 2's Related Work / grounding:

| Loop source | Grounds |
|---|---|
| Campion, Bastin & D'Andréa-Novel 1996 — 5-class WMR mobility taxonomy | D1 (why drivetrains differ) |
| ROS2-control wheeled mobile robot kinematics (diff/omni/Ackermann) | the kinematic models / `(J, U)` |
| Holonomic ORCA + non-holonomic ORCA; mecanum holonomic planning (Heliyon 2024) | D3 (omni / holonomy) |
| Reeds-Shepp / Dubins / Hybrid-A* / Pivtoraiko–Kelly state lattice | D2 (curvature reachability; branch from sibling-loop canon) |
| STVL / nvblox / VoxelLayer / BEVNav | D4 (modality layers) |
| Nav2 MPPI multi-drivetrain (`vy_max`, Ackermann) + Nav2 tuning/plugin docs | controller drivetrain support + engineering |

(Reeds-Shepp/Dubins/Hybrid-A*/state-lattice are in the sibling loop's OUT-OF-SCOPE
canon; cite from there. The current loop's in-scope new hits are Campion, holonomic
ORCA, mecanum-holonomic, Pivtoraiko, and the modality layers.)

---

## 4. Dependencies, sequencing, risks

- **Sequencing:** P2 depends on P1's benchmark harness and ideally P1 submitted first
  (P2 cites P1's proposition). Order: finish P1 §VI-C → build P2 method/infra → P2 eval.
- **Infra effort (P2-specific):** omni + Ackermann Gazebo models and ≥2 costmap layers
  are real work beyond P1.
- **Standalone-novelty risk (P2):** per-drivetrain CBF instantiation is mostly known
  technique; **D2 is the load-bearing novelty** and must hold up. If D2's switching
  proof or the reverse-escape does not pan out, fall back to the "Split, paper 2 =
  narrow novelty" option (D2 alone as the paper).
- **Honesty:** every cross-axis result is measured-or-marked-pending; no unmeasured
  claims (repo rule).
- **Series numbering:** repo already has SE-Predict (paper 2 of series) and
  Multi-SE-MPPI (paper 3). This drivetrain paper needs its own track slot; **number
  to be assigned by the user** (named "Drivetrain-General SE-MPPI" here).
- **Build/branch rule:** development branch `claude/fervent-newton-lbo96`; do not push
  to other branches. Interfaces implemented against installed Jazzy headers (critic
  costs = xtensor, not main-branch Eigen).

---

## 5. Out of scope (explicitly not doing)

- Not widening paper 1 (decision: split, not broaden in place).
- Not building perception / SLAM / mapping / learned-nav (repo delegates these to
  other repos; modality stays an interface argument).
- Not the fleet (MAPF/VDA5050) or social-nav veins of the loop — those belong to the
  extension tracks (Multi-SE-MPPI / SE-Predict), not this drivetrain paper.
- Not a survey paper (Approach D was rejected).

---

## 6. Open questions for user review

1. Series number/slot for paper 2 ("Drivetrain-General SE-MPPI").
2. How much omni/Ackermann eval is *required* for submission vs. acceptable as
   "spot-measured + future work" — affects infra effort.
3. Target venue for paper 2 (RA-L+ICRA like P1, or a venue friendlier to a
   methods+generalization paper).
4. Whether D4 (modality) must be measured on ≥2 layers for v1, or argued-only with one
   demonstrated layer.
