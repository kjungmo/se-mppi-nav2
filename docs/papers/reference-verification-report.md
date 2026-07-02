# SE-MPPI Reference Verification Report

**Date:** 2026-07-02
**Bibliography:** `docs/papers/references.bib` (30 entries, all verified/corrected)
**Method:** Two-pass verification — a verifier pass followed by an adversarial skeptic pass. The skeptic's bibtex wins where present. Entries with a final verdict of `unverified` are excluded from the bib.

## Summary counts

| Outcome | Count |
|---|---|
| Verified (final) | 17 |
| Corrected (final) | 13 |
| Unverified (excluded) | 0 |
| **Total in bib** | **30** |

No entries were dropped as unverified — all 30 claims resolved to a real, checkable arXiv/DOI/DBLP record.

## Per-reference results

| Key | Claimed | Final verdict | Venue kept? | What changed |
|---|---|---|---|---|
| acp_yang | Yang et al., Safety-Critical Control w/ ACP, ACP/ACC 2024, arXiv:2407.03569 | corrected | yes (ACC 2024) | Authors corrected — real authors are Zhou, Zhang, Luo (no "Yang"); key is now a misnomer but kept per instruction |
| ames_cbf_ecc | Ames et al., CBFs: Theory & Applications, ECC 2019, arXiv:1903.11199 | corrected | yes (ECC 2019) | Venue confirmed via Crossref DOI 10.23919/ECC.2019.8796030; authors/pages filled in |
| ames_cbf_tac | Ames et al., CBF-QP safety-critical, IEEE TAC 2017 | verified | yes (IEEE TAC) | None; vol/no/pages confirmed via DOI |
| barn | BARN benchmark, arXiv:2008.13315 | corrected | no | Title/authors supplied; SSRR 2020 venue NOT asserted (no arXiv journal-ref) → @misc |
| barn_challenge | 3rd BARN Challenge report, arXiv:2407.01862 | verified | no | ICRA 2024 appears in title only (event reported, not venue) → @misc |
| biased_mppi | Biased-MPPI, arXiv:2401.09241 | corrected | yes (RA-L) | **Upgraded** @misc→@article; skeptic found RA-L acceptance + DOI 10.1109/LRA.2024.3397083 the verifier missed |
| br_mppi | BR-MPPI, arXiv:2506.07325 | verified | no | None; preprint only |
| cbfkit | CBFKit toolbox, arXiv:2404.07158 | verified | no | None; preprint only |
| conformal_hri | Safe Probabilistic Planning w/ Conformal Risk Control, arXiv:2603.10392 (2026) | verified | no | Confirmed 2026 paper exists; preprint only |
| dixit | Dixit et al., Adaptive CP for motion planning, L4DC 2023, arXiv:2212.00278 | verified | no | L4DC 2023 NOT confirmed from arXiv metadata → @misc |
| dpcbf | DPCBF "Beyond Collision Cones", ICRA 2026, arXiv:2510.01402 | verified | yes (noted) | Full title used; ICRA 2026 acceptance kept as `note`, still @misc (no proceedings DOI yet) |
| drpa_mppi | DRPA-MPPI, IEEE CASE 2025, arXiv:2503.20134 | corrected | no | CASE 2025 was "submitted to", not accepted → @misc, venue removed |
| dualguard | DualGuard MPPI, arXiv:2502.01924 | corrected | yes (RA-L) | Authors/venue filled; RA-L Vol 10 Iss 7 confirmed via arXiv journal-ref → @article |
| dwa | Dynamic Window Approach, Fox/Burgard/Thrun, IEEE RAM 1997 | verified | yes (IEEE RAM) | None; pre-arXiv, DOI 10.1109/100.580977 |
| dynabarn | DynaBARN, Nair et al. | corrected | yes (SSRR 2022) | No arXiv exists; canonical is SSRR 2022, DOI 10.1109/SSRR56537.2022.10018758 → @inproceedings |
| flow_mppi | "FlowMPPI (Power & Berenson)" | corrected | yes (CoRL 2022) | **Misattributed/fabricated original** — real paper is Sacks & Boots, "Learning Sampling Distributions for MPC", CoRL 2022, arXiv:2212.02587 |
| gs_mppi | GS-MPPI, arXiv:2410.02154 | verified | no | ACC 2025 was "submitted to" only → preprint |
| hunavsim | HuNavSim, arXiv:2305.01303 | verified | yes (noted) | RA-L acceptance kept as `note`; @misc (no vol/pages/DOI) |
| lindemann | Lindemann et al., Safe Planning w/ CP, RA-L 2023, arXiv:2210.10254 | corrected | no | **RA-L 2023 venue could NOT be confirmed** from arXiv; year set to 2022 (submission) → @misc |
| log_mppi | log-MPPI, arXiv:2203.16599 | verified | yes (RA-L) | None; RA-L confirmed via arXiv comment; vol/pages omitted |
| mppi_orig | Williams et al., original MPPI (ICRA 2017 / T-RO) | corrected | no | Resolved conflation to arXiv:1707.02342; "submitted to T-RO" only → @misc, no venue asserted |
| nav2 | Marathon 2 nav system, IROS 2020 | verified | yes (IROS 2020) | None; venue confirmed via arXiv journal-ref |
| osqp | OSQP solver, Math Prog Comp 2020 | verified | yes (MPC journal) | None; DOI 10.1007/s12532-020-00179-2 confirms venue |
| rpp | Regulated Pure Pursuit, Macenski et al. | corrected | yes (Autonomous Robots) | **Upgraded** @misc→@article; skeptic confirmed arXiv journal-ref "Autonomous Robots 2023" (verifier's v3-404 was a red herring) |
| scbf_mppi | reach-avoid stochastic CBF + MPPI, arXiv:2407.13693 | verified | no | None; preprint only |
| shield_mppi | Shield-MPPI, arXiv:2302.11719 | verified | no | "Submitted to RA-L" only → preprint |
| svg_mppi | SVG-MPPI, arXiv:2309.11040 | verified | no | None; preprint only |
| teb | TEB local planner, Roesmann et al. | corrected | yes (ROBOTIK 2012) | Canonical founding paper is ROBOTIK 2012 (not the claimed ECMR 2015/2017); 5 authors confirmed via DBLP |
| tsallis_mppi | Tsallis-MPPI / VI-MPC w/ Tsallis divergence | corrected | no | Canonical id found: Wang et al., arXiv:2104.00241; RSS 2021 commonly cited but unconfirmed → @misc |
| ua_pcbf | UA-PCBF, arXiv:2508.20812 | verified | no | None; preprint only |

## Unverified entries (excluded from paper)

**None.** Every claimed reference resolved to a verifiable record. No entry needs to be removed from the paper.

Two entries required a **replacement / re-identification** rather than removal (recorded above as corrected):
- `flow_mppi` — the claimed "FlowMPPI" by "Power & Berenson" could not be found on arXiv or elsewhere in either pass (likely fabricated/misattributed). Replaced with the real paper matching the described concept: Sacks & Boots, "Learning Sampling Distributions for Model Predictive Control," CoRL 2022 (arXiv:2212.02587). **Action:** ensure the paper text attributes this to Sacks & Boots, not Power & Berenson.
- `acp_yang` — no author named "Yang" exists on the paper; true authors are Zhou, Zhang, Luo. The citation key is retained but is a misnomer. **Action (optional):** rename key to e.g. `acp_zhou24`.

## Venue notes

### Venue downgrades (claimed venue NOT asserted in bib → cited as arXiv preprint)
- **drpa_mppi** — claimed IEEE CASE 2025; arXiv says "submitted to", not accepted. Downgraded to @misc.
- **lindemann** — claimed IEEE RA-L 2023; no journal-ref on arXiv and no independent IEEE Xplore record locatable. Downgraded to @misc (year also corrected 2023→2022 submission year).
- **dixit** — claimed L4DC 2023; not confirmable from arXiv metadata. Kept as @misc.
- **mppi_orig** — "submitted to T-RO" only; no journal-ref/DOI. Kept as @misc arXiv preprint.
- **gs_mppi**, **shield_mppi** — "submitted/preprint to" ACC 2025 / RA-L respectively; not accepted. Kept as @misc.
- **barn** (SSRR 2020) and **barn_challenge** (ICRA 2024 in title) — no arXiv-backed venue; kept as @misc.
- **tsallis_mppi** — RSS 2021 commonly cited in secondary sources but unconfirmed by primary metadata. Kept as @misc.

### Venue upgrades (skeptic strengthened over verifier)
- **biased_mppi** — @misc → @article (IEEE RA-L, DOI 10.1109/LRA.2024.3397083). Verifier missed the arXiv "Accepted for RA-L" comment + related DOI.
- **rpp** — @misc → @article (Autonomous Robots 2023). Skeptic re-fetched and found the arXiv journal-ref field the verifier's summary fetch missed.

### Acceptance-noted but kept as @misc (no proceedings DOI/vol/pages yet)
- **dpcbf** — accepted to ICRA 2026 (noted); no proceedings record yet.
- **hunavsim** — accepted to RA-L (noted); no vol/issue/pages/DOI on arXiv.
