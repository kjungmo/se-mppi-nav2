# SE-MPPI — Escape–Safety Coordination: Forward-Invariance Proof (C2 supplementary)

> **Status:** formal note, 2026-06-13. Supplementary to the paper draft §III/§V (claim **C2**).
> **Notation is reconciled to the implementation** — symbols match `cbf_safety_filter.cpp`,
> `cbf_types.hpp`, `escape_safety_coordinator.hpp`. Where the draft and the code disagree,
> the code is treated as ground truth and the discrepancy is flagged in §5.
> **Scope:** this note proves the *load-bearing* claim of C2 — that gain coordination preserves
> the safety certificate — and states honestly the regime in which it holds.
> **Reconciliation:** the draft already states this as *Proposition (certified-safe escape)* in
> §IV-E with a proof sketch. The statements agree; this note is the expanded, rigorous version the
> draft should reference. The proof here uses an integrating-factor argument that covers any
> measurable $\alpha(t)$ directly (no piecewise-constant chaining), and corrects the slack to be
> shared (§5.1), matching the implementation.

## 1. Setup (as implemented)

Robot state $x=(p_x,p_y,\theta)$, control $u=(v,\omega)$. The CBF acts on a **look-ahead point**
a distance $L>0$ ahead of the base (`cfg_.lookahead`, default $0.2\,$m):
$$p_L = \big(p_x + L\cos\theta,\; p_y + L\sin\theta\big),\qquad \dot p_L = G(\theta)\,u,$$
$$G(\theta)=\begin{bmatrix}\cos\theta & -L\sin\theta\\ \sin\theta & L\cos\theta\end{bmatrix},\qquad \det G = L>0.$$
$\det G=L>0$ is why the look-ahead point makes the unicycle control-affine and the
relative-degree-1 CBF well-posed (the map $u\mapsto\dot p_L$ is invertible).

For a tracked obstacle $j$ with position $p_j(t)$, constant velocity $v_j$, radius $r_j$, define the
**effective radius** and **barrier** (`cbf_safety_filter.cpp:58,60`):
$$R_j = \rho + r_j + m + q_j,\qquad h_j(x,t) = \lVert p_L - p_j(t)\rVert^2 - R_j^2,$$
where $\rho$ = robot inscribed radius, $m$ = safety margin, $q_j\ge 0$ = the conformal bound
(0 if uncalibrated). The safe set is $\mathcal C=\{x:\,h_j(x)\ge 0\ \forall j\}$.

With $d_j = p_L - p_j$, the barrier rate is
$$\dot h_j = 2\,d_j^\top(\dot p_L - v_j) = 2\,d_j^\top\big(G(\theta)\,u - v_j\big).$$

The QP (single-robot share $\lambda=1$) enforces, per cycle, the **continuous-time CBF condition**
relaxed by a **single, shared** slack $\delta\ge 0$ (`cbf_safety_filter.cpp:65,155`):
$$\boxed{\;\dot h_j + \alpha\, h_j \;\ge\; -\,\delta\;}\qquad\Longleftrightarrow\qquad
\underbrace{2\,d_j^\top G}_{A_j}\,u + \delta \;\ge\; \underbrace{-\alpha h_j + 2\,d_j^\top v_j}_{b_j},$$
with $\alpha>0$ the class-$\mathcal K$ gain. The slack is **one decision variable shared by all
obstacle rows** (`cbf_safety_filter.cpp:111` `n_vars=3`; every row loads the same column `:162`),
not one per obstacle. The QP minimizes
$w_v(v-v_{\text{nom}})^2 + w_\omega(\omega-\omega_{\text{nom}})^2 + \rho_\delta\,\delta^2$
subject to these rows and box limits. The **escape–safety coordinator** supplies $\alpha$ each
cycle (`escape_safety_coordinator.hpp:89`):
$$\alpha(t)=\begin{cases}
\alpha_{\text{base}} & \text{not entrapped}\\
\alpha_{\text{base}} & \text{entrapped} \wedge \big(\mathrm{TTC}<\tau \;\vee\; \max_j q_j > \bar q\big)\\
\alpha_{\text{esc}} & \text{entrapped} \wedge \mathrm{TTC}\ge\tau \wedge \max_j q_j\le \bar q
\end{cases}
\qquad 0<\alpha_{\text{base}}\le\alpha_{\text{esc}}<\infty.$$
(Defaults: $\alpha_{\text{base}}=2.0$, $\alpha_{\text{esc}}=6.0$, $\tau=1.5\,$s, $\bar q=0.25\,$m.)
The schedule is piecewise-constant, always **strictly positive and bounded**.

## 2. Proposition (coordination preserves forward invariance)

> **Proposition.** Fix an obstacle $j$ and suppose the QP is feasible *without slack*, i.e. the
> shared $\delta(t)=0$, so that $\dot h_j(t)\ge -\alpha(t)\,h_j(t)$ for a.e. $t\ge0$. Let
> $\alpha:[0,\infty)\to[\alpha_{\min},\alpha_{\max}]\subset(0,\infty)$ be **any** measurable,
> positive, bounded gain schedule. If $h_j(0)\ge 0$, then $h_j(t)\ge 0$ for all $t\ge0$.

**Proof.** Define the (strictly positive, finite) integrating factor
$\varphi(t)=\exp\!\big(\int_0^t \alpha(s)\,ds\big)$; finiteness holds because $\alpha\le\alpha_{\max}$.
Then, a.e.,
$$\frac{d}{dt}\big[\varphi(t)\,h_j(t)\big] = \varphi(t)\big[\dot h_j(t)+\alpha(t)h_j(t)\big]\ \ge\ 0.$$
Hence $\varphi(t)h_j(t)$ is non-decreasing, so $\varphi(t)h_j(t)\ge\varphi(0)h_j(0)=h_j(0)\ge0$.
Since $\varphi(t)>0$, $h_j(t)\ge0$. $\qquad\blacksquare$

> **Corollary (the C2 claim).** Forward invariance of $\mathcal C_j=\{h_j\ge0\}$ holds for *every*
> positive bounded gain schedule — in particular for the coordinator's switching between
> $\alpha_{\text{base}}$ and $\alpha_{\text{esc}}$, and for the TTC / prediction-trust overrides
> that drop $\alpha$ back to $\alpha_{\text{base}}$. **Raising $\alpha$ never permits $h_j<0$;** it
> only weakens the lower bound on $\dot h_j$, letting the robot approach $\partial\mathcal C_j$
> faster (bolder detours). Thus *escape aggressiveness and the safety certificate are decoupled*:
> the gain trades conservativeness for maneuverability within an invariant safe set. This — not the
> mere sum of an escape layer and a CBF layer — is the contribution.

**Multiple obstacles.** When the QP returns $\delta=0$ on all rows, every $h_j$ satisfies the
hypothesis simultaneously, so $\mathcal C=\bigcap_j\mathcal C_j$ is invariant. The proof is per-row
and gain-agnostic, so a single shared $\alpha(t)$ across rows is immaterial.

## 3. Why the gain is the right coordination knob

The escape layer (EscapeCritic) pushes samples *toward* $\partial\mathcal C$; the CBF filter clamps
exactly those samples. Naively stacking them deadlocks (escape proposes, CBF vetoes). The
Proposition shows the conflict has a one-parameter resolution: $\alpha$ sets *how close to the
boundary the filter tolerates*, and **any** positive value keeps $\mathcal C$ invariant. So the
coordinator can hand the escape layer more boundary room (raise $\alpha$) precisely when entrapped,
and reclaim it (lower $\alpha$) when a dynamic obstacle's TTC is imminent — all without ever
forfeiting the certificate. The gain is the unique knob that buys maneuverability *for free* in the
safety budget.

## 4. Coordinator overrides are consistent with the Proposition

The TTC override ($\mathrm{TTC}<\tau\Rightarrow\alpha=\alpha_{\text{base}}$) and prediction-trust
gate ($\max_j q_j>\bar q\Rightarrow\alpha=\alpha_{\text{base}}$) only ever *lower* $\alpha$ to a
still-positive value. By the Corollary they keep $\mathcal C$ invariant; their role is to make the
approach **more conservative** when dynamic risk is high or the prediction is untrustworthy — a
behavioral choice, not a safety prerequisite. (The $q$-inflation of $R_j$ is the complementary
robustness mechanism; see §5.4.)

## 5. Scope and honest caveats (what this proof does *not* cover)

**5.1 Hard-safe regime only ($\delta=0$), and the slack is *shared*.** The Proposition assumes the
QP is feasible without slack. The implementation uses a **single** slack $\delta$ shared by all
obstacle rows (`cbf_safety_filter.cpp:111,162`), not one per obstacle. Two consequences: (i) when the
QP must relax ($\delta>0$, giving $\dot h_j\ge-\alpha h_j-\delta$ for *every* $j$), invariance is not
certified that cycle, and the implementation flags `hard_safe=false` (`:218`); (ii) **one imminent
obstacle relaxes every barrier by the same $\delta$** — the per-obstacle certificates are coupled
through the shared slack, so a single critical obstacle momentarily loosens the margin on all
others. This is the standard slack-CBF-QP trade-off; the paper should certify safety *conditional on
$\delta=0$*, report slack-usage rate (already in the protocol), and state the shared-slack coupling
as a property of the safety story (a per-obstacle slack would decouple them at the cost of more QP
variables — a design choice worth naming).

**5.2 Look-ahead point vs. robot body.** $h_j$ certifies that $p_L$ (a point
$L$ ahead of the base) stays outside the disc of radius $R_j$. Body safety (base-centered, radius
$\rho$) follows only if the inflation absorbs the offset. With defaults $L=0.20>m=0.05$, protecting
$p_L$ does **not** strictly certify the base point. Standard look-ahead-CBF practice protects $p_L$
and argues body safety heuristically (the robot leads with $p_L$ along its motion). **RESOLVED
(2026-06-14):** claim scoped to the look-ahead point; body safety stated as approximate (mitigated
by $L\approx\rho$ + costmap critic + collision_monitor). Full-body inflation rejected — adding $L$ to
every margin needs ~0.94 m clearance > BARN's ~0.9 m, making tight scenarios infeasible and cratering
success rate.

**5.3 Continuous CBF sampled per cycle.** The code enforces
the *continuous* condition $\dot h+\alpha h\ge0$ once per control cycle. It is **not** a
discrete-time CBF ($h_{k+1}-h_k\ge-\gamma h_k$, $\gamma\in(0,1]$). Between samples
the condition is held under zero-order hold and excursions are uncontrolled; strict guarantees need
a sampled-data CBF margin (tighten $R_j$ by an $O(\dot h_{\max}\Delta t)$ term).
**RESOLVED (2026-06-14):** draft reworded to "continuous-time CBF enforced in a sampled-data QP" +
sampled-data caveat added. Code unchanged — it was always a continuous CBF; no discrete-time CBF was
introduced.

**5.4 Conformal $q$ — robustness, not a claimed guarantee (per scope decision 2026-06-13).** The
$q$-inflated radius $R_j=\dots+q_j$ gives robustness to bounded prediction error: if the true
obstacle position stays within $q_j$ of the prediction, the inflated barrier certifies the true one
by the triangle inequality. Paper 1 presents this as an **implementation detail**, not a claimed
contribution (the conformal-CBF contribution is deferred to Paper 2; see
`docs/research/2026-06_se-mppi-novelty-verification.md` §5). The draft's *formal* barrier already
excludes $q$ ($R=r+R_o+m$, §IV-E, deferring it to §VII), so the **certified** statement is $q$-free
and the code's $q$-inflation is strictly extra (more conservative) robustness — paper and proof
agree. Keep $q$ out of the certified claim.

**5.5 Multi-robot reciprocity (Paper 3 scope).** For a reciprocal neighbor the row uses
$b=-\lambda\alpha h$ (no velocity term). Joint invariance of the pair requires the budget split
$\lambda_{ij}+\lambda_{ji}=1$; this is out of Paper 1 scope and not proved here.

## 6. One-line takeaway for the paper

> *Forward invariance of the (per-obstacle, zero-slack) safe set holds for any positive bounded
> class-$\mathcal K$ gain; therefore the escape–safety coordinator's gain modulation — and its
> TTC/trust overrides — change only how closely the robot may approach the boundary, never whether
> it is collision-free.* The certificate is conditional on (i) zero QP slack and (ii) the
> look-ahead/sampled-data modeling assumptions of §5.2–5.3, which the paper must state explicitly.
