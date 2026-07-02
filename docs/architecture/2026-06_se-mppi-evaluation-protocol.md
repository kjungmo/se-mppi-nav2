# SE-MPPI 평가 프로토콜 설계

> **작성일:** 2026-06-08
> **목적:** SE-MPPI(Safe-Escape MPPI)의 기여를 정량 검증하기 위한 평가 설계 —
> 메트릭, baseline, ablation 매트릭스, 벤치마크 tier, 통계 분석, 재현 절차.
> **선행:** `2026-06_safe-escape-mppi-problem-statement.md`, `2026-06_safe-escape-mppi-design.md`
> **상태:** 설계 (실행은 동작하는 시뮬 필요 — §7 인프라 참조).

---

## 1. 검증할 주장(claims)과 매핑

| Claim | 내용 | 검증 메트릭 | Ablation 축 |
|---|---|---|---|
| **C1** | Nav2-native 배포 가능 플러그인 | 런타임 로드·활성화(완료) | — |
| **C2-escape** | 온라인 감지·탈출이 always-on보다 우수 | 로컬미니마 성공률↑, 진동·시간↓ | gating on/off |
| **C2-safety** | DCBF 안전필터가 충돌을 형식적으로 줄임 | 충돌률↓, min-dist↑, h≥0 위반횟수 | CBF on/off |
| **C2-coord** | escape↔safety **조율**이 단순 합보다 우수 | 좁고 동적인 곳에서 성공률↑+충돌률↓ 동시 | α 변조 on/off |
| **C3-dynamic** | 동적장애물 CV 예측이 안전마진 개선 | 동적 충돌률↓, min-dist↑ | 예측 none/CV |

핵심 가설(H): **C2-coord** — escape와 safety를 독립으로 켠 것(escape+CBF)보다
**조율(α 변조)** 이 *좁고 동적인 환경에서 성공률과 충돌률을 동시에* 개선한다.

---

## 2. Baseline

**Nav2 기본 컨트롤러 (동일 스택, FollowPath 교체):**
- **Stock MPPI** (`nav2_mppi_controller::MPPIController`) — 1차 baseline (`se_enabled=false`와 동일)
- **DWB**, **Regulated Pure Pursuit (RPP)**, **TEB** — 표준 비교군

**연구 baseline (재구현/문헌):**
- **Always-on repulsion** (DRPA-MPPI류) — 우리 cost-proxy를 gating 없이 상시 적용(`use_gap_search=false`, gating 제거판)
- **CBF-only** (Shield-MPPI류) — escape 없이 CBF 필터만(`EscapeCritic` 미등록 + CBF on)
- **BARN 챌린지 상위팀** 공개 결과 (정성 대조)

> 모든 컨트롤러는 **동일 planner(Smac Hybrid-A*)·costmap·BT** 위에서, 동일 로봇/맵/목표로 평가.

---

## 3. Ablation 매트릭스 (구현된 토글에 직접 대응)

| 구성 | `EscapeCritic` | gating | `use_apf` | `use_gap_search` | CBF | α 변조 | 예측 |
|---|---|---|---|---|---|---|---|
| **A. Stock MPPI** | ✗ | — | — | — | ✗ | ✗ | — |
| **B. +Escape(always-on)** | ✓ | ✗ | ✓ | ✓ | ✗ | ✗ | — |
| **C. +Escape(detect-switch)** | ✓ | ✓ | ✓ | ✓ | ✗ | ✗ | — |
| **D. +CBF only** | ✗ | — | — | — | ✓ | ✗ | CV |
| **E. Escape+CBF (독립)** | ✓ | ✓ | ✓ | ✓ | ✓ | ✗(α=base) | CV |
| **F. SE-MPPI (조율, full)** | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | CV |
| **F⁻. SE − gap** | ✓ | ✓ | ✓ | ✗ | ✓ | ✓ | CV |
| **F″. SE, APF→cost-proxy** | ✓ | ✓ | ✗ | ✓ | ✓ | ✓ | CV |
| **F‴. SE − 예측(static)** | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | none |

**토글 매핑 (실제 파라미터):**
- gating off = `EscapeCritic`을 비-detect 모드로(상시 비용) — 평가용 빌드 플래그/파라미터
- CBF off = `se_enabled=false`(전체) 또는 obstacle 미공급
- α 변조 off = `se_alpha_escape == se_alpha_base` (조율 무력화, 독립 escape+CBF)
- 예측 none = `se_obstacle_max_speed=0`(CV 무력화, 정적 취급)
- APF↔proxy = `use_apf`
- gap on/off = `use_gap_search`

**핵심 비교:** **E vs F** (독립 vs 조율) — H 검증. **B vs C** (always-on vs detect-switch).
**A vs C** (escape 효과). **A vs D** (CBF 효과). **F vs F‴** (예측 효과).

---

## 4. 메트릭

**탐색·효율 (정적/동적 공통):**
- **Success rate** (충돌·타임아웃 없이 목표 도달 비율) — 1차 지표
- **Collision rate** (정적/동적 분리 집계)
- **Timeout rate**, **Time-to-goal**, **Path length**
- **BARN navigation score** `1_success · OT / clip(AT, 2·OT, 8·OT)` (정적 tier)
- **로컬미니마 탈출 성공률** (U-trap 시나리오 한정), **진동/제자리회전 시간 비율**

**안전:**
- **Min distance to obstacle** (전 구간 최소), **CBF 위반 횟수**(h<0 사건), **slack 사용 비율**(hard_safe=false 빈도)
- 동적: **min distance to dynamic obstacle**, **TTC 분포**

**제어 품질·실시간성:**
- **Smoothness/jerk**, **control frequency**, **per-cycle compute time**(MPPI + CBF-QP), QP 반복수

**소셜(HuNavSim tier):**
- **min/avg distance to humans**, **personal-space 침해 횟수/시간**, **social work**

---

## 5. 벤치마크 tier (3단)

| Tier | 환경 | 시나리오 | 주 메트릭 |
|---|---|---|---|
| **T1 정적 혼잡** | BARN(0–299), 50 held-out×10 | 좁은 통로·U-trap | BARN score, success, 탈출성공률 |
| **T2 동적 비소셜** | DynaBARN(300–359) | 이동 장애물 | success, 동적충돌률, min-dist, TTC |
| **T3 소셜/군중** | HuNavSim(ROS2) | 보행자(SFM) | success, social work, proxemics |

각 구성 × tier에 대해 **N≥20 시드 반복**(시나리오당), 평균·표준편차·신뢰구간 보고.

---

## 6. 통계 분석

- **주 가설(H, E vs F):** 성공률은 **이항** → McNemar/부트스트랩 비율차 검정;
  충돌률·시간은 **Mann–Whitney U**(비정규). 효과크기(Cliff's δ) 병기.
- **다중비교 보정**(Holm–Bonferroni): 9개 구성 × 3 tier.
- **시드 고정·공개**로 재현. 각 run 로그(궤적, cmd_vel, α, slack, min-dist) 저장 → `experiments/analysis`.
- **민감도 분석:** `progress_stall_window`, `alpha_escape`, `apf_influence_dist`,
  `cbf_lookahead`, `ttc_override_threshold` 스윕.

---

## 7. 실행 인프라 (정직한 제약)

- **현 상태:** 컨트롤러는 단위테스트(164 checks) + 라이브 Nav2 로드/유효 cmd_vel 검증 완료.
- **시뮬 완주 미해결:** RoboStack **loopback** 빌드가 odom/구동 플러밍 결함으로
  로봇을 구동 못 함(stock 데모도 동일 — env 문제). → **정량 평가는 동작하는 시뮬 필요.**
- **권장 경로:** **Gazebo 물리 시뮬(headless+xvfb)** 또는 정상 ROS2 워크스테이션.
  BARN/DynaBARN는 ROS1 canonical → ROS2 브리지 또는 자체 BARN-like 절차 구축.
  HuNavSim은 ROS2-native라 우선 통합 후보.
- **재현 산출물:** `experiments/{barn,dynabarn,hunav,baselines,analysis}` + 구성별 params yaml
  + run·집계 스크립트. (스모크 자산 `experiments/sim/`은 이미 존재.)

---

## 8. 예상 결과 형태 (논문 표 골격)

- **표 1 (T1 정적):** A–F 구성 × {success, BARN score, time, 탈출성공률} — *C/F가 A 대비 탈출↑*
- **표 2 (T2 동적):** 구성 × {success, 동적충돌률, min-dist, TTC} — *F/F‴ 비교로 예측 효과*
- **표 3 (T3 소셜):** 구성 × {success, social work, proxemics} — *F가 사회적 안전 균형*
- **핵심 표 (E vs F):** 좁고 동적 환경에서 **성공률↑ + 충돌률↓ 동시** → H 입증
- **그림:** U-trap 탈출 궤적(escape on/off), α·slack 시계열(조율 동작), compute-time 분포

> 주의: 위는 **설계상 기대**이며 실측 아님. 실행 후 실제 수치로 대체.
