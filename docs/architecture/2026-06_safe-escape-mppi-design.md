# Safe-Escape MPPI (SE-MPPI) — 소프트웨어 아키텍처 설계

> **작성일:** 2026-06-08
> **상태:** 설계 초안 (v0.1)
> **선행 문서:** `docs/research/2026-06_safe-escape-mppi-problem-statement.md`
> **목표:** ROS2 Nav2-native 로컬 컨트롤러로, MPPI + 온라인 로컬미니마 탈출 + 동적장애물 CBF 안전필터를 결합. Gazebo/BARN/HuNavSim에서 정량 평가 가능한 구현.

---

## 1. 설계 원칙

1. **Nav2 통합 최우선** — 기존 `nav2_mppi_controller`의 검증된 `mppi::Optimizer`를 **재사용**하고, 최소 침습으로 확장. 무거운 옵티마이저를 fork하지 않는다.
2. **모듈 분리 → ablation 용이** — escape / safety / prediction을 독립 컴포넌트로 두어 켜고 끄며 평가(논문 ablation = 구현 토글).
3. **CPU 실시간** — MPPI가 CPU 50Hz로 도는 강점을 유지. CBF-QP는 저차원(2 입력)이라 CPU에서 ms급.
4. **점진적 구현** — 각 컴포넌트가 단독으로도 stock MPPI에 붙어 동작(critic은 drop-in, 필터는 standalone 노드 변형 제공).

---

## 2. 시스템 아키텍처

```
                         ┌──────────────────────────────────────────────┐
                         │           Nav2 Controller Server             │
                         │  ┌────────────────────────────────────────┐  │
   /plan (global) ──────▶│  │   SafeEscapeController                  │  │
   costmap ─────────────▶│  │   (nav2_core::Controller)              │  │
   /odom ───────────────▶│  │                                        │  │
                         │  │   ┌──────────────────────────────┐     │  │
                         │  │   │  mppi::Optimizer (재사용)     │     │  │
                         │  │   │   critics:                   │     │  │
                         │  │   │    - 기존 critics            │     │  │
                         │  │   │    - ★EscapeCritic (신규)    │◀────┼──┼── entrapment 감지+
                         │  │   └──────────────┬───────────────┘     │  │   repulsive 증강
                         │  │     u_mppi (Twist)│                     │  │
                         │  │   ┌──────────────▼───────────────┐     │  │
                         │  │   │ ★CBF Safety Filter (신규)    │◀────┼──┼── DynamicObstacleTracker
                         │  │   │  DCBF/parabolic QP            │     │  │   (CV/학습 예측)
                         │  │   │  + escape-safety 조율         │     │  │
                         │  │   └──────────────┬───────────────┘     │  │
                         │  └─────────────────│────────────────────┘  │
                         └────────────────────│───────────────────────┘
                                       u_safe  │ (TwistStamped)
                                               ▼
                                      [collision_monitor] ── 최종 안전망 (기존)
                                               ▼
                                            /cmd_vel → robot
```

**핵심 통합 결정:**
- **EscapeCritic**: `mppi::critics::CriticFunction` 서브클래스 → pluginlib로 `critics:` 리스트에 추가. **옵티마이저 fork 불필요.** stock 컨트롤러에도 그대로 붙는다.
- **SafeEscapeController**: `nav2_core::Controller` 구현. 내부에 `mppi::Optimizer`를 보유(`MPPIController`와 동일 패턴, 얇은 wrapper ~200줄), `computeVelocityCommands()`에서 옵티마이저 호출 후 **CBF-QP 필터를 출력 twist에 후처리**.
- **DynamicObstacleTracker**: 코스트맵/센서에서 장애물을 추출, 속도 추정(CV 모델)하여 미래 궤적 예측. CBF 필터와 EscapeCritic이 공유.

---

## 3. 컴포넌트 설계

### 3.1 EscapeCritic (로컬미니마 감지·탈출)

**역할:** 매 제어주기 `CriticData`로부터 entrapment를 감지하고, 감지 시에만 repulsive-potential 비용을 `data.costs`에 주입(detect-and-switch).

**인터페이스(확정):**
```cpp
class EscapeCritic : public mppi::critics::CriticFunction {
  void initialize() override;          // 파라미터 로드
  void score(CriticData & data) override;  // 감지 + 조건부 증강
};
```

**entrapment 감지 신호(CriticData에서 직접 가용):**
1. **진행 정체** — `furthest_reached_path_point`가 N주기 동안 미증가(글로벌 경로 진척 없음).
2. **전 rollout 고비용/충돌** — `trajectories_in_collision` 다수 true 또는 `costs` 최소값이 임계 이상(모든 샘플이 나쁨 = 갇힘).
3. **목표 방향 차단** — goal 방향 best rollout이 장애물 비용 벽에 막힘(`ObstaclesCritic` 거리 활용).
- 세 신호의 가중합이 임계 초과 + 일정 시간 지속 → `entrapped_ = true`.

**탈출 증강(감지 시):**
- 가장 가까운 장애물 클러스터에 대한 **repulsive-potential(APF)** 비용을 trajectory별로 계산, goal-attractive와 합성하여 우회 궤적을 저비용으로 만듦(DRPA-MPPI 차용).
- **비볼록 확장(우리 기여):** 단일 장애물 중심 repulsion 대신, 막힌 **개구부(gap) 탐지** — 코스트맵 로컬 윈도에서 자유공간 개구부를 찾아 그 방향으로 attractive subgoal을 일시 설정(U자 탈출).
- 히스테리시스로 on/off 진동 방지(탈출 후 진척 회복되면 `entrapped_=false`).

**파라미터:** `entrapment_progress_window`, `entrapment_cost_threshold`, `repulsion_weight`, `repulsion_power`, `gap_search_radius`, `hysteresis_steps`, `enabled`.

> ablation: `enabled=false`면 stock MPPI 거동. detect-and-switch vs always-on 비교 토글 제공.

### 3.2 CBF Safety Filter (동적장애물 안전)

**역할:** 옵티마이저 출력 `u_mppi = [v, ω]`를 안전집합 forward-invariance를 만족하는 최근접 `u_safe`로 사영(mechanism (b): hard QP 필터).

**차동구동 모델:** unicycle `ẋ=v cosθ, ẏ=v sinθ, θ̇=ω`.

**CBF 정식화(채택: DPCBF류 parabolic / 동적장애물):**
- 각 추적 장애물 j(위치 `p_j`, 속도 `v_j`, 반경 `r_j`)에 대해 상대거리·상대속도 기반 안전함수 `h_j(x, x_j)`. 정적은 거리 CBF, 동적은 **상대속도를 고려한 parabolic 안전집합**(collision-cone보다 덜 보수적, 밀집에서 feasible).
- **이산시간 CBF(DCBF)** 조건: `h_j(x_{k+1}) - h_j(x_k) ≥ -γ · h_j(x_k)`, `γ∈(0,1]`.
- 입력제약 `v∈[v_min,v_max], ω∈[-ω_max,ω_max]`.

**QP:**
```
min_{u}      ‖u - u_mppi‖²_W + ρ·δ²
s.t.   DCBF_j(x, u) ≥ -δ        ∀ j (slack δ≥0 으로 feasibility 보존)
       u ∈ [u_min, u_max]
```
- 2변수 QP → OSQP 또는 소규모 해석적 해. CPU ms급, 실시간.
- slack `δ`는 극한 상황 feasibility 보존(safety-critical시 ρ 큼).

**★escape-safety 조율(핵심 기여):**
- `EscapeCritic.entrapped_` 신호를 필터가 구독 → 감지 시 **class-K 여유 γ를 동적으로 증가**(γ↑ = 경계 더 가까이 허용)하여 탈출 기동을 certified-safe 범위 내에서 통과시킴.
- 단, 동적장애물 TTC(time-to-collision)가 짧으면 γ 변조를 무효화(안전 우선) — 안전과 탈출의 명시적 우선순위 규칙.

**파라미터:** `cbf_gamma_base`, `cbf_gamma_escape`, `slack_weight_rho`, `obstacle_radius_margin`, `ttc_override_threshold`, `solver`(osqp/analytic), `enabled`.

> ablation: `enabled=false`면 critic-only MPPI. 정적 CBF vs DCBF/parabolic 토글.

### 3.3 DynamicObstacleTracker

**역할:** 코스트맵/`/scan`/장애물 토픽 → 추적 장애물 리스트(위치·속도·반경).
- **v0:** 로컬 코스트맵 lethal 셀 클러스터링(연결성분) + 프레임간 매칭으로 **CV(등속) 속도 추정**.
- **v1(옵션):** 외부 detection/tracking(예: 사람 토픽, learned predictor) 입력 수용 인터페이스.
- 출력: `std::vector<TrackedObstacle{ Eigen::Vector2d p, v; double r; }>` + 호라이즌 예측 `predict(t)`.

> ablation: 예측 모드 none(현재위치 고정) / CV / 외부학습 — C3 동적장애물 ablation.

---

## 4. 패키지 레이아웃

```
src/
  nav2_se_controller/                 # 메인 컨트롤러 + critic
    include/nav2_se_controller/
      safe_escape_controller.hpp
      escape_critic.hpp
      cbf_safety_filter.hpp
      dynamic_obstacle_tracker.hpp
    src/
      safe_escape_controller.cpp      # nav2_core::Controller
      escape_critic.cpp               # mppi::critics::CriticFunction
      cbf_safety_filter.cpp           # DCBF-QP
      dynamic_obstacle_tracker.cpp
    se_controller_plugin.xml          # nav2_core::Controller export
    escape_critics.xml                # mppi::critics::CriticFunction export
    CMakeLists.txt  package.xml
  se_mppi_bringup/                     # launch, params, maps
    launch/  params/nav2_se_params.yaml  worlds/
experiments/
  barn/         # the_barn_challenge 연동(BARN/DynaBARN) + 리포트 스크립트
  hunav/        # HuNavSim(ROS2) 시나리오 + 메트릭 수집
  baselines/    # DWB/RPP/MPPI(stock)/TEB 비교 설정
  analysis/     # 결과 집계·플롯
```

**빌드 의존성:** ROS2 **Kilted**(MPPI Eigen 재구현·ARM, Route Server 기준), `nav2_mppi_controller`(Optimizer/critic 베이스), `nav2_core`, `nav2_costmap_2d`, `pluginlib`, `Eigen3`, QP 솔버(`osqp` + `osqp-eigen`). 시뮬: **Gazebo Harmonic/Ionic**.

---

## 5. 평가 계획 (3-tier)

| Tier | 환경 | 핵심 메트릭 |
|---|---|---|
| **정적 혼잡** | BARN(world 0–299), 50 held-out×10 | **BARN score** `1_success·OT/clip(AT,2·OT,8·OT)`, success/collision/timeout, time |
| **동적 비소셜** | DynaBARN(world 300–359) | success, collision, time, **min obstacle dist** |
| **소셜/군중** | **HuNavSim**(ROS2, Gazebo) | success, collision, time-to-goal, **min/avg dist to humans, personal-space 침해, social work** |
| **컨트롤러 bake-off** | 동일 world | vs **DWB/RPP/MPPI(stock)/TEB**: success, path length, **smoothness/jerk**, **control Hz·compute time**, min dist |

**Ablation(컴포넌트 = 토글):**
1. 예측: none → CV → learned/SFM (C3)
2. escape: off → always-on → **detect-and-switch(SE)** (C2 탈출)
3. safety: off → static CBF → **DCBF/parabolic(SE)** (C2 안전)
4. 조율: 독립 escape+CBF → **escape-safety 조율(SE)** ← 핵심 기여 검증

> 주의(리서치): BARN 레퍼런스 파이프라인은 ROS1 canonical, ROS2 브랜치 미검증 → **포팅/브리지 비용 예산화**. HuNavSim은 ROS2-native라 Nav2 통합 최적.

---

## 6. 구현 로드맵 (마일스톤)

- **M0 — 스캐폴드·재현환경:** `nav2_se_controller` 패키지 골격 + `se_mppi_bringup`로 Gazebo+Nav2+stock MPPI 주행 확인. BARN ROS2 브리지 PoC.
- **M1 — EscapeCritic:** 감지 신호 3종 + repulsive 증강 + gap 탐색. stock MPPI에 drop-in, BARN 정적에서 stock 대비 success↑ 확인. (C-탈출 단독)
- **M2 — CBF Safety Filter:** DynamicObstacleTracker(CV) + DCBF-QP(OSQP). SafeEscapeController wrapper로 출력 후처리. DynaBARN에서 충돌↓·forward-invariance 검증. (C-안전 단독)
- **M3 — escape-safety 조율:** γ 변조 + TTC override. 통합 컨트롤러로 정적/동적 동시 개선. (C2 핵심)
- **M4 — 동적 CBF 고도화:** parabolic/DPCBF 채택, 예측 ablation(CV vs learned). HuNavSim 소셜 메트릭. (C3)
- **M5 — 벤치마크·재현 패키지:** 3-tier 전체 + baseline bake-off + ablation 표·플롯. 공개 코드·결과. (C1/C4)
- **M6 — 논문화:** 선행 검증 TODO 완료, 결과 정리, 투고(타깃: ICRA/IROS/RA-L).

---

## 7. 리스크와 대응

| 리스크 | 대응 |
|---|---|
| BARN ROS1→ROS2 포팅 비용 | M0에서 조기 PoC; 안 되면 HuNavSim+자체 정적 world로 대체, BARN은 정성 비교 |
| CBF-QP 실시간성 | 2변수 QP라 경량; OSQP warm-start, 불가시 해석해 fallback |
| escape-safety 조율 불안정(γ 변조 진동) | 히스테리시스 + TTC 하드 우선순위 + 단조 스케줄 |
| novelty 중복(누가 먼저 Nav2 CBF-MPPI 냄) | M0에서 GitHub/arXiv 최종 검증; 차별점을 조율+동적+재현으로 다층화 |
| DRPA-MPPI 코드 부재로 baseline 재현 난이 | always-on repulsive를 자체 구현해 baseline화, 차이를 detect-switch로 |

---

## 8. 다음 액션

1. **M0 착수** — `nav2_se_controller` 패키지 스캐폴드 + bringup으로 stock MPPI 주행 베이스라인 확보.
2. 병렬로 **선행 검증 TODO**(GitHub topic 검색, DRPA/DPCBF PDF 확인) 수행.
3. M1 EscapeCritic 인터페이스 스텁부터 코드 작성.
