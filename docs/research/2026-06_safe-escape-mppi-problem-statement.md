# Safe-Escape MPPI: 문제 정의 · 선행연구 · 기여 위치

> **작성일:** 2026-06-08
> **연구 코드네임:** Safe-Escape MPPI (**SE-MPPI**)
> **한 줄 요약:** ROS2 Nav2-native 로컬 컨트롤러로, MPPI 위에 (1) 온라인 로컬미니마 **감지·탈출**과 (2) 동적장애물에 대한 형식적 **CBF 안전필터**를 결합하여, 좁고 혼잡한 환경에서 *빠르면서도 충돌 없이 갇히지 않는* 주행을 보장한다.
> **근거:** `docs/research/2026-06_mobile-navigation-sota-survey.md`의 빈틈 #2를 4개 타깃 리서치로 심화한 결과.

---

## 1. 문제 정의 (Problem Statement)

ROS2 Nav2의 **MPPI 컨트롤러**(`nav2_mppi_controller`)는 현재 가장 성능 좋은 기본급 로컬 컨트롤러다(차동/omni/Ackermann, CPU 50+Hz). 그러나 두 가지 구조적 약점이 있다:

1. **로컬미니마 함정** — 제한된 예측 호라이즌(기본 56 step × 0.05s ≈ 2.8s)과 Gaussian 샘플링 탓에, U자형/비볼록 장애물·대칭 장애물장·좁은 갭에서 갇히거나 제자리 진동한다. Nav2가 가진 완화책은 `PreferForwardCritic`·`TwirlingCritic` 같은 **항상 켜진 비용 정형(cost-shaping) 휴리스틱**뿐이며, entrapment를 *감지*하거나 형식적 탈출을 보장하지 않는다.
2. **형식적 안전 보장 부재** — 충돌회피는 `ObstaclesCritic`/`CostCritic` 같은 **soft 비용항**으로만 이뤄진다. forward-invariance(안전집합 불변) 보장이 없고, 무엇보다 **동적장애물의 미래 궤적을 예측하지 않는다**. Nav2 코스트맵은 현재 스냅샷이고, `collision_monitor`는 현재 센서·현재 로봇속도 기반의 순수 반응형(Stop/Slowdown/Approach)이다.

이 두 약점은 **혼잡·동적 환경에서 동시에** 문제가 된다: 안전을 위해 보수적으로 굴면 갇히고(로컬미니마), 빠르게 빠져나오려 하면 충돌 위험이 커진다. 두 문제를 **함께** 푸는 배포가능한 컨트롤러가 필요하다.

---

## 2. 선행연구 정밀 분석 (3개 축)

### 축 A — Nav2 MPPI 내부 (통합 표면)

- `mppi::Optimizer`가 `batch_size`개 rollout을 샘플→critic 비용 합산→정보이론적 softmax(`exp(-1/temperature · cost)`)로 가중평균하여 control sequence 갱신, 첫 원소를 `TwistStamped`로 출력.
- **Critic 플러그인 인터페이스**(`mppi::critics::CriticFunction`): `initialize()` + `score(CriticData&)` 구현, `data.costs`(길이 `batch_size`의 `Eigen::ArrayXf`)에 `(weight · raw).pow(power)` 누적. `CriticData`는 `trajectories`, `path`, `goal`, `trajectories_in_collision`, `furthest_reached_path_point` 등을 제공 → **entrapment 감지에 필요한 정보가 이미 들어있다.**
- 충돌체크: `FootprintCollisionChecker` + 코스트맵. `ObstaclesCritic`는 inflation 거리 역산으로 `distanceToObstacle()` 제공.
- 컨트롤러는 `nav2_core::Controller`를 구현, pluginlib로 Controller Server가 로드.
- *함의:* **escape는 커스텀 critic으로 fork 없이 삽입 가능**하고, **CBF 안전필터는 컨트롤러 출력단에 후처리로 삽입**하는 것이 가장 깔끔한 통합 지점.

### 축 B — CBF × MPPI 안전 (이미 된 것 vs 빈틈)

융합 방식 4가지 모두 연구코드로 존재:
- **(a) soft 비용/critic**: Shield-MPPI(arXiv:2302.11719), BR-MPPI(2506.07325)
- **(b) 출력에 hard QP 필터**: CBFKit(2404.07158), Shield-MPPI의 shield 단계
- **(c) unsafe 샘플 거부**: reach-avoid SCBF-MPPI(2407.13693), DualGuard 1단계
- **(d) projection/shielding(전 rollout 증명가능 안전)**: GS-MPPI(2410.02154, 비홀로노믹), DualGuard(2502.01924, HJ-reachability)

동적장애물용 CBF의 최신: **DPCBF "Beyond Collision Cones"**(arXiv:2510.01402, ICRA 2026) — parabolic 안전집합으로 collision-cone/velocity-obstacle CBF보다 훨씬 덜 보수적, 밀집 동적장애물(최대 100개)에서 QP feasibility 유지. **단, QP-only이며 아직 MPPI 안에 들어가지 않음.**

> **빈틈(축 B):** ① **Nav2-native `nav2_core::Controller` 플러그인으로 배포된 CBF-MPPI는 전무**(CBFKit조차 standalone 노드 생성, Nav2 아님). ② **동적장애물용 DCBF/parabolic CBF를 MPPI 안에서 차동구동에 적용**한 배포 스택은 미점유. ③ DPCBF의 feasibility 이점을 MPPI의 장기 호라이즌 성능과 결합한 사례 없음.

### 축 C — 로컬미니마 탈출 (이미 된 것 vs 빈틈)

대부분 **always-on** 증강: log-MPPI(다른 샘플링 분포), Tsallis-MPPI, SVG-MPPI(2309.11040, Stein mode-seeking), Biased-MPPI(2401.09241), FlowMPPI(학습 proposal). 유일한 **detect-and-switch**는:
- **DRPA-MPPI**(arXiv:2503.20134, Fuke/Endo/Honda/Ishigami, Keio, **IEEE CASE 2025**): 예측 궤적에서 entrapment를 **감지**→repulsive-potential(APF) 비용항으로 전환(detect-and-switch). 한계: **CBF/형식적 안전 없음, 정적·반응형(동적장애물 모델 없음), 공개코드 없음, Nav2 미통합**. (비볼록 결과도 주장하므로 "convex-only" 표현은 쓰지 말 것 — 검증 §2 참조)

> **빈틈(축 C):** ① **로컬미니마 탈출 + 형식적 CBF 안전을 한 컨트롤러에 결합한 사례 없음** — 두 연구라인이 분리됨(탈출은 안전증명 없음, CBF는 entrapment 악화 가능: 경계에서 탐색샘플을 깎음). 가장 근접한 BR-MPPI·DBaS-Log-MPPI도 *명시적 entrapment 감지 + 전용 탈출 행동*은 없음. ② 탈출 라인 중 Nav2-native 배포 전무.

---

## 3. 기여 위치 (Novelty Positioning)

단일 선행연구가 점유하지 못한 **3축 교집합**에 위치한다:

| 축 | 기존 SOTA | SE-MPPI 기여 |
|---|---|---|
| **탈출** | always-on 증강 / DRPA의 detect-switch(정적·반응형·CBF없음·코드없음·비-Nav2) | 온라인 entrapment 감지 + 조건부 repulsive 탈출, **CBF 안전 결합·Nav2 배포** |
| **안전** | CBF-MPPI 연구코드(정적·연속시간 CBF 위주) | **DCBF/parabolic 동적장애물 CBF**를 MPPI 출력에 QP 필터로, 탐색을 certified-safe로 |
| **배포** | 전부 연구 repo(JAX/CUDA/MATLAB) / Nav2는 휴리스틱 critic | **Nav2-native 플러그인**(critic + controller) + 재현가능 ROS2 벤치 |

**핵심 통찰(논문 thesis):** escape와 safety는 *상충*한다 — repulsive 탈출은 안전경계로 로봇을 밀고, CBF 필터는 탈출에 필요한 탐색샘플을 깎는다. SE-MPPI는 이 둘을 **조율(coordinate)**한다: entrapment 감지 시 CBF의 class-K 여유를 동적으로 풀어 탈출 기동을 certified-safe 범위 내에서 허용한다. *이 조율 메커니즘 자체가 핵심 기여*이며, 두 기법의 단순 합이 아니다.

### 기여 주장 (Claims) — 보수적으로
1. **C1 (시스템):** CBF 안전필터 + 온라인 로컬미니마 탈출을 결합한 최초의 **Nav2-native 로컬 컨트롤러 플러그인**.
2. **C2 (알고리즘):** escape-safety **조율** 메커니즘 — entrapment 감지에 연동해 DCBF class-K 여유를 변조하여 *갇히지 않으면서 forward-invariance를 유지*.
3. **C3 (동적장애물):** DPCBF류 parabolic/DCBF를 차동구동 MPPI에 통합 + CV/학습 예측 ablation.
4. **C4 (재현성):** BARN/DynaBARN + HuNavSim(ROS2) 위 공개 벤치마크·코드.

> **포지셔닝 주의(리서치 권고):** "CBF와 MPPI를 융합"하는 일반적 novelty는 주장하지 말 것(이미 됨). 빌드 위에 명시: **(i) Nav2-native 배포, (ii) escape-safety 조율, (iii) 동적 DCBF + 차동구동 + 재현 벤치.** Shield-MPPI·GS-MPPI를 알고리즘 선행으로, DPCBF를 채택한 동적 CBF로, DRPA-MPPI를 탈출 baseline으로 인용·차별화.

### 검증 TODO (논문 전 필수)
- [x] **GitHub/web novelty 검증 완료(2026-06)** — Nav2-native CBF-MPPI 플러그인 0건,
      escape+CBF 결합 컨트롤러 0건. 4-way 교집합 유지. → `docs/research/2026-06_se-mppi-novelty-verification.md`
- [ ] 카메라레디 전: DRPA-MPPI PDF로 venue(CASE 2025 추정)·비볼록 주장·코드유무 직접 확인(WebFetch 403로 스니펫 기반).
- [ ] DPCBF(2510.01402) 정식 채택 전 라이선스·수식 PDF 확인.
- [ ] 프레이밍: abstract에서 "CBF+MPPI"(기존) 대신 **Nav2 배포 + α 변조 조율**을 차별점으로(검증 권고).

---

## 4. 핵심 참고문헌

**Nav2 MPPI:** ros-navigation/navigation2 `nav2_mppi_controller` (소스/README, main). docs.nav2.org/configuration/packages/configuring-mppic.html
**CBF×MPPI:** Shield-MPPI arXiv:2302.11719 · GS-MPPI 2410.02154 · DualGuard 2502.01924 · BR-MPPI 2506.07325 · CBFKit 2404.07158 · reach-avoid SCBF-MPPI 2407.13693 · DBaS-Log-MPPI 2504.06437
**동적 CBF:** DPCBF "Beyond Collision Cones" arXiv:2510.01402 (ICRA 2026) · "No Minima No Collisions" 2502.14238 (CBF+Modulation, 비-MPPI)
**로컬미니마 탈출:** DRPA-MPPI 2503.20134 · RPA-MPPI 2410.11379 · log-MPPI 2203.16599 · Biased-MPPI 2401.09241 · SVG-MPPI 2309.11040 (code: github.com/kohonda/proj-svg_mppi) · Tsallis-MPPI 2104.00241 · SMPPI 2112.09988
**예측+MPC(crowd):** MPC+learned pred 2508.07079 · 예측 불확실성 2504.19193 · conformal 2502.06221 · 계층 MPC 2506.09859
**평가:** BARN dataset 2008.13315 · BARN ICRA2024 2407.01862 · DynaBARN (cs.gmu.edu/~xiao/papers/dynabarn.pdf) · HuNavSim 2305.01303 (github.com/robotics-upo/hunav_sim) · Arena 4.0 2409.12471 · Nav2 controller 벤치(Figshare 2026)

> 신뢰도: arXiv 자동 fetch가 403이라 일부 저자·수치는 2차 출처 기반(논문화 전 PDF 재확인 필요). 벤치 수치는 자체보고.
