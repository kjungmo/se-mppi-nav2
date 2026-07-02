# L2 학습기반 동적장애물 예측 — 문제정의·선행연구 (Phase C)

> **작성일:** 2026-06-10 · **상태:** 문제정의 초안 (Phase C 착수 문서)
> **위치:** 전체 아키텍처 L2(인지) — `docs/architecture/2026-06_full-stack-platform-architecture.md` §5의 1순위 기여 후보.
> **목표:** SE-MPPI(L6/L7)의 CBF·TTC 입력인 동적장애물 추정을 **학습 예측 + 보정된 불확실성**으로 고도화. 소유 층 강화 + 신규 논문.
> **인용 신뢰도 표기:** 본 문서의 외부 연구 요약은 **(검색요약 기반·원문 미확인)** — 설계 확정 전 원문 확인 필수.

---

## 1. 동기 — 현재 L2의 한계 (라이브 런 실측 근거)

현재 SE-MPPI의 동적장애물 파이프라인(`dynamic_obstacle_tracker`)은:
**costmap LETHAL 셀 클러스터링 → 최근접 연관(gate 0.6m) → 상수속도(CV) 추정 → CBF/TTC 입력.**

실 Gazebo 런(2026-06, 핸드오프 문서 §3·§4)에서 드러난 구조적 한계:

| # | 한계 | 실측 증거 / 귀결 |
|---|---|---|
| H1 | **정적/동적 구분 불가** — LETHAL이면 다 클러스터 | 벽이 거대 원형 "장애물"로 들어가 CBF 영구 freeze (커밋 `21d740a`에서 속도·반경 게이트로 응급 수정). 게이트는 휴리스틱일 뿐, 천천히 움직이는 사람(<0.1m/s)을 놓치고 association 실패 시 동적 장애물도 정적으로 오인. |
| H2 | **CV 예측의 근시안** — 등속 직선 외삽 | 회전·가감속하는 보행자에 대해 CBF의 ḣ 항과 TTC가 틀림 → 과소(위험) 또는 과대(과보수) 추정. |
| H3 | **불확실성 부재** — 점추정 속도만 전달 | CBF 유효반경이 고정 margin(0.05m). 예측 오차가 정량화되지 않아 안전 주장(∀α>0 forward-invariance)이 *완벽한 예측* 가정에 의존 — 논문의 정직성 약점. |
| H4 | **클러스터 취약성** — 분리/병합·가림 | 군중에서 클러스터가 합쳐지면 속도 추정이 유령값(이미 association 단일소비로 일부 방어, 근본 해결 아님). |

→ **L2를 학습 예측으로 교체하면 SE-MPPI의 안전 주장과 성능이 동시에 강해진다.** 이것이 Phase C를 L2로 정한 이유.

---

## 2. 선행연구 미니서베이 (2024–2026)

*모든 항목 (검색요약 기반·원문 미확인) — 표기된 arXiv ID로 원문 확인 후 설계 확정.*

**(a) 학습 예측 + MPC/제어 통합**
- **MPC + Social-Implicit 예측** (arXiv:2508.07079): 딥 보행자 예측기를 MPC에 통합, 실로봇에서 저밀도 기준 예측오차 최대 −76%·혼잡 환경 안전성 향상 주장. → *예측→제어 통합의 실증 사례. 단 MPPI/Nav2 아님.*
- **사회적 존(speed-dependent ellipse)을 CBF로 강제하는 하이브리드 MPC** (Frontiers 리뷰 2025에서 소개). → *학습된 사회 규범을 CBF 제약으로 변환하는 패턴.*

**(b) 불확실성 보정(calibration) — 핵심 도구**
- **SoNIC**: Adaptive Conformal Inference(ACI)로 보행자 예측 불확실성을 정량화해 안전 제약 도출. → *분포무가정(distribution-free) 보장 — 우리 CBF 반경 보정에 직접 이식 가능.*
- **Egocentric Conformal Prediction** (arXiv:2504.00447): 안전-임계 예측오차에만 반응하는 자기중심 score + adaptive CP → 과보수 없이 동적 장애물 대응. → *CP를 내비에 특화한 최신형.*
- **Calibrated Gaussian predictors** (arXiv:2603.10407): 가우시안 예측기의 불확실성 보정 자체를 다룸.
- **UA-PCBF** (arXiv:2508.20812): 확률적 인체동작 예측 + CBF의 형식 보장 융합(HRI 도메인). → *예측+CBF 융합의 직접 선행. 모바일 내비/Nav2 아님.*
- **CCVP-MPC-CBF** (2025, UAV): 상대 위치·속도의 확률 제약 + 충돌확률 임계 + CBF.

**(c) 표현: per-agent 궤적 vs 점유격자 흐름**
- **SOGM 자기지도 예측** (arXiv:2208.12602, arXiv:2108.10585): 로봇이 스스로 주행하며 라벨 없이 시공간 점유격자 예측 학습. → *어노테이션 비용 0 — 우리처럼 LiDAR-only 셋업에 적합.*
- **RNN 동적 점유격자** (arXiv:2011.08659): 셀별 점유+속도 추정(free/static/dynamic/unknown 분류 포함). → *H1(정적/동적 구분)의 원리적 해법.*
- **LV-DOT** (arXiv:2502.20607): LiDAR-visual 동적장애물 검출·추적.

**(d) 우리 교집합에 가장 가까운 이웃 (novelty 위협 — 원문 정독 필수)**
- **"Proactive Local-Minima-Free Navigation: Blending Motion Prediction with Safe Control"** (arXiv:2601.10233): 동작 예측과 안전 제어를 결합해 로컬미니마 회피를 *선제적으로* 다룬다고 보임. → **우리 (escape + CBF + 예측) 교집합과 가장 인접. 차별점 후보: Nav2-native 배포, detect-and-switch escape(상시 아님), conformal 보정 CBF, MPPI 통합.** 원문 확인이 Phase C 첫 작업.
- **CN-CBF** (arXiv:2603.06921): 동적 환경용 복합 신경 CBF.

**갭(잠정):** *(i)* 학습 예측 + **보정된(conformal) 불확실성** + **DCBF 안전필터** + **escape 조율**을 **Nav2-native MPPI 컨트롤러**에 통합한 단일 시스템은 미발견. *(ii)* 특히 "예측 불확실성을 escape-safety 조율 변수(α·margin)에 연결"한 사례 미발견. → SE-MPPI 논문의 α-변조 novelty를 자연 확장하는 위치.

---

## 3. 제안 — SE-Predict (가칭)

### 3.1 설계 원칙
1. **인터페이스 보존**: `TrackedObstacle{position, velocity, radius}`를 확장(`+ covariance 또는 conformal bound, + horizon 예측 poses`)하되, CBF/coordinator의 기존 소비 코드는 점진 마이그레이션. L2 교체가 L6/L7을 깨지 않게.
2. **LiDAR-only 우선**: 우리 플랫폼 기본 센서. SOGM류 자기지도(라벨 0) 우선, 카메라 융합은 후순위.
3. **불확실성은 형식 보장으로**: ACI/CP로 분포무가정 오차 한계 `q_t`를 얻어 CBF 유효반경을 `r_eff(t) = r₀ + q_t`로 시변 확장 → **bounded prediction error 하에서 forward-invariance 유지**라는 명제로 논문화 (SE-MPPI의 ∀α>0 명제의 자연 확장).
4. **조율 연결(핵심 novelty)**: entrapment 시 α를 올리는 기존 조율에 더해, **불확실성이 클 때 margin↑·escape 공격성↓**의 2변수 조율 — "예측을 믿을 수 있을 때만 과감히 탈출".

### 3.2 파이프라인 (목표형)

```
LiDAR scans (과거 T프레임)
  → [N1] 셀별 동적성 분류 + 속도장 (RNN/CNN 점유격자: free/static/dynamic)
  → [N2] per-agent 추출 + 단기 궤적 예측 (0.5–3s, 멀티모달 가능)
  → [N3] Adaptive Conformal 보정 → q_t (오차 상계, online 적응)
  → TrackedObstaclePred{poses(t), velocity, radius, q_t}
  → SE-MPPI: CBF(h with r_eff=r₀+q_t, 예측 ḣ) · TTC(예측 기반) · 조율(α, margin)
```

### 3.3 마일스톤

| M | 내용 | 산출물 |
|---|---|---|
| N0 | **선행 원문 검증** — §2(d) 2601.10233 정독 + novelty 매트릭스 갱신 | novelty 검증 문서 v2 |
| N1 | 정적/동적 분류 + 속도장 (학습 또는 고전 DOGM) — H1 근본 해결 | tracker v2 + 단위테스트 |
| N2 | 단기 궤적 예측기 (CV 베이스라인 대비 ADE/FDE) — sim 데이터 자기지도 수집 | 예측 노드 + 평가 |
| N3 | ACI 보정 + CBF `r_eff(t)` 통합 + forward-invariance 명제 | SE-MPPI 통합 + 증명 스케치 |
| N4 | 평가: DynaBARN/HuNavSim에서 SE-MPPI(CV) vs SE-MPPI(SE-Predict) A/B | 논문 §실험 |

### 3.4 리스크
- **연산 예산**: 로컬 머신 실측상 제어루프 여유가 빠듯(핸드오프 §3 `52bf1c8`). 예측은 별도 노드(비동기, 5–10Hz)로 분리하고 컨트롤러는 최신 예측을 소비 — 제어루프에 학습 inference를 넣지 않는다.
- **sim 데이터 편향**: HuNavSim 보행자 모델로 학습 → 같은 모델로 평가하는 순환 위험. 교차 시나리오(다른 보행자 파라미터) 평가 + CP가 모델 불일치를 흡수함을 명시.
- **novelty**: §2(d) 원문 확인 전까지 모든 차별점 주장은 잠정.

---

## 4. SE-MPPI(Phase A)와의 관계

Phase A(M6 평가·논문)는 **CV 트래커 그대로** 완결한다 — SE-Predict는 *별도 논문*이고, Phase A 논문의 한계(§Limitations: simple CV tracker)가 곧 Phase C의 동기 문단이 된다. 두 작업은 독립 진행 가능하되, N3 통합 시점에 SE-MPPI 코드(`cbf_safety_filter`의 `eff_r`, `coordinator`의 TTC)에 시변 반경 훅만 추가하면 된다.

---

## 5. 출처 (검색요약 기반 — 원문 미확인, N0에서 검증)

- arXiv:2508.07079 — MPC for Crowd Navigation via Learning-Based Trajectory Prediction
- arXiv:2504.00447 — Egocentric Conformal Prediction for Safe Navigation
- arXiv:2511.18170 — Time-aware Motion Planning with Conformal Prediction
- arXiv:2504.00352 — Koopman + Conformal Prediction Safe Navigation
- arXiv:2508.20812 — Uncertainty-Aware Predictive CBF (HRI)
- arXiv:2601.10233 — **Proactive Local-Minima-Free Navigation (최인접 — 필독)**
- arXiv:2603.06921 — CN-CBF · arXiv:2603.10407 — Calibrated Gaussian Predictors
- arXiv:2208.12602 / 2108.10585 — 자기지도 SOGM 예측 · arXiv:2011.08659 — RNN 동적 점유격자
- arXiv:2502.20607 — LV-DOT · Frontiers in Robotics & AI 2025 — social navigation 리뷰
