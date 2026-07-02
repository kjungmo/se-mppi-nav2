# 확장 트랙(L2·L9·L10) N0 — 선행연구 검증·갭 확정

> **작성일:** 2026-06-11 · **상태:** 1차 검증 완료(웹검색 스니펫 기반 — arXiv 직접 fetch는
> 네트워크 정책 403, 제목·초록 수준 확인. 카메라레디 전 PDF 정독 필요 항목은 ☐ 표기)
> **선행:** `2026-06_se-mppi-novelty-verification.md`(논문 1 SE-MPPI — 기검증).
> **목적:** 세 문제정의 문서의 "(검색요약 기반·미확인)" 갭 주장들을 실제 문헌으로
> 업그레이드/수정하고, 각 트랙의 차별점을 확정한다.

---

## 1. L2 SE-Predict (conformal × CBF × escape)

### 검증된 선행
- **ACP→CBF (가장 근접)**: *Safety-Critical Control with Uncertainty Quantification using
  Adaptive Conformal Prediction* (arXiv:2407.03569) — **우리 N3의 핵심 메커니즘이 선행에
  존재**: ACP로 온라인 예측 불확실성 정량화 → 확률적 CBF 제약 → MPC. 검증 도메인:
  단일적분기/unicycle **시뮬**(멀티로봇 포함). ☐ PDF로 α-게인 변조 여부 확인.
- **ACP-SBC**: 적응 conformal + safety barrier certificates(멀티로봇 충돌회피 보장) —
  conformal×CBF×멀티로봇 결합도 존재.
- 주변: Interaction-aware CP for crowd navigation (arXiv:2502.06221), ACP for motion
  planning among dynamic agents (arXiv:2212.00278), CP-SIPP/time-aware CP-RRT
  (arXiv:2511.18170), ATOM-CBF(OOD 인지 안전, arXiv:2511.08741).

### 갭 수정 (정직)
- ~~"conformal 오차상계를 CBF 시변 반경으로"가 미점유~~ → **기성**(2407.03569 등).
  **이 일반 주장은 novelty로 쓰지 말 것.**
- **확정된 차별점:**
  1. **q-신뢰 escape 게이팅** — 예측 신뢰(q)로 escape 공격성(α)을 변조하는 **2변수
     escape-safety 조율**("믿을 때만 과감히"). conformal×CBF 선행들은 *안전 제약*만
     다루고 *탈출 행동과의 조율*이 없음(escape 개념 자체가 없음).
  2. **costmap-네이티브 파이프라인** — 클러스터 트래킹 잔차의 온라인 셀프 채점
     (전용 perception 없이 Nav2 costmap만으로), 벽-freeze(N1)와 한 몸의 통합 스택.
  3. **Nav2 플러그인 배포** + 재현 벤치(L11). 선행은 커스텀 시뮬 스택.

---

## 2. L9 Multi-SE-MPPI (책임분배 CBF × deadlock 우선권)

### 검증된 선행
- **Safety Barrier Certificates** (Wang·Ames·Egerstedt, arXiv:1609.00651, IEEE TRO):
  멀티로봇 쌍별 CBF-QP, **상호(reciprocal) 분담**으로 형식적 무충돌 — 우리 λ-분담
  제약의 원형(균등 1/2 분담). 가속도 한계·이종 동역학까지.
- **PrSBC** (arXiv:1912.09957): 불확실성 하 확률적 SBC.
- **CBF deadlock/liveness** (arXiv:2012.10261): CBF 컨트롤러의 멀티에이전트
  **deadlock 분석·해소**(liveness) — deadlock×CBF 결합도 연구됨. ☐ 해소 방식이
  우선권/비대칭 λ인지 PDF 확인.

### 갭 수정 (정직)
- ~~"책임분배 CBF" 자체~~ → **기성**(SBC의 핵심이 바로 그것).
  ~~"deadlock 해소×CBF"~~ → 존재(2012.10261).
- **확정된 차별점:**
  1. **비대칭 역할 λ(pass/yield) + α 변조의 결합** — deadlock 시 budget 몫과 CBF
     게인을 *역할에 따라 함께* 바꾸는 escape-safety 조율의 분산판. SBC 계열은 균등
     분담·고정 α, liveness 계열은 perturbation/규칙 중심.
  2. **샘플링 MPC(MPPI) 스택과의 통합** — SBC 계열은 단순 적분기 정류층.
     entrapment 감지(실행 경로 진행 기반)·escape critic과 한 컨트롤러.
  3. **Nav2-native 멀티로봇 플러그인**(이웃 odom만으로, 인지 우선) — 미발견.

---

## 3. L10 FM-Shielded SE-MPPI (내비 FM × 인증 안전층)

### 검증된 선행
- **ViNT** (arXiv:2306.14846) / **NoMaD**: 내비 파운데이션 모델 — 일반화·전이는
  입증, **충돌 안전 보장 없음**(문제정의 §1 전제 확인됨).
- **CARE** (arXiv:2506.03834, 가장 근접): *비주얼 내비 FM의 안전 강화* — repulsive
  estimation 기반 충돌회피 래퍼. TurtleBot4에서 ViNT 70%→100%, NoMaD 20%→50%
  goal-reaching. **형식적 보장 없음**(repulsion 휴리스틱, CBF/forward-invariance 아님).

### 갭 수정 (정직)
- "FM 안전 래퍼" 공간은 **이미 활성**(CARE) — "최초의 FM 안전층" 류 주장 금지.
- **확정된 차별점:**
  1. **인증된 거부권** — CARE는 휴리스틱 repulsion, 우리는 **CBF 사영(forward-invariance,
     제안 품질과 무관)** + 적대 제안 stress 테스트로 기계 검증(L10 N1).
  2. **escape 의도 제안 인터페이스** — FM이 subgoal+boldness를 제안하고 *조율기*가
     α로 수용 수위를 결정(escape 개념이 CARE에 없음).
  3. **Nav2-native + graceful degrade**(FM 침묵 시 기준과 동일 거동 — 측정됨).

---

## 4. 갱신된 novelty 한 줄 요약 (세 트랙 공통 패턴)

각 트랙의 *부품*(conformal×CBF, 분담 CBF, FM 안전 래퍼)은 **모두 기성**이다.
공통으로 미점유인 것은 **escape-safety "조율"의 확장축**이다:
- L2: 신뢰 기반 α/margin **2변수** 조율 (시간축)
- L9: 역할 기반 λ/α **비대칭** 조율 (공간축)
- L10: boldness 기반 α 조율 + 인증 거부권 (추상축)
— 모두 단일 Nav2-native 스택에서, 재현 벤치(L11)와 함께. 논문 시리즈의 프레임은
"부품 발명"이 아니라 **"조율의 일반화와 배포 가능한 통합"**으로 잡을 것.

## 5. 남은 검증 (카메라레디 전, 워크스테이션/일반망에서)
- ☐ 2407.03569 PDF: α 변조 유무, 잔차 정의, 보장 형식(고확률 vs 점근).
- ☐ 2012.10261 PDF: deadlock 해소 메커니즘 상세(우선권? 섭동?).
- ☐ CARE PDF: 래퍼 위치(액션 필터?), 실로봇 범위.
- ☐ ACP-SBC 원문 식별(검색 스니펫에서 제목 미확정).

> 세 문제정의 문서의 §2(선행)·§3(갭) 본문에 본 문서를 반영했고, "(미확인)" 마커를
> "(N0 1차 검증)"으로 갱신했다. 신뢰도 표기는 유지: 스니펫 수준 검증임.
