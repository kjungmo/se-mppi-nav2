# SE-MPPI Novelty 검증 (2026-06)

> **목적:** 논문 전 선행연구 점유 확인. "Nav2-native CBF-MPPI 플러그인 전무" + escape↔CBF 조율 미점유를 검증.
> **방법:** GitHub repo/code 검색 + 웹 + ROS Discourse + awesome-list 스윕.
> **신뢰도:** arxiv/ieee/taekyung.me 등이 WebFetch 403 → 검색 스니펫 기반(다중 쿼리 일치). 카메라레디 전 PDF 직접 확인 권장.

## 판정: **4-way 교집합 유지(미점유)**

(a) Nav2-native 플러그인 + (b) 온라인 로컬미니마 탈출 + (c) 동적장애물 CBF 안전필터 + (d) escape-safety α 변조 — **단일 선행연구가 4개 중 2개 초과를 점유한 사례 없음.** 특히 CBF를 가진 Nav2-native 컨트롤러 전무.

## 항목별

**1. Nav2 CBF-MPPI 플러그인 (가장 중요): CONFIRMED NOVEL.**
GitHub repo/code 검색(`nav2_core::Controller`+barrier, `CBFCritic`/`BarrierCritic`, plugins.xml+nav2), 웹·Discourse·awesome-list 모두 **0건**.
- 근접: **CBFKit**(bardhh/cbfkit) — CBF+MPPI 있으나 standalone 노드 생성, Nav2 아님. **ntnu-arl/composite_cbf** — 멀티로터 CBF-QP 필터, MPPI/Nav2/escape 없음. **BR-MPPI**(2508.05773), **shaoanlu/mppi_cbf**(JAX), **Shield-MPPI**(2302.11719), **GS-MPPI**(2410.02154), reach-avoid MPPI-CBF(2407.13693) — 전부 연구코드, 비-Nav2.
- 결론: CBF+MPPI 개념은 기존, **배포가능 Nav2 플러그인은 미점유**(load-bearing novelty 생존).

**2. DRPA-MPPI (2503.20134): 최근접 escape baseline, 구별됨.**
- 저자: Fuke, Endo, Honda, Ishigami (Keio). **venue: IEEE CASE 2025**(IEEE Xplore 기록 — 기존 "IROS" 표기 수정 필요).
- detect-and-switch 탈출 CONFIRMED. **단, abstract가 비볼록 결과도 주장 → "convex-only" 표현 수정/완화 필요.**
- CBF/형식적 안전 없음, 정적·반응형, **공개코드 없음**, 비-Nav2.

**3. DPCBF (2510.01402): 채택할 동적 CBF, MPPI 미통합.**
- 저자: Park, Kim, Panagou. **ICRA 2026 CONFIRMED.** CBF-QP(파라볼릭, 상대속도 적응), 동적장애물(최대 100개), **MPPI/Nav2 미통합.** → MPPI 안전필터로 Nav2에 통합하는 것이 기여.

**4. escape + 형식적 CBF 결합 단일 컨트롤러: CONFIRMED NOVEL(없음).**
escape-only(DRPA, RPA-MPPI 2410.11379) ↔ CBF-safety(Shield, BR, composite) 양분, 교량 없음. 근접: 계층 MILP-MPC+Minkowski-CBF(2604.00162) — 비-MPPI/비-Nav2, planner hand-off 방식. 구별됨.

## 논문 전 수정사항 (반영)
1. **DRPA-MPPI "convex-only" 제거/완화** — 비볼록 주장 있음. 정확한 구별점: CBF 없음·형식적 안전 없음·정적/반응형·공개코드 없음·비-Nav2.
2. **venue: IROS→CASE 2025** 정정(카메라레디 전 PDF 재확인).
3. **abstract 프레이밍:** "CBF+MPPI"(기존) 대신 **Nav2-native 배포 + escape-safety α 변조 조율**을 차별점으로. α 변조 조율(d)이 **가장 깨끗한 미점유 novelty** — 강조.

## 출처
2503.20134 · 2510.01402 · taekyung.me/dpcbf · github.com/bardhh/cbfkit · github.com/ntnu-arl/composite_cbf · 2508.05773 · 2302.11719 · 2410.02154 · github.com/shaoanlu/mppi_cbf · 2410.11379 · 2604.00162

---

## 5. Conformal-CBF (C5 / N3) novelty — 2026-06-13 재검증

> 위 1~4번은 conformal+CBF를 **한 번도 평가하지 않았다.** C5(conformal이 시변 CBF
> 안전반경을 sizing)를 기여로 격상하기로 결정함에 따라 별도 재검증.

**판정: RISKY-PARTIALLY-OCCUPIED → "시스템 통합" 좁힌 프레이밍으로만 주장 가능.**

> **결정(2026-06-13, jmokang): C5 주장 철회.** conformal은 Paper 1에서 unclaimed 구현/robustness
> 디테일로 유지, 기여 주장은 Paper 2로 이연. **Paper 1 기여 = C1~C4(4-way 교집합)만** — 이미 검증됨.
> 아래 §5 분석은 Paper 2용 근거 + Paper 1 related-work 인용 계보로 보존(주장 아님).

"conformal prediction이 동적장애물 회피용 CBF 마진을 sizing한다"는 **넓은 개념은 이미
다수 선행(2024–2026)에 점유됨** — 단독 개념 기여로 주장하면 RA-L novelty challenge를
통과하지 못한다. 단, **구체적 시스템**(Nav2-native MPPI 안에서 conformal로 sizing된
DCBF 안전필터 + escape 조율 결합)은 미점유로 보임.

**내부 일관성 신호(결정적):** problem-statement 문서가 이미 conformal+CBF를 *도입 도구*로
취급했다 — SoNIC ACI "직접 이식 가능", UA-PCBF "예측+CBF 융합의 직접 선행"으로 인용하고
novelty를 α/마진 **조율**에 둠. C5 격상은 이 추론과 모순.

### 최근접 선행 (위협 순)
1. **Yang et al., ACC 2024 (2407.03569)** — adaptive CP로 예측 불확실성 온라인 정량화 +
   CBF 융합(분포무관). C5가 앉은 바로 그 교집합. 차이: 일반 unicycle, **확률제약 CBF로
   보임**(가산 반경 inflation 아닌 듯, PDF 미추출로 abstract 수준), MPPI·Nav2·escape 없음.
   *가장 직접적 위협.*
2. **CRC-CBF HRI, 2026 (2603.10392)** — "CRC로 CBF 안전 마진을 알고리즘적으로 튜닝",
   마진이 온라인 LSTM 예측으로 시변. "conformal-error가 시변 CBF 마진 sizing" 거의 그대로.
   차이: HRI 도메인, **CRC**(우리 pinball-quantile ACI 아님), MPPI 미통합. *정면 대응 필수.*
3. **Lindemann et al., RA-L 2023 (2210.10254)** — CP-for-safe-control 정전. CP 예측영역
   → **MPC 제약**(CBF 아님), CARLA. RA-L 리뷰어 기대 인용; "MPC지 CBF 아님"이 wedge 일부.
4. **Dixit et al., L4DC 2023 (2212.00278)** — adaptive CP 불확실성집합 → **MPC**(CBF 아님).
   우리 온라인 quantile 갱신의 알고리즘적 조상.
5. **UA-PCBF, 2025 (2508.20812)** — 예측 불확실성으로 predictive CBF 마진 동적 스케일.
   구조 동일하나 **불확실성이 Gaussian 예측(conformal 아님)** → "conformal 특정" 여지는
   남기나 1·2번이 닫음.
6. (snippet-only, 미확인) IEEE doc 11007767 — CP+CBF이나 불확실성이 **상태추정** 채널
   (예측 아님). 인용 전 확인.

### 방어가능 프레이밍 (주장한다면) — 맨몸 메커니즘 아닌 **시스템/통합** 주장으로만
> "online·분포무관 conformal calibrator가 매 제어주기 동적 CBF 유효반경을
> **r_eff = r₀ + q_k**로 inflation, 이를 **DCBF 안전필터 + escape 조율**(저신뢰 시 마진↑/
> 탈출 공격성↓)에 결합한 **최초의 Nav2-native MPPI 로컬 컨트롤러**."

방어 토큰: (i) **Nav2-native + MPPI**(conformal-CBF 미점유, 확인), (ii) **가산 반경
inflation(r₀+q, 삼각부등식 인증)** — 대부분 선행은 확률제약/MPC 영역, (iii) **escape↔
safety↔confidence 조율**. **"conformal로 CBF sizing한 최초"라 쓰면 안 됨 — 1·2번이 반박.**

### 권고
**C5를 좁힌(시스템 통합) 기여로만 주장.** 메커니즘은 *방법* 진술로 강등 + 1~5번을 계보로
인용; *주장*은 미점유 복합체(배포가능 Nav2 MPPI 플러그인 + escape 조율 결합)뿐.
**평가에서 F vs F_no_conformal로 고정마진 대비 측정가능 이득 못 보이면 → unclaimed robustness
디테일로 강등**(개념 지반이 혼잡해 프레이밍만으론 기여 못 버팀).

### 추가 인용 필요 (1~4번 doc·problem-statement 양쪽에 없음)
Yang ACC 2024 (2407.03569, 양쪽 부재·최우선) · CRC-CBF (2603.10392, 양쪽 부재·최근접) ·
Lindemann RA-L 2023 (2210.10254) · Dixit L4DC 2023 (2212.00278) · UA-PCBF (2508.20812,
problem-statement엔 있음). 카메라레디 전 2407.03569·2603.10392 PDF로 반경-inflation vs
확률제약 구분 확인.
