# SE-MPPI 2D 검증 (standalone)

ROS/Gazebo/센서/GPU 없이 **SE-MPPI 알고리즘(C++ 컨트롤러의 핵심)** 을 2D unicycle로
재현해 핵심 기여가 실제로 작동함을 검증한다. Gazebo 실주행이 컨테이너 인프라(GPU
부재)로 막혔기에, 알고리즘 자체를 여기서 돌려 보이는 용도.

## 실행
```bash
pip install numpy matplotlib osqp scipy
cd experiments/prototype && python3 run_validation.py
# -> figures/{utrap_escape,dynamic_cbf,coordination}.png + 메트릭 표
```

## C++ 패리티
`se_mppi_proto.py`의 수식은 `src/nav2_se_controller`를 충실히 반영:
- APF `U=0.5·η·(1/d−1/d0)²` ← `repulsion.cpp`
- gap raycast(진짜 열린 ray) ← `gap_search.cpp`
- look-ahead-point DCBF-QP(OSQP) ← `cbf_safety_filter.cpp`
- α 변조 + TTC override ← `escape_safety_coordinator.cpp`
- entrapment 단조 진행 stall ← `entrapment_detector.hpp`

## 결과 (실측)

| 시나리오 / 구성 | 도달 | 충돌 | 시간 | min-clear |
|---|---|---|---|---|
| U-trap / **Stock MPPI** | ❌ | — | (정지) | 0.33 |
| U-trap / **SE-MPPI(escape)** | ✅ | — | 27.7s | 0.32 |
| Dynamic / **No CBF** | ❌ | **💥 충돌** | — | -0.00 |
| Dynamic / **SE-MPPI(CBF)** | ✅ | 안전 | 18.6s | 0.01 |
| Coord / SE-MPPI(F) · Independent(E) | ✅ | 안전 | 27.7s | 0.32 |

**그림으로 입증된 것:**
1. **`utrap_escape.png`** — Stock MPPI는 벽 앞 로컬미니마에 **갇힘**(x≈1.2 고정);
   SE-MPPI는 **감지·탈출**로 벽을 돌아 **완주**.
2. **`dynamic_cbf.png`** — 횡단 동적장애물에 No-CBF는 **충돌**; 속도-인지 CBF 필터는
   **예측 회피·완주**.
3. **`coordination.png`** (핵심) — 탈출 phase(빨강)에서 CBF 게인 **α가 2→6로 상승**해
   탈출을 허용하되, **slack≈0 유지 = certified-safe**. 즉 "α를 올려도
   forward-invariance(h≥0)는 유지되어 탈출이 안전"이라는 **논문 핵심 명제**를 입증.

## 정직한 범위·한계
- 2D point/unicycle 단순화. **글로벌 플래너 없음** — Nav2는 Smac으로 장애물 뒤
  경로를 라우팅하지만, 이 proto는 그게 없어 탈출 시 **gap 방향에 임시 subgoal**
  (설계의 free-space gap subgoal)을 두어 우회한다. 즉 **메커니즘 검증용**이지
  Nav2 컨트롤러 그 자체가 아니다.
- 파라미터는 **예시용 튜닝**. 정량 ablation(E vs F의 측정 가능한 이득)과 벤치마크
  수치는 동작하는 Nav2 시뮬에서의 **풀 평가(M6)** 의 몫이다(여기선 메커니즘만 확인).
- 이 시나리오에서 E와 F는 동일하게 안전 완주(benign) — 조율의 *정량적* 이득은
  좁고 동적인 벤치마크에서 드러난다(평가 프로토콜 §3 참조).

---

# Multi-SE-MPPI 2D 검증 (L9 / N1)

같은 머신리(MPPI·detector·CBF-QP)를 재사용해 **다수의 SE-MPPI 로봇**이 좁은 공유
공간에서 서로를 가두는 문제(M1 상호 deadlock)와 그 해법(책임분배 CBF + 우선권
escape)을 검증한다.

## 실행
```bash
cd experiments/prototype && python3 run_multirobot_validation.py
# -> figures/multirobot_{corridor,intersection}.png + 메트릭 표 + 검증 게이트
```

## 구성 (A/B)
- **independent** (베이스라인): 각 로봇이 **무수정 단일로봇 SE-MPPI** 실행 — 상대를
  완전책임(λ=1)·속도반응 동적장애물로 취급, entrapment 시 단일로봇 gap escape.
- **coordinated** (Multi-SE-MPPI): ① 쌍별 CBF를 **책임 λ_ij+λ_ji=1로 분배**
  (Egerstedt식 분담 — 속도 교환 불필요, 합치면 ḣ+αh≥0 복원), ② 상호 deadlock
  감지(인지 기반: entrapped + 근접 정지 로봇) 시 **결정적 우선권**: PASS 로봇은
  α↑·λ↓·blocker 기준 우회 차선 subgoal로 통과, YIELD 로봇은 우측 파킹·hold,
  ③ 통과 중 blocker의 MPPI soft 회피 존 축소 — **안전 마진의 소유권을 CBF로
  이전**(α 상승과 동일한 조율 원리), ④ 근접 비상시(λ=1+속도항 복원) TTC override의
  멀티로봇 판.

## 결과 (실측, seed 고정)

| 시나리오 / 구성 | 전원 도달 | 시간 | min 로봇간 간격 |
|---|---|---|---|
| corridor(1.4m) / **independent** | ❌ **교착**(진동) | 60s 타임아웃 | +0.30 m |
| corridor(1.4m) / **coordinated** | ✅ | **27.7 s** | **+0.12 m (무충돌)** |
| intersection(4로봇) / independent | ✅ | 22.4 s | +0.30 m |
| intersection(4로봇) / coordinated | ✅ | 28.7 s | +0.28 m |

**그림으로 입증된 것:**
1. **`multirobot_corridor.png`** — independent는 두 로봇이 통로 중앙에서 마주보고
   **진동·교착**(M1 그대로: 단일로봇 escape는 "양보" 개념이 없어 gap을 서로
   맞물려 고름); coordinated는 yielder가 우측 파킹 후 passer가 **0.12m 간격으로
   certified-safe 통과**, 양쪽 모두 완주.
2. **`multirobot_intersection.png`** — 열린 교차로(4로봇 대척 스왑)에서는 둘 다
   해결: 공간이 충분하면 단일로봇 회피로 족하고, **조율의 이득은 좁은 공유
   공간에서 발생**한다(문제정의 §1과 일치). coordinated가 약간 느린 것은 우선권
   프로토콜의 보수성 비용.

## 정직한 범위·한계
- 우선권 타이브레이크는 ID 관례(경량 브로드캐스트/사회규범의 대역). 인지-only
  타이브레이크(우측통행 등)는 N2 과제.
- min_h가 통과 순간 −0.002까지 스침(이산시간·동기 갱신 탓) — 물리 간격은 항상
  +0.116m 이상(eff_r의 margin 0.06이 흡수). 형식 명제의 이산화 보정은 N2.
- 2D·플래너 없음 등 단일로봇 proto의 한계 동일. 정량 멀티로봇 벤치마크(성공률·
  throughput 분포)는 L11 하니스의 멀티로봇 시나리오(N3)의 몫.

---

# FM-Shielded SE-MPPI 2D 검증 (L10 / N1)

"**FM이 제안하고 CBF가 보증한다**"의 구조 검증 — FM을 오라클로 대체해 *모델 성능과
안전 구조를 분리*해 진행(설계 §3, N1). 제안은 **저주파(0.5s 주기) 비동기**로 소비되고
(제어루프 밖), 모든 명령은 무조건 CBF 사영을 통과한다(TTC override 포함).

## 실행
```bash
cd experiments/prototype && python3 run_fm_shield_validation.py
# -> figures/fm_shield.png + 메트릭 표 + 검증 게이트
```

## 결과 (실측, seed 고정)

| 런 | 도달 | 충돌 | 시간 | min-clear |
|---|---|---|---|---|
| (기준) U-trap 휴리스틱 SE-MPPI | ✅ | — | 27.7 s | +0.32 |
| U-trap + **OracleFM** (의미적 우회 제안) | ✅ | — | **18.2 s** | +0.34 |
| Dynamic + **AdversarialFM** (장애물 조준 제안·max bold) | ❌(진행 희생) | **무충돌** | — | **+0.34** |
| U-trap + **SilentFM** (제안 없음, degrade) | ✅ | — | **27.7 s (기준과 동일)** | +0.32 |

**입증된 구조 (`fm_shield.png`):**
1. **제안의 가치** — 오라클의 의미적 우회 제안은 stall-감지 휴리스틱보다 **34% 빠른
   완주**(정체 후 탈출이 아니라 선제 우회).
2. **거부권 (핵심)** — *최악의 환각*(움직이는 장애물에 lead-pursuit + max boldness)도
   **충돌 0**: 제안은 목적함수만 바꾸고 제약은 CBF 소유 → **forward-invariance는 제안
   품질과 무관**(설계 §3.3 명제의 기계적 검증). 성능만 망가지고 안전은 불가침.
3. **Graceful degrade** — FM 침묵 시 휴리스틱 SE-MPPI와 **동일 거동**(27.7s) — FM은
   순수 추가 레이어이며 의존성이 아님.

## 정직한 범위·한계
- FM은 오라클(지도 정답/적대 규칙) — 실제 학습 모델 연결(ViNT류·소형 정책)은 N2.
- boldness→α 매핑은 최소형(bold=α↑). margin 변조 등 2변수 조율은 L2 N3와 합류.
- 2D proto 한계 동일(플래너 없음 등). 사회성 A/B(HuNavSim)는 N4·L11 하니스의 몫.
