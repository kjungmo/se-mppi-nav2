# L2 SE-Predict — 학습 동적장애물 예측 설계

> **작성일:** 2026-06-10 · **상태:** 설계 (구현 대기) · **층:** L2(인지)
> **선행:** `docs/research/2026-06_learned-dynamic-prediction-problem-statement.md`(Phase C 동기·서베이·갭).
> **목표:** SE-MPPI의 CV 트래커를 **정적/동적 분류 + 단기 궤적 예측 + conformal 보정**으로 교체해, CBF/TTC 입력의 정확도와 *형식적 안전 주장*을 동시에 강화.
> **인용 신뢰도:** 외부 방법은 (검색요약 기반·원문 미확인) — N0에서 검증.

---

## 1. 설계 원칙 (문제정의 §3 재확인)

1. **인터페이스 호환·점진 교체** — 기존 `TrackedObstacle{position, velocity, radius}`를 깨지 않고 확장.
2. **LiDAR-only 우선** — 라벨 없는 자기지도. 카메라 융합 후순위.
3. **불확실성은 형식 보장으로** — conformal 오차상계 `q_t`를 CBF 시변 반경으로.
4. **조율 연결(novelty)** — 불확실성을 escape-safety α·margin 2변수 조율에 연결.
5. **제어루프 밖** — 예측은 비동기 노드(5–10Hz), 컨트롤러는 최신 예측 소비. 제어 cycle에 학습 inference 금지(라이브 런 compute 교훈).

---

## 2. 데이터 계약 (인터페이스)

### 2.1 확장 타입
```
TrackedObstaclePred {
  position   : Vec2          # 현재 추정 위치
  velocity   : Vec2          # 현재 추정 속도 (호환 유지)
  radius     : double        # 물체 반경
  horizon    : [Vec2]        # 예측 위치 p̂(t_k), k=1..K (Δt 간격)
  q          : [double]      # conformal 오차상계 q_{t_k} (시변, m)
  is_dynamic : bool          # 정적/동적 분류 결과
}
```
- **하위호환**: `horizon` 비었거나 1점이면 SE-MPPI는 기존 CV 경로로 동작(점진 마이그레이션).
- ROS 메시지: 내부 struct 우선, 노드 분리 시 커스텀 msg(`se_msgs/TrackedObstacleArray`).

### 2.2 SE-MPPI 소비 지점(기존 코드 훅)
- `cbf_safety_filter.cpp`의 `eff_r = r + R_o + m` → **`eff_r(t_k) = r + R_o + m + q_{t_k}`** (시변 반경).
- `cbf` 의 장애물 위치를 예측 `p̂(t_k)` 사용(현재는 등속 외삽).
- `escape_safety_coordinator.cpp`의 `minTimeToCollision` → 예측 궤적 기반 TTC.
- 신규: coordinator에 **margin 변조** 추가(불확실성↑ → margin↑, escape 공격성↓).

---

## 3. 파이프라인 (3 스테이지)

### 3.1 [N1] 정적/동적 분류 + 속도장
- **입력**: 과거 T LiDAR 스캔(또는 누적 점유격자).
- **방법(택1, 점진)**:
  - (a) **고전 DOGM**(Dynamic Occupancy Grid Map, 파티클/베이즈) — 학습 불필요, 셀별 free/static/dynamic + 속도. H1(정적/동적 구분) 즉시 해결, 라이브 런의 벽-freeze 근본 차단.
  - (b) **학습 DOGM**(RNN/CNN, 자기지도) — 더 정확, 데이터 필요.
- **출력**: 동적 셀 마스크 + 셀 속도 → 클러스터링으로 per-agent.
- **우선순위**: (a)부터(빠른 승리, 기존 트래커 대체) → (b)는 N2와 함께.

### 3.2 [N2] 단기 궤적 예측
- **입력**: per-agent 과거 트랙(N1 출력 + 연관).
- **베이스라인**: CV(현재) → CVCA(등가속) → 학습(Social-LSTM/Implicit류, 멀티모달).
- **출력**: `horizon` p̂(t_k), K≈10–30 @ Δt=0.1–0.2s (호라이즌 1–3s).
- **평가**: ADE/FDE vs CV 베이스라인(같은 데이터).

### 3.3 [N3] Conformal 보정
- **방법**: Adaptive Conformal Inference(ACI) — 온라인으로 잔차 분위수 추적, 시변 `q_{t_k}` 산출(목표 커버리지 1−α_CP).
- **보장**: 분포무가정. 예측이 틀려도 `q_t`가 부풀어 CBF 반경이 커짐 → **bounded error 하 forward-invariance** 유지.
- **출력**: 각 예측 스텝 `q_{t_k}` → §2.2 시변 반경.

---

## 4. 안전 명제 (SE-MPPI 명제의 확장)

기존(논문 §IV-E): δ=0이면 ∀α>0에서 h≥0 유지.
**확장:** 예측 위치 오차가 `‖p_o(t)−p̂_o(t)‖ ≤ q_t`로 유계이고 CBF가 `h = ‖p_L−p̂_o‖² − (r+R_o+m+q_t)²`를 쓰면, 실제 거리 기준 안전여유가 보존된다 → **예측을 쓰되 보정된 불확실성만큼 보수화**해 안전을 잃지 않음. (증명 스케치: 삼각부등식으로 실제 h_real ≥ 0이 시변 반경 CBF의 h≥0에서 따름.)

이로써 논문의 정직성 약점(H3: 완벽한 예측 가정)이 닫힌다.

---

## 5. 조율 확장 (2변수)

```
entrapped & 예측신뢰(q 작음)  → α↑(escape 허용), margin 보통   # 과감히 탈출
entrapped & 예측불신(q 큼)    → α 보통, margin↑               # 조심히 탈출
TTC 임박                      → α_base, margin↑               # 안전 우선(기존)
```
"예측을 믿을 수 있을 때만 과감히 탈출" — 가장 깨끗한 미점유 novelty(문제정의 §2 갭).

---

## 6. 마일스톤 (문제정의 N0–N4 구체화)

| M | 내용 | 산출물 | 검증 | 상태 |
|---|---|---|---|---|
| N0 | 선행 원문(arXiv:2601.10233 등) 정독 + novelty 매트릭스 v2 | 검증 문서 | 차별점 확정 | 미착수 |
| N1 | DOGM(고전) 정적/동적 분류 → tracker v2 (CV 대체) | C++ 모듈 + 단위테스트 | 벽-freeze 재현 안 됨 | **구현·검증 완료** (`static_occupancy_filter` + tracker 통합, gtest 9종 — `WallPhantomVelocitySuppressed`가 벽-freeze 회귀 테스트, 186 tests 0 fail) |
| N2 | 궤적 예측기(CV→학습) + ADE/FDE 평가 | 예측 노드 | CV 대비 오차↓ | **고전 단계 완료** — 영속 트랙(이력 10프레임, miss 생존) + LS 적합 `TrajectoryPredictor`(CV/CVCA) → `TrackedObstacle.horizon`(§2.1 계약, 빈 값=레거시). ADE/FDE 하니스(`experiments/prediction/`): **CVCA가 accel/turn에서 ADE 52~62%↓**, weave(진동)에선 악화 — 1s 이력으론 등가속·진동 식별 불가(측정된 학습예측기 동기). 기본값 CV(보수), CVCA opt-in. 학습 모델은 워크스테이션(동일 프로토콜로 평가) |
| N3 | ACI 보정 + CBF 시변반경 통합 + 명제 | SE-MPPI 통합 | 커버리지·완주율 | **구현·단위검증 완료** — `ConformalCalibrator`(스텝별 온라인 분위수 추적, 분포무가정; 트랙 간 풀링) → `TrackedObstacle.q` → CBF `eff_r += q[0]`(시변 반경, §4 명제의 기계화) + coordinator **q-신뢰 게이트**(`se_q_trust_threshold` — §5 2변수 조율: "예측을 믿을 때만 과감히, 불신 시 α=base+margin은 q가 자동 확대"). 합성 잔차에서 0.9 분위수 수렴·분포이동 추적·캡 검증(gtest 7종). **실커버리지·완주율 측정은 라이브 sim(N4)** |
| N4 | DynaBARN/HuNavSim에서 SE-MPPI(CV) vs (SE-Predict) A/B (L11 하니스 재사용) | 논문2 §실험 | 충돌↓·과보수↓ | 미착수 |

**N1 구현 노트(2026-06):** `StaticOccupancyFilter` — 월드 고정(rolling window 불변) 셀별 점유 지속성 그리드. occupied≥`se_static_min_frames`(기본 10 ≈ 1s@10Hz) → static; 클러스터 static-cell 비율 ≥ `se_static_fraction`(기본 0.5, 센서가 벽을 한 프레임에 2배로 드러내도 유지) → `is_dynamic=false` + 속도 0. 컨트롤러 동적 필터에 is_dynamic 거부권 추가 — **연관 지터의 유령 속도가 벽에 붙어도 CBF에 도달 불가**(벽-freeze 근본 차단). 한계(문서화됨): 자기 반경/초보다 느린 장애물은 static으로 표류 — 준정적이므로 costmap 경로가 처리, DOGM 의미론과 일치.

→ **L11 평가 하니스를 그대로 재사용** (예측 ablation = config 하나 추가).

---

## 7. 리스크

- **연산**: 학습 inference는 별도 노드·GPU. 고전 DOGM(N1)은 CPU 가능 → 먼저.
- **sim 학습 편향**: HuNavSim 모델로 학습→평가 순환 → 교차 파라미터 평가 + CP가 불일치 흡수 명시.
- **novelty**: N0 전까지 차별점 잠정.
- **통합 안전성**: 시변 반경이 너무 커지면 과보수→entrapment 악화. 조율(margin)과 상한으로 방어.

---

## 8. 현재 위치

- 문제정의 완료(Phase C 착수 문서). 본 설계로 **인터페이스·파이프라인·명제·마일스톤** 확정.
- **다음 행동**: N0(원문 검증) → N1(고전 DOGM으로 정적/동적 분류 — 라이브 런 벽-freeze의 근본 해결이자 빠른 승리). N1은 SE-MPPI 코드(`dynamic_obstacle_tracker`)의 직접 강화라 즉시 가치.

> 살아있는 설계. N0 결과로 차별점이 바뀌면 §4·§5 갱신.
