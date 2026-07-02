# L10 내비 파운데이션 모델 × 클래식 안전층 하이브리드 — 문제정의 (Phase C/D 후보)

> **작성일:** 2026-06-10 · **상태:** 문제정의 초안 · **층:** L10(학습)
> **위치:** 전체 아키텍처 §5 기여지도 — "FM의 제안 + CBF의 보장 = 안전한 학습내비".
> **목표:** 학습된 내비 정책/파운데이션 모델(FM)이 *전역적·의미적 가이드와 escape 제안*을 내고, SE-MPPI의 **CBF 안전층이 그것을 certified-safe로 보증**하는 하이브리드.
> **인용 신뢰도:** 외부 방법은 (검색요약 기반) — **N0 1차 검증 완료**: `2026-06_extension-tracks-n0-verification.md`(갭 수정 포함 — 부품은 기성, 차별점은 조율). 카메라레디 전 PDF 정독 항목 동 문서 §5.

---

## 1. 동기 — 학습내비의 약점과 클래식 안전의 약점은 상보적

| | 학습 내비(FM/RL/VLA) | 클래식(SE-MPPI) |
|---|---|---|
| 강점 | 의미·맥락·사회규범·장기 직관, 새 상황 일반화, "어디로 가야 하나" | 형식적 안전(forward-invariance), 실시간, 해석가능, 배포가능 |
| 약점 | **안전 보장 없음**(환각·분포이탈), 검증 곤란 | 의미·장기 추론 약함, 휴리스틱 escape, sim 튜닝 의존 |

→ **FM이 제안하고 CBF가 보증**하면 두 약점이 상쇄된다. SE-MPPI는 이미 (escape + CBF + 조율) 안전층을 가지므로, 그 위에 FM 제안기를 얹는 **shielding 아키텍처**의 이상적 토대다.

---

## 2. 선행연구 미니서베이 (잠정 — 원문 미확인)

**(a) 내비 파운데이션 모델 / 학습 정책.** ViNT·NoMaD류 내비 FM, 사회적 RL 내비, VLA(vision-language-action). 강점은 일반화·의미, **공통 약점은 형식 안전 부재.**
**(b) 안전 shielding(학습×CBF).** 학습 정책 출력을 CBF-QP로 사영(safety filter/shield), RL-CBF, "neural CBF". → *제안=학습, 보증=CBF* 패턴 확립. 단 대부분 단순 도메인·비-Nav2.
**(c) 예측·계획에서의 conformal**(L2와 공유): 학습 출력의 불확실성을 분포무가정으로 보정.

> **갭(잠정):** *내비 FM 제안 + escape 인지 + DCBF 안전 + Nav2-native 배포*의 결합 미점유. 특히 "FM이 **escape subgoal/우회 의도**를 제안하고, SE-MPPI의 escape-safety 조율이 그것을 certified-safe로 실행"하는 구조는 단일-로봇 조율의 *상위 제안기* 확장으로 미발견.

---

## 3. 제안 — FM-Shielded SE-MPPI (가칭)

### 3.1 아키텍처 (계층)
```
[FM 제안기] (비동기, 저주파 1–5Hz, 의미·장기)
   → 제안: subgoal/우회 의도/사회적 선호 (예: "왼쪽 갭으로 우회", "사람에게 양보")
   ↓  (제안을 비용/subgoal로 변환)
[SE-MPPI] (실시간 10–20Hz)
   → MPPI 샘플링 + escape critic(FM subgoal 반영) + CBF 안전필터(보증) + 조율
   → certified-safe cmd_vel
```
- **FM은 제어루프 밖**(저주파, GPU). 컨트롤러는 최신 제안을 *힌트*로 소비. 제안이 늦거나 없으면 SE-MPPI 단독으로 graceful degrade.
- **CBF는 거부권**: FM 제안이 위험하면 CBF가 사영/제동 → **FM 환각이 충돌로 이어지지 않음**(load-bearing 안전).

### 3.2 FM 제안의 진입점(기존 훅 재사용)
- **escape subgoal**: `gap_search`의 subgoal을 FM 제안으로 대체/가중 → 휴리스틱 gap 대신 *의미적* 우회.
- **사회적 비용**: FM의 사회 선호를 MPPI critic 비용으로(양보·개인공간).
- **조율 힌트**: FM이 "지금은 과감/조심"을 제안 → α·margin 조율의 추가 입력(단, 안전 하한은 CBF가 강제).

### 3.3 안전 명제
FM 제안은 **목적함수(어디로)만 바꾸고 제약(안전)은 못 바꾼다**: cmd는 항상 CBF-사영을 통과하므로, FM이 무엇을 제안하든 §IV-E 명제(∀α>0, δ=0 → h≥0)가 유지 → **FM 품질과 무관하게 forward-invariance.** (제안 품질은 *성능*에, 안전은 *CBF*에 분리.)

---

## 4. 마일스톤

| M | 내용 | 산출물 |
|---|---|---|
| N0 | 내비 FM·shielding·neural-CBF 선행 원문 검증 + novelty 매트릭스 | 검증 문서 |
| N1 | 인터페이스: FM 제안 메시지(subgoal/선호) → SE-MPPI critic/subgoal 훅 — **완료**: `experiments/prototype/fm_shield_proto.py`(Proposal{subgoal, boldness} + 저주파 비동기 소비 + 무조건 CBF 사영). OracleFM은 휴리스틱 대비 **34% 빠른 완주**(18.2s vs 27.7s), SilentFM은 기준과 **동일 거동**(graceful degrade). | 통합 스텁(FM=오라클) ✓ |
| N2 | 경량 FM/정책 연결(기성 내비 FM 또는 소형 정책) — 비동기 노드 | 제안기 노드 |
| N3 | CBF 거부권 검증: FM이 위험 제안해도 무충돌 유지(적대적 테스트) — **2D proto 선행 검증 완료**: AdversarialFM(동적장애물 lead-pursuit + max bold)에도 **충돌 0**(min-clear +0.34m), 진행만 희생. Nav2 통합 stress 테스트는 N2 후. | 안전 stress 테스트 (proto ✓) |
| N4 | HuNavSim에서 SE-MPPI vs FM-Shielded SE-MPPI A/B(사회성·성공률·안전) | 논문4 §실험 |

→ **L11 하니스 재사용** · **L2 conformal 재사용**(FM 불확실성 보정). FM=오라클로 먼저 통합(N1)해 *안전 거부권 구조*부터 검증 — 모델 성능과 안전 구조를 분리해 진행.

---

## 5. 리스크

- **FM 성숙도·연산**: 대형 FM은 무거움 → 저주파·비동기·degrade 설계로 흡수. 소형 정책부터.
- **sim-to-real / 분포이탈**: FM의 본질적 약점 → *그래서 CBF 보증이 핵심.* 안전은 모델에 의존하지 않게 설계(N3가 이를 검증).
- **평가 순환**: sim FM을 sim에서 평가 → 교차 시나리오 + 안전은 모델 무관 보장이라 영향 적음.
- **novelty**: N0 전까지 차별점 잠정(shielding은 기성 — 차별점은 *내비 FM의 escape 제안 + SE-MPPI 조율 + Nav2 배포*).

---

## 6. SE-MPPI와의 관계 (3축 확장의 완성)

SE-MPPI의 "조율"을 세 방향으로 확장하는 시리즈의 마지막 축:
- **시간**(Phase C, L2): 현재 → 미래(예측+conformal).
- **공간**(L9): 자기 안↔밖 → 로봇 간(분산 조율).
- **추상**(L10): 휴리스틱 → 의미·학습 제안(FM), CBF가 안전 하한.

세 경우 모두 **SE-MPPI의 CBF 안전층이 불변의 토대**이고, 그 위에서 *제안*이 시간·공간·추상으로 풍부해진다. 안전은 항상 클래식 CBF가 책임진다 — 이것이 "안전한 학습내비"의 설계 철학.

> 살아있는 문서. N0 결과로 §2 갭·§3 명제를 갱신.
