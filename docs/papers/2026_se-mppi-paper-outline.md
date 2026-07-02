# SE-MPPI 논문 구조 초안

> **작성일:** 2026-06-08
> **상태:** 구조 초안 (실험 수치 미확보 — 평가 후 채움)
> **타깃 venue:** IEEE RA-L (+ICRA/IROS 동시), 대안 CoRL 워크숍
> **선행:** 문제정의·설계·평가 프로토콜 문서, SOTA 서베이

---

## 제목(가안)
**"Safe-Escape MPPI: Coordinating Online Local-Minima Escape with Control-Barrier-Function Safety in a Nav2-Native Controller"**

대안: "SE-MPPI: Certified-Safe Escape from Local Minima for Sampling-Based Mobile Robot Navigation"

---

## Abstract (골격, ~150단어)
1. **문제:** MPPI는 Nav2 기본급 SOTA 로컬 컨트롤러지만 (i) 제한된 호라이즌→로컬미니마 함정, (ii) 형식적 안전 부재 — 좁고 동적인 환경에서 *동시에* 문제.
2. **기존 한계:** escape 연구(always-on 증강)는 안전 보장 없음; CBF-MPPI 연구는 escape를 악화시킬 수 있고 배포 가능한 Nav2 플러그인 전무.
3. **제안:** SE-MPPI — 온라인 로컬미니마 **감지·탈출**(거리장 APF + gap 탐색)과 동적장애물 **DCBF 안전필터**를, **escape-safety 조율**(entrapment 연동 α 변조)로 통합한 단일 Nav2-native 컨트롤러.
4. **핵심 통찰:** forward-invariance(무충돌)는 α>0에서 항상 유지 → 탈출 기동이 *certified-safe*. 조율이 기여(두 레이어 단순 합 아님).
5. **결과:** (실측 후) 좁고 동적 환경에서 성공률↑+충돌률↓ 동시; 단위·런타임 검증; 공개 구현.

---

## 1. Introduction
- 모바일 내비게이션과 MPPI의 부상(Nav2). 두 약점 제시 + 동시 발생성.
- 그림 1: U-trap에서 stock MPPI 함정 vs SE-MPPI 탈출(certified-safe).
- 기여 4가지 (C1–C3, 문제정의 문서와 일치):
  1. escape↔safety **조율** 메커니즘(핵심): entrapment 감지에 α를 변조해 탈출을 certified-safe 범위에서 허용, 동적 TTC 임박 시 안전 우선.
  2. 온라인 detect-and-switch 탈출(거리장 APF + 자유공간 gap 탐색) — 비볼록/동적 확장.
  3. 차동구동용 look-ahead-point DCBF 안전필터(동적장애물, CV 예측).
  4. **Nav2-native 배포 가능 플러그인** + 재현 가능 오픈소스/벤치마크.

## 2. Related Work
- **MPPI & 로컬 컨트롤러**(Nav2 MPPI, TEB, RPP).
- **로컬미니마 탈출**: log-MPPI, SVG-MPPI, Biased-MPPI, **DRPA-MPPI**(detect-switch 유일, CBF없음·형식적 안전 없음·정적/반응형·코드없음·비-Nav2) — 우리 baseline/차별점.
- **CBF × MPPI**: Shield-MPPI, GS-MPPI, DualGuard, BR-MPPI, **DPCBF**(동적, QP-only) — 알고리즘 선행, 단 Nav2 플러그인 전무.
- **포지셔닝:** escape+CBF **조율** ∩ 동적 DCBF ∩ Nav2-native — 단일 선행연구 미점유 교집합(표).

## 3. Method
- **3.1 개요:** Nav2 파이프라인 + SE-MPPI 4 컴포넌트 그림(아키텍처 §2).
- **3.2 Entrapment 감지(단일 source):** 단조 furthest-reached 진행도 stall + near-goal 억제. 컨트롤러가 유일 detector, critic·coordinator가 공유.
- **3.3 Escape(detect-and-switch):** 거리장 APF `U=½η(1/d−1/d0)²` + gap 탐색(raycast로 goal 최근접 개구부 → attractive subgoal). entrapment 시에만 비용 주입.
- **3.4 CBF 안전필터:** look-ahead-point로 (v,ω) 상대차수 1화 → 장애물별 `ḣ+αh≥0` 선형부등식 → slack-QP(OSQP). hard_safe 판정·제동.
- **3.5 Escape-safety 조율(핵심):** α = base / escape / (TTC 임박 시 base). **명제:** ∀α>0에서 h≥0 유지 ⇒ 탈출 certified-safe. (증명 스케치)
- **3.6 동적장애물:** 코스트맵 클러스터링 + CV 트래킹 → CBF/TTC 입력.

## 4. Implementation
- ROS2 Jazzy + Nav2, MPPIController 서브클래스 + pluginlib critic, osqp-eigen QP.
- 라이브러리 분리(class_loader 정합), 단일 entrapment 공유 상태.
- 단위테스트(164 checks), 런타임 플러그인 로드.

## 5. Experiments
- **5.1 설정:** 동일 Nav2 스택, 로봇(차동), 3-tier(BARN/DynaBARN/HuNavSim), 메트릭(평가 프로토콜 §4).
- **5.2 Baseline:** stock MPPI, DWB, RPP, TEB, always-on(DRPA류), CBF-only(Shield류).
- **5.3 Ablation:** A–F (평가 프로토콜 §3). 핵심 **E vs F**(독립 vs 조율).
- **5.4 결과:** 표 1–3 + 핵심 E/F 표 + U-trap 궤적·α/slack 시계열·compute-time.
- **5.5 통계:** McNemar/Mann–Whitney + Holm 보정, 효과크기.

## 6. Discussion / Limitations
- 조율의 이득과 실패모드(TTC 1-D 모델 낙관성, 좁은 공간 회전과다).
- 실시간성(per-call QP), 트래커 단순 CV의 한계.
- 일반화(다른 로봇/맵), sim-to-real.

## 7. Conclusion
- 조율이 escape와 safety의 상충을 해소하며 단일 Nav2-native 플러그인으로 배포 가능.
- 향후: 학습 예측, HOCBF, 사회적 비용 통합.

---

## 작성 전 필수 TODO (정직성·검증)
- [ ] **실험 실측** — 현재 모든 수치 미확보(설계만). 동작 시뮬 확보 후 표 채움.
- [x] **novelty 검증 완료(2026-06)** — 4-way 교집합 유지(Nav2 CBF-MPPI/escape+CBF 0건).
      `docs/research/2026-06_se-mppi-novelty-verification.md`. **반영사항:**
      ① abstract는 "CBF+MPPI"가 아니라 **Nav2 배포 + α 변조 조율**을 차별점으로(§Related/§Intro);
      ② DRPA-MPPI는 "convex-only"라 쓰지 말 것(비볼록 주장 있음), venue=CASE 2025로 인용;
      ③ α 변조 조율(d)이 가장 깨끗한 미점유 novelty — 강조.
- [ ] **조율 명제 증명** 형식화(forward-invariance ∀α>0) — 보충자료.
- [x] 그림 생성 — 아키텍처(`docs/papers/figures/architecture.mmd`, Mermaid 신규),
      U-trap·dynamic-CBF·α/slack은 prototype PNG 재사용. 매핑: `docs/papers/figures/README.md`.
      본문(draft §I·§IV-A·§VI-A)에 Fig.1–4 캡션 부착 완료.
- [x] DRPA "convex" 표현 제거(이 outline §2 라인) — venue=CASE 2025는 draft·서베이 정합 확인.
- [x] conformal 계보(Yang ACC'24·CRC-HRI'26·Lindemann RA-L'23·Dixit L4DC'23·UA-PCBF)
      를 draft §II에 **unclaimed 구현 디테일(Paper 2 이연)**로 추가 — 기여 아님.
- [ ] 인용·신뢰도 표기 정리(서베이 문서 기준; arXiv 미확인 항목 재확인).

> **주의:** 이 문서는 구조이며, 본문 수치·주장은 실험·검증 완료 후에만 확정한다.
> 자체보고/미확인 인용은 원문 확인 전까지 그대로 표기 유지.
