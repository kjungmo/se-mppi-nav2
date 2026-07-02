# L11 평가 하니스 — 자동 벤치마크 러너 설계

> **작성일:** 2026-06-10 · **상태:** 설계 (구현 대기) · **층:** L11(시뮬·평가)
> **목표:** SE-MPPI 논문의 *pending* 정량표(성공률·충돌률·시간·여유·compute)를 **재현가능하게 자동 측정**하는 러너.
> **선행:** `docs/architecture/2026-06_se-mppi-evaluation-protocol.md`(메트릭·ablation·통계 정의), 라이브 런 핸드오프(런타임 함정).
> **원칙:** 프로토콜은 이미 설계됨 — 이 문서는 *그것을 돌리는 인프라*만 정의한다.

---

## 1. 왜 별도 모듈인가

평가 프로토콜(baseline·ablation A–F·메트릭·통계)은 정의돼 있으나, 그것을 **수백 개 시나리오 × 여러 컨트롤러 × 반복**으로 돌리고 결과를 모으는 **실행 인프라**가 없다. 라이브 런에서 드러난 운영 함정(프로세스/clock 충돌, AMCL 텔레포트 오해, 컨트롤러 abort)을 *자동으로* 처리하지 못하면 N00개 시도를 사람이 돌릴 수 없다. → 러너가 곧 논문 표를 만든다.

기존 `experiments/` 골격(빈 디렉터리 `barn/ dynabarn/ hunav/ baselines/ configs/ analysis/`)을 채운다.

---

## 2. 아키텍처

```
experiments/
  configs/        # 컨트롤러×ablation YAML (params 오버레이) + 러너 설정
  barn/ dynabarn/ hunav/   # 시나리오 정의(맵·world·start/goal·동적에이전트)
  runner/         # ← 신규: 러너 코드
    run_suite.py        # 스위트 오케스트레이터 (시나리오×config×seed 루프)
    trial.py            # 단일 시도: launch→goal→metrics→teardown
    scenario.py         # 시나리오 로딩(맵/world/start/goal) + pick_goal 재사용
    metrics.py          # /odom·/tf·충돌·시간 구독→메트릭 집계
    cleanup.py          # ★프로세스/SHM 클린 재시작(핸드오프 §2-A 코드화)
  results/        # trial별 raw JSON + 집계 CSV (gitignore 큰 로그)
  analysis/       # CSV→표·그림·통계(McNemar/Mann-Whitney+Holm)
```

### 단일 시도 상태기계 (`trial.py`)
```
[CLEANUP] 프로세스·/dev/shm·daemon 정리 (실패해도 진행)
   → [LAUNCH] nav2 + sim + controller(config) , 로그 /results/<id>.log
   → [WAIT_ACTIVE] "Managed nodes are active" & 컨트롤러 로드 확인 (타임아웃 → FAIL:setup)
   → [DRIVE] goal 전송, 메트릭 스트림 기록, watchdog(시간초과/진동/abort)
   → [CLASSIFY] SUCCESS / COLLISION / TIMEOUT / STUCK / SETUP_FAIL
   → [TEARDOWN] 종료, raw JSON 기록
```
각 시도는 **독립 프로세스**(서브프로세스)로 격리 — 한 시도의 크래시가 스위트를 죽이지 않게.

---

## 3. 메트릭 수집 (`metrics.py`)

| 메트릭 | 소스 | 정의 |
|---|---|---|
| success | bt_navigator 결과 + 골 도달 | goal tol 내 도달 & 무충돌 |
| collision | `/tf` 로봇pose vs 장애물 GT(또는 footprint∩lethal) | min-clearance < 0 발생 |
| time-to-goal | 시뮬 clock | goal 수락→도달 |
| path length | `/odom` 적분 | 실제 이동거리 |
| min clearance | 로봇 vs 최근접 장애물(GT) | 시도 중 최소 |
| compute/cycle | controller_server 로그 "loop rate" 또는 별도 프로브 | per-call ms (실시간성) |
| SPL | success·optimal/actual | 표준 내비 효율 |

- **동적장애물 GT**: HuNavSim/DynaBARN가 에이전트 GT pose를 토픽으로 제공 → 충돌·clearance는 GT 기반(코스트맵 추정 아님)으로 정확히.
- **시뮬 clock 기준** 측정(실시간 배속 무관).

---

## 4. 시나리오 tier (프로토콜 §5 대응)

| tier | 소스 | 변동축 | 비고 |
|---|---|---|---|
| BARN | 300 procedural worlds | 정적 난이도 | 로컬미니마·좁은 통로 — escape 핵심 |
| DynaBARN | BARN + 동적에이전트 | 동적 밀도·속도 | CBF·조율 핵심 |
| HuNavSim | 사회적 보행자 모델 | 군중·사회규범 | 동적 예측(L2) 평가에도 재사용 |

각 시나리오는 `scenario.py`가 `{map, world, start, goal, agents}`로 로딩. start/goal은 `pick_goal.py`의 도달가능·벽회피 산출을 재사용(임의 goal로 인한 setup 실패 방지).

---

## 5. config 매트릭스 (ablation A–F)

`configs/`에 params 오버레이로 표현(전체 yaml 복제 대신 base + diff):
- **A** stock MPPI · **B** escape always-on(DRPA류) · **C** escape detect-switch only
- **D** CBF only · **E** escape+CBF 독립 · **F** escape+CBF 조율(=SE-MPPI 전체)
- baselines: DWB·RPP·TEB (별도 plugin)

핵심 대조 **E vs F**가 조율 기여(C2)를 격리. 각 config × 각 시나리오 × `--seeds N`.

---

## 6. 출력·분석 (`analysis/`)

- trial → `results/<tier>/<config>/<scenario>_<seed>.json`
- 집계 → `results/summary.csv` (모든 메트릭 long-format)
- `analysis/aggregate.py` → 논문 표(성공률±CI, 충돌률, 시간/길이/clearance 중앙값±IQR)
- `analysis/stats.py` → McNemar(성공, 쌍대) + Mann–Whitney(연속) + Holm 보정 + 효과크기
- `analysis/plots.py` → U-trap 궤적, α/slack 시계열, compute 분포

---

## 7. 운영 견고성 (라이브 런 교훈의 코드화)

핸드오프 §6 함정을 러너가 자동 처리:
- **클린 재시작**(§2-A)을 `cleanup.py`로 매 시도 전 강제 — TF/clock 충돌 원천 차단.
- **AMCL 모드**에선 initialpose 재발행 생략(이미 smoke_drive에 반영) — 또는 시나리오가 GT teleport 제공 시 그것 사용.
- **setup 실패**(컨트롤러 미로드/라이프사이클 abort)는 SUCCESS/FAIL과 **구분**해 기록(분모 오염 방지) + 자동 1회 재시도.
- **결정성**: seed 고정, world·start·goal 로깅 → 재현가능.

---

## 8. 마일스톤

| M | 내용 | 산출물 | 상태 |
|---|---|---|---|
| H-1 | `cleanup.py`+`trial.py`: 단일 시나리오 상태기계(SUCCESS/COLLISION/TIMEOUT/STUCK/SETUP_FAIL 분류, setup 1회 재시도) | 1-시도 러너 | **구현·테스트 완료** (오프라인, `FakeLauncher`) |
| H-2 | `metrics.py`: success·collision·time·path·min-clearance·SPL·BARN·compute·smoothness·oscillation | 메트릭 검증 | **구현·테스트 완료** |
| H-3 | `run_suite.py`: config×scenario×seed 루프 + 시도별 격리 + resume | 스위트 러너 | **구현·테스트 완료** |
| H-4 | BARN 로더 + 소규모 파일럿 | 첫 정량표(소표본) | 로더·검증(`scenario.py`/`gridmap.py`) 완료, **실측은 GPU 필요**(§9) |
| H-5 | DynaBARN/HuNavSim 로더 + 풀 스위트 | 논문 §실험 수치 | 미착수(시나리오 스키마·`agents` 필드 준비됨) |
| H-6 | `analysis/`: 표·통계·그림 자동화 | 카메라레디 표/그림 | **aggregate+stats 구현·테스트 완료**, `plots.py` 미착수 |

> **구현 현황(2026-06):** 순수 코어(config 해소·메트릭·분류·도달성·통계·집계·오케스트레이션)는
> ROS 없이 **51개 단위테스트 통과**. 라이브 런처(`RosLauncher`)만 GPU 워크스테이션 대기.
> `python3 -m pytest experiments/runner/tests experiments/analysis/tests`로 재현.
> ablation 9구성은 **실제 base yaml**(`nav2_se_loopback.yaml`)에서 해소 검증됨 — 프로토콜 §3과 1:1.

---

## 9. 정직한 제약

- **GPU 필요**: 동적·사회 시뮬은 하드웨어 렌더링 필요(클라우드 GPU 부재로 막힘) → **로컬 NVIDIA 워크스테이션에서 실행**. 러너는 OS-독립(RoboStack) 빌드 위에서 돈다.
- **계산량**: tier×config×seed = 수천 시도. 병렬화(여러 시도 동시, 포트/도메인ID 분리) 설계는 H-3에서.
- 이 러너가 채우기 전까지 논문 §VI-C 표 셀은 *pending* 유지(수치 날조 금지).

> 이 모듈이 완성되면 SE-MPPI 논문의 마지막 한 단계(*pending* 표)가 닫힌다.
