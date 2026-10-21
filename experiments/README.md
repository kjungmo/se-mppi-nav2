# experiments — SE-MPPI 평가

평가 프로토콜: `docs/architecture/2026-06_se-mppi-evaluation-protocol.md`.

```
experiments/
  sim/         # 스모크/런타임 자산 (loopback params + drive 스크립트)
  configs/     # ablation 구성별 param override (아래 매핑)
  barn/        # BARN(정적) tier 연동·결과
  dynabarn/    # DynaBARN(동적) tier
  hunav/       # HuNavSim(소셜) tier
  baselines/   # stock MPPI/DWB/RPP/TEB 비교 설정
  analysis/    # run 로그 집계·통계·플롯
  prototype/   # 2D standalone 검증 (§VI-A 메커니즘, C++ 수식 1:1)
  benchmark2d/ # 랜덤화 2D 정량 벤치마크 (§VI-C: 생성기·러너·집계·그림)
  results_2d/  # benchmark2d 실측 산출물 (per-trial CSV, 요약 표, 그림)
```

> **재생성 주의(numpy 버전)**: `results_2d/`의 커밋 산출물(stats.json/tables.md/
> summary.csv)은 micromamba `ros2` env의 **numpy 2.4.6**으로 생성됐다. 다른 numpy
> 버전으로 `benchmark2d.report`를 재실행하면 논문에 인용되지 않은 일부 평균 필드가
> 마지막 ulp 수준에서 달라질 수 있다 (인용된 값 전부는 버전과 무관하게 정확히
> 재현됨 — 2026-07-03 검수에서 확인).

## SE-MPPI v2 미러 (선택 기능, 기본값은 논문 v1)

`benchmark2d`와 `prototype`에 v2 탈출 논리가 선택 기능으로 들어 있습니다.
기본값(`CONFIGS`, `--variant v1`)은 논문 v1 그대로이며, 커밋된 `results_2d/`를 비트 단위로 재현합니다.

- `prototype/se_mppi_v2.py`는 플러그인의 C++ 코드를 옮겨 쓴 파일입니다. 설계 문서를 옮긴 파일이 아닙니다.
  - 앵커 경로 진행: 1 Hz 직선 재계획을 흉내 냅니다. 후보 경로의 부호 있는 전진량을 사슬로 잇습니다 (`path_progress.cpp`).
  - 지평선 광선 선택: 광선 36개, 2.0 m, 이력 0.5 rad. 최소 여유 0.6 m와 회피 목록을 씁니다 (`opening_search.cpp`).
  - 고정 경유점 이동(hop): 도달·시간 초과·이동 감시·막힘 규칙으로 다시 고릅니다 (`manageEscapePath`).
  - 모든 롤아웃이 충돌하면 영 명령을 냅니다.
  - 탐지에서 탈출까지 한 주기 지연이 있습니다. 플러그인과 같습니다.
- 설계 문서에 없는 추가 규칙 네 개는 `escape_extras` 하나로 켜고 끕니다. 기본값은 켜짐입니다 (출하 알고리즘).
  - (a) 정지 자세·이전 hop 출발점·실패한 경유점으로 가는 광선을 뺍니다.
  - (b) 3 s 동안 0.15 m 미만으로 움직이면 다시 고릅니다.
  - (c) 0.2 m보다 가까운 경유점은 쓰지 않습니다.
  - (d) 지평선 후보에 0.6 m 최소 여유를 요구합니다.
- `escape_extras=False`는 설계 문서대로 만든 알고리즘입니다.
- 나머지 기능은 `configs.make_v2_configs(progress=..., gap_rule=..., hop=..., zero_on_all_collide=..., act_on_previous_cycle=..., escape_extras=...)`로 각각 끕니다.
- 변형(`--variant`)은 세 가지입니다.
  - `v1`: 논문 v1. 기본값입니다.
  - `v2`: 출하 알고리즘입니다 (`escape_extras` 켜짐).
  - `v2_advised`: 설계 문서대로 만든 알고리즘입니다 (`escape_extras` 꺼짐). 다른 설정은 `v2`와 같습니다.
- 실행: `python3 -m experiments.benchmark2d.runner --variant v2 --out <새 파일>`. `v2_advised`도 같은 명령에서 변형 이름만 바꿉니다.
  - `v2`와 `v2_advised`는 `results_2d/trials.csv`에 쓰지 못합니다.
  - 두 변형은 `v2_*` 열과 `variant` 열을 따로 씁니다. 한 파일에 두 변형을 섞으면 오류가 납니다.
  - 1,200회 재실행은 변형마다 파일을 따로 씁니다 (`SMOKE.md`의 "Full re-run").
- 스모크 결과와 전체 재실행 명령: `results_2d_v2_smoke/SMOKE.md`. 스모크는 결과가 아닙니다. 1,200회 전체 재실행(D-2)은 아직 돌리지 않았습니다.
- 시험: `benchmark2d/tests/test_v2_mirror.py`, `test_v2_cxx_parity.py`, `test_v1_unchanged.py`.
  - `test_v2_cxx_parity.py`는 C++ 코드를 직접 부릅니다 (`parity_shim`). 선택 규칙·광선 쏘기·진행 사슬을 비교합니다.
  - 실행 전에 `colcon build`로 `parity_shim`을 만드세요. `SE_PARITY_SHIM=<빌드>/nav2_se_controller/parity_shim`을 지정하세요.
  - `SE_REQUIRE_PARITY=1`이면 shim이 없을 때 시험이 실패합니다. 미러가 C++와 같다고 주장할 때는 꼭 켜세요.

## Ablation 구성 → 실제 파라미터 override

기준: `controller_server.FollowPath`. 베이스는
`src/nav2_se_controller/config/nav2_se_controller_params.yaml`.

| 구성 | override |
|---|---|
| **A. Stock MPPI** | `plugin: nav2_mppi_controller::MPPIController` (critics에서 EscapeCritic 제외) |
| **B. Escape always-on** | SE plugin, `se_enabled: false`(CBF 우회), EscapeCritic 등록 + `EscapeCritic.always_on: true` |
| **C. Escape detect-switch** | B와 동일 + gating 정상(기본) |
| **D. CBF only** | SE plugin, `se_enabled: true`, critics에서 EscapeCritic 제외 |
| **E. Escape+CBF 독립** | SE full, `se_alpha_escape == se_alpha_base` (조율 무력화) |
| **F. SE-MPPI 조율(full)** | SE full, `se_alpha_escape: 6.0`, `se_alpha_base: 2.0` |
| **F⁻. SE − gap** | F + `EscapeCritic.use_gap_search: false` |
| **F″. SE APF→proxy** | F + `EscapeCritic.use_apf: false` |
| **F‴. SE − 예측** | F + `se_obstacle_max_speed: 0.0` (CV 무력화→정적) |

> 구성(B)의 상시 비용(always-on)은 런타임 파라미터 `EscapeCritic.always_on`(기본 false)로 켠다 — 이미 구현·배선됨(`escape_critic.cpp:39`). 위 표는 요약이고, **실행 가능한 정식 오버레이 정의는 `experiments/configs/ablations.yaml`가 기준**이다 (A–F + F⁻/F″/F_static/F_no_conformal 모두 즉시 실행 가능).

> **주의 (출시 컨트롤러 기준):** 오버레이 `C_escape_detect`와 `F_minus_gap`은 출시된 C++ 컨트롤러에서 이름대로 동작하지 않습니다.
> 1. `C_escape_detect`는 `se_enabled: false`를 씁니다. 이 경우 컨트롤러가 매 주기 공유 갇힘 플래그를 지우고, 크리틱은 그 플래그를 따르므로 탈출 비용을 넣지 않습니다. 결과는 stock(A)과 같습니다.
> 2. `F_minus_gap`의 `EscapeCritic.use_gap_search: false`는 크리틱의 로그 탐침만 끕니다. 개구부 탐색은 컨트롤러 안에 있으므로 결과는 F와 같습니다.
> 3. 논문의 C와 F⁻는 Python 2D 미러에만 있습니다. Nav2에서 두 구성의 결과는 보고하지 않습니다.
> 4. 컨트롤러가 크리틱을 구동하는 동안 `EscapeCritic.progress_stall_window`와 `EscapeCritic.use_gap_search`는 탈출 동작에 영향을 주지 않습니다(`nav2_se_campaign2.yaml`은 동결 파일이라 주석을 달지 않았습니다).
> 5. 미러 docstring(`se_mppi_v2.py`)은 추가 규칙 (d)가 수평선 후보에만 걸린다고 적지만, 스위치는 대체(fallback) 임계값도 함께 끕니다. 논문 본문이 맞습니다. 미러 코드는 재현 출처를 지키려고 고치지 않았습니다.

## 메트릭 수집

각 run에서 로깅: 궤적(map→base), `/cmd_vel`, α, CBF slack/hard_safe,
min-distance-to-obstacle, time/collision/timeout. → `analysis/`에서 구성별
평균·CI·검정(McNemar/Mann–Whitney + Holm).

## 실행 (turnkey)

전체 스위트는 단일 진입점에서 실행한다(러너 상세는 `runner/README.md`):

```bash
# 오프라인 드라이런(ROS 불필요): 파이프라인 전체 점검 + 결과 JSON 기록
python3 -m experiments.runner.run_suite --launcher fake

# 실 벤치마크(ROS2/Gazebo GPU 워크스테이션):
python3 -m experiments.runner.run_suite --launcher ros
# 이후 집계·그림:
python3 -c "from experiments.analysis import plots; plots.plot_from_results('experiments/results')"
```

## 실행 제약

정량 평가는 **동작하는 시뮬** 필요. 현 RoboStack loopback은 구동 플러밍 결함(BACKLOG
M5b.x). 권장: Gazebo 물리(headless+xvfb) 또는 정상 ROS2 워크스테이션 + HuNavSim(ROS2).
워크스테이션 전용으로 남는 두 층: **(a)** 동적 tier의 GT 장애물 토픽 샘플링(운동학
메트릭은 odom 텔레메트리로 충족), **(b)** BARN/DynaBARN의 ROS1→ROS2 브리지(로더가
`scenario.meta['requires_ros1_bridge']`로 표시).
