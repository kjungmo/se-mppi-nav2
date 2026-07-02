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
