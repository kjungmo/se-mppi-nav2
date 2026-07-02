# SESSION HANDOFF — 로컬 세션에서 이어가기

> **이 문서 하나로 새 세션에서 이어갈 수 있게 정리.** 마지막 갱신: 2026-06-12.
> 모든 작업은 **`main`** 에 있음 (2026-07-03 히스토리를 단일 커밋으로 재구성, 단일 브랜치).
> 새 세션 시작 시: **이 파일 먼저 읽기** → §1(지금 할 일) → 필요하면 §3~§6.

---

## ⭐ 2026-07-02 갱신 — 논문 1 완성 (이 섹션이 아래 §0–§1보다 최신)

**논문 1이 제출 가능한 상태로 완성됐다**: `docs/papers/latex/main.pdf` (IEEEtran, 10쪽,
tectonic 클린 컴파일). 2026-07-03 저널식 다회전 검수(paper-repo-audit: 3렌즈 ×
2라운드, 라운드2 전원 accept 수렴)까지 통과 — 판정·기각 사유는 PR #15 코멘트 참조.
넘버가드 `scripts/check_paper_numbers.py`가 인용 수치↔아티팩트 추적을 상시 단언한다.
이 세션(들)에서 한 일:

- **§VI-C 정량**: 랜덤화 2D 벤치마크 실측 1,200 트라이얼 (`experiments/benchmark2d/`,
  4계열 × 시드50 × 6 config, paired McNemar/Mann–Whitney+Holm). 탈출 효과
  0%→88–90% (p≈10⁻⁹); **E vs F(조율) = null (전 계열 p=1)** — 논문에 공개 보고,
  조율은 "형식 보장이 있는 메커니즘"으로만 주장. 데이터: `experiments/results_2d/`,
  요약: `docs/papers/2026_2d-benchmark-results.md`.
- **레퍼런스 전수 검증**: 30건 전부 arXiv/출판사 원문 대조 (2패스 적대 검증).
  17 확인 / 13 교정, venue 강등 9건. `docs/papers/references.bib` +
  `reference-verification-report.md`가 진실의 원천.
- **라이브 트랙**: escape 발동(감지→비용 주입) 라이브 로그 검증 완료. **xtensor ABI
  불일치 크래시 발견·수정** (`74d5fe7` — 설치된 MPPI .so는 XSIMD+AVX2, 플러그인은
  std::allocator → 첫 라이브 주입에서 SIGSEGV; per-target 플래그로 해소, 254 테스트
  그린). 라이브 정량 A/B는 이 WSL2 호스트의 아티팩트(RTF 0.34, TF 스미어→팬텀
  코스트맵)로 보류 — **GPU 워크스테이션에서 재시도가 다음 관문** (커스텀 BT·타임아웃
  준비됨: `experiments/se_mppi_utrap/behavior_trees/`, 진단 로그 `experiments/results_pilot/`).
- **적대적 리뷰 반영**: 5렌즈 리뷰 18건(major 9) 전부 수정 (`00f17c3`).

**남은 것 (GPU 워크스테이션)**: ① S1 U-trap 라이브 A/B (팬텀 코스트맵이 RTF≈1에서
소멸하는지 확인) ② 풀스케일 3D 물리 벤치마크(BARN류) ③ 투고 전 최종 교정.

---

## 0. 한 줄 요약

SE-MPPI(단일로봇, 논문1)는 **코드·테스트·라이브로드 완료**. 확장 3트랙(L2 예측,
L9 멀티로봇, L10 FM)의 **컨테이너에서 가능한 코드는 전부 구현·단위검증 완료**
(`colcon test` 245개 통과). **이제 남은 건 전부 워크스테이션(GPU) 작업**이고,
현재 **첫 관문 = 라이브 Gazebo 시뮬을 띄우는 것**(`scripts/run_sim.sh`).

---

## 1. 지금 당장 할 일 — 라이브 시뮬 띄우기

```bash
cd ~/ws/se-mppi-nav2          # (로컬 워크스테이션 경로)
git pull origin main
bash scripts/run_sim.sh --drive    # 환경+extras 보강 → 빌드 → Gazebo+Nav2+RViz
```

### 진행 상황 (run_sim 디버깅)
- ✅ **해결됨 `CONDA_BUILD: unbound variable`** — conda 활성화가 `set -u`와 비호환.
  활성화 구간을 `set +u`로 감쌈 (PR #13).
- ✅ **해결됨 `package 'rviz2' not found` + sdf 생성 실패** — 두 원인:
  1. env가 `ros-base`로 깔려 **rviz2/ros-gz 누락** → `setup_ros2_env.sh`에
     `ros-jazzy-rviz2`, `ros-jazzy-ros-gz` 추가 + 기존 env 자동 보강(§2b).
     `run_sim.sh`가 매 실행마다 setup을 호출해 extras를 보장.
  2. **ROS1 noetic 경로 혼입**(`.bashrc`가 noetic을 source) → `PYTHONPATH`/
     `CMAKE_PREFIX_PATH` 오염 → launch의 sdf 생성 python이 깨짐. `run_sim.sh`가
     활성화 전에 noetic/melodic 경로를 PATH류에서 제거하도록 수정.
  > **위 수정들이 이 핸드오프 직전 커밋에 포함됨. `git pull` 후 재실행이 첫 테스트.**

### 다음에 막히면 볼 것 (예상 순서)
1. **`git pull` 후 첫 실행**: extras 설치(rviz2/ros-gz, 수 분) → 빌드 → launch.
   RViz 창 + Gazebo 창이 뜨고 로봇이 목표로 가면 성공.
2. **GPU 렌더 안 잡힘** → 라이다 느림 → `global_costmap: Failed to activate` /
   nav "Failed to bring up". 대응: `RUN.md §7`. 워크스테이션 GPU면 보통 OK.
   소프트웨어 폴백: `export MESA_GL_VERSION_OVERRIDE=3.3` 후 재시도.
3. **`.bashrc`의 noetic source 제거 권장** — run_sim이 런타임에 걷어내지만,
   근본적으론 `.bashrc`에서 `/opt/ros/noetic/setup.bash` 줄을 주석 처리하는 게 깔끔.
4. **에러가 나면 그 터미널 출력을 그대로 공유** → 스크립트/문서에 반영.

### 시각화 (RViz)
RViz에서 **Add → By topic**:
- `FollowPath/se_markers` — SE-MPPI 내부: 주황 원반=CBF 유효반경(**conformal q 팽창 포함**),
  청록 선=예측 horizon, 로봇 위 텍스트=`a=α slack q esc`(탈출 중 빨강).
- `FollowPath/trajectories` — MPPI 후보 궤적.

---

## 2. 빌드·테스트 치트시트

```bash
# 환경 활성화 (인터랙티브 셸. 스크립트 안이면 micromamba activate 전 'set +u')
export MAMBA_ROOT_PREFIX=$HOME/micromamba        # 로컬 비-root 기준
eval "$($HOME/.local/bin/micromamba shell hook -s bash)"
micromamba activate ros2

# 빌드 + C++ 단위테스트 (245개)
colcon build --packages-select nav2_se_controller
colcon test --packages-select nav2_se_controller && colcon test-result --verbose

# 파이썬 (시뮬 불필요)
python3 -m pytest experiments/runner/tests experiments/analysis/tests -q   # L11 하니스 51개
python3 experiments/prototype/run_validation.py            # 단일로봇 2D 검증 그림
python3 experiments/prototype/run_multirobot_validation.py # L9 멀티로봇 2D 검증
python3 experiments/prototype/run_fm_shield_validation.py  # L10 FM 거부권 검증
python3 experiments/prediction/ade_fde_eval.py             # L2 예측 ADE/FDE
```

---

## 3. 구현 현황 (트랙별)

| 트랙 | 마일스톤 | 상태 | 핵심 코드 | 검증 |
|---|---|---|---|---|
| **L6/L7 SE-MPPI** (논문1) | — | 코드·라이브로드 완료, 논문 초안 | `safe_escape_controller`, `cbf_safety_filter`, `escape_safety_coordinator` | 단위 + 라이브 cmd_vel |
| **L11 평가 하니스** | H1–H3·H6 | **코어 완료**, 라이브 런처(`RosLauncher`)만 GPU 대기 | `experiments/runner/`, `experiments/analysis/` | 51 pytest |
| **L2 SE-Predict** | N1 | DOGM 정적/동적 분류(벽-freeze 차단) | `static_occupancy_filter` | gtest |
| | N2(고전) | 영속트랙 + LS CV/CVCA 예측기 + horizon | `trajectory_predictor`, tracker v2 | gtest + ADE/FDE |
| | N3 | conformal q → CBF 시변반경 + q-신뢰 escape 게이트 | `conformal_calibrator` | gtest(커버리지 수렴) |
| | N2(학습)·N4 | **미착수 — GPU/데이터 필요** | — | — |
| **L9 Multi-SE-MPPI** | N1 | 2D proto: deadlock 해소 vs 교착 | `experiments/prototype/multi_se_proto.py` | 그림 |
| | N2 | 책임분배 CBF(λ) + `MultiRobotCoordinator`(이웃식별·우선권) 플러그인 통합 | `cbf_safety_filter`(λ), `multi_robot_coordinator` | gtest(11) |
| | N3 | **미착수 — 멀티로봇 fleet sim 필요** | — | — |
| **L10 FM-Shield** | N1 | FM 제안 인터페이스 + CBF 거부권(적대 제안 무충돌) | `experiments/prototype/fm_shield_proto.py` | 그림+게이트 |
| | N2·N4 | **미착수 — 실모델(ViNT류)·GPU 필요** | — | — |
| **N0 선행검증** | — | 1차 완료(스니펫 기반) — 부품은 기성, **차별점=조율** | `docs/research/2026-06_extension-tracks-n0-verification.md` | ☐ 카메라레디 전 PDF 정독 |

`se_*` 파라미터는 **기본값이 기존 거동 유지**(N2/N3/멀티로봇은 opt-in 또는 무해 기본).
샘플: `src/nav2_se_controller/config/nav2_se_controller_params.yaml`.

---

## 4. 다음 마일스톤 (전부 워크스테이션/GPU)

우선순위 순:
1. **라이브 시뮬 띄우기** (§1) — 모든 정량 평가의 전제.
2. **L11 `RosLauncher` 구현 + BARN 파일럿(H-4)** — `experiments/runner/trial.py`의
   `RosLauncher` 스텁에 launch/wait/drive/teardown 배선 → 논문1의 *pending* 표를 닫음.
   하니스 골격·메트릭·통계·집계는 이미 완성.
3. **L2 N4** — DynaBARN/HuNavSim에서 SE-MPPI(CV) vs SE-Predict A/B (L11 재사용).
4. **L9 N3** — 다중로봇 Nav2 launch(네임스페이스) + 프로토 시나리오 sim 검증.
5. **L10 N2** — 경량 내비 FM/정책을 `fm_shield_proto`의 Proposal 인터페이스에 연결.
6. **N0 마무리** — 일반망에서 arXiv PDF 4건 정독(컨테이너는 403). 목록은 N0 문서 §5.

---

## 5. 파일 지도

```
src/nav2_se_controller/           # ROS2 Nav2 플러그인 (C++)
  include/.../*.hpp, src/*.cpp     # 컨트롤러 + 코어 모듈 (위 §3 표 참조)
  config/nav2_se_controller_params.yaml   # 전체 se_* 파라미터 샘플(주석 설명)
  test/                            # gtest 245개
experiments/
  runner/  analysis/               # L11 하니스 (pytest 51)
  prototype/                       # 2D 검증: se_mppi/multi_se/fm_shield_proto + run_*
  prediction/ade_fde_eval.py       # L2 예측 평가
  sim/                             # nav2_se_loopback.yaml(컨트롤러 교체), smoke_drive.py
  configs/ablations.yaml           # L11 ablation 매트릭스(A–F + F-variants, 11구성)
docs/
  architecture/                    # 설계: SE-MPPI, SE-Predict, 평가 프로토콜, 하니스, 전체 플랫폼
  research/                        # 문제정의(L2/L9/L10), SOTA 서베이, novelty 검증, N0
  papers/                          # 논문1 초안·아웃라인
scripts/
  setup_ros2_env.sh                # RoboStack 환경 설치(멱등) + 라이브 extras
  run_sim.sh                       # 한 줄 실행: 환경→빌드→Gazebo+Nav2+RViz
RUN.md                             # 워크스테이션 실행 가이드(§0 한줄, §7 트러블슈팅)
docs/architecture/BACKLOG.md       # 미뤄둔 항목
```

---

## 6. 환경·함정 (RoboStack 기준)

- **OS 무관**: RoboStack(conda)이라 Ubuntu 20.04/22.04/24.04 어디서나 Jazzy 동작.
- **`set -u` 함정**: conda/ROS 활성화·소싱 스크립트는 nounset 비호환 → 스크립트에서
  `micromamba activate`/`source setup.bash` 전후로 `set +u`/`set -u`.
- **ROS1 혼입**: `.bashrc`에 noetic이 source돼 있으면 경로 오염. `run_sim.sh`가
  런타임에 제거하지만, `.bashrc`에서 빼는 게 근본 해결.
- **GPU**: 라이브 Gazebo는 하드웨어 렌더 필요(라이다). GPU 없으면 Nav2 활성화 타임아웃
  — 컨테이너에서 막혔던 지점. 워크스테이션 GPU가 그래서 필요.
- **loopback 시뮬 쓰지 말 것**: odom/구동 플러밍 결함(stock 데모도 동일). Gazebo 물리 시뮬 사용.
- **빌드 캐시**: `build/ install/ log/`는 .gitignore. `run_sim.sh`는 소스가 바뀌었을 때만 재빌드.

---

## 7. 작업 규칙

- 브랜치 정책: 원격은 `main` 단일 브랜치. 작업 브랜치는 머지 후 삭제.
- 인터페이스는 **설치된 Jazzy 헤더 기준**(예: critic costs는 xtensor — main 브랜치 Eigen과 다름).
- 리서치 수치·인용은 **신뢰도 표기 유지**(자체보고/미확인/N0 1차검증).
- 커밋 시 `colcon test`(C++) / `pytest`(파이썬) 통과 확인.

> 막히는 지점이 생기면 **터미널 출력 그대로 공유** → 스크립트/문서 즉시 보강.
