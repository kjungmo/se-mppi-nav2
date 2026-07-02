# RUN.md — SE-MPPI 로컬 머신 실주행 가이드

이 레포의 `nav2_se_controller`(SafeEscapeController)를 로컬 머신에서 빌드하고
Gazebo 시뮬 → 실로봇 순으로 돌리는 절차. 명령은 복붙 가능. 막히면 §7 트러블슈팅.

> **권장 환경:** Ubuntu 24.04 + **GPU**(Gazebo 라이다 렌더가 빨라야 Nav2 활성화가
> 안정적 — 클라우드 컨테이너에서 막힌 지점). ROS2 **Jazzy**.
>
> **호스트가 Ubuntu 20.04/22.04여도 OK.** 이 레포는 **RoboStack(conda)** 기반이라
> 호스트 OS 버전과 무관하게 ROS2 Jazzy가 설치된다(conda 패키지는 glibc 2.17 타겟 →
> 20.04의 glibc 2.31에서 그대로 실행). §1-A를 그대로 쓰거나, GPU Gazebo를 깔끔히
> 원하면 §1-C(Docker)를 쓴다.

---

## 0. 한 줄 실행 (권장 — 위 §1~§3을 한 번에)

환경 설치(최초 1회 자동) → 빌드 → Gazebo+Nav2+RViz까지 한 명령:

```bash
bash scripts/run_sim.sh            # GUI + RViz: RViz에서 Nav2 Goal 클릭
bash scripts/run_sim.sh --drive    # + 초기 pose·목표 자동 발행(완주 확인)
bash scripts/run_sim.sh --headless # 화면 없이 (원격/CI)
```

### SE-MPPI 내부가 보이는 시각화 (RViz)
RViz에서 **Add → By topic** 으로 두 토픽을 추가:

| 토픽 | 내용 |
|---|---|
| `FollowPath/se_markers` | **SE-MPPI 내부 상태**: 주황 원반 = 동적장애물별 CBF 유효반경(**conformal q 만큼 팽창된 값 그대로**), 청록 선 = 1.5s 예측 horizon, 로봇 위 텍스트 = `a=α slack q esc`(탈출 중이면 빨강) |
| `FollowPath/trajectories` | stock MPPI 후보 궤적(샘플링) |

탈출이 일어나는 순간 **α가 2→6으로 바뀌고 텍스트가 빨개지는 것**, 동적장애물 원반이
예측 불확실성(q)에 따라 커졌다 작아지는 것을 실시간으로 볼 수 있다. 안 보이면:
컨트롤러 param `se_viz: true`(기본) 확인 + 구독해야 발행됨(RViz에 추가해야 켜짐).

---

## 1. ROS2 환경 준비 (셋 중 하나)

### 경로 A — RoboStack/conda (레포에서 검증된 방식, **OS 버전 무관: 20.04/22.04/24.04**)
```bash
git clone <your-fork>/se-mppi-nav2 && cd se-mppi-nav2
bash scripts/setup_ros2_env.sh          # micromamba + ROS2 Jazzy + Nav2 + Gazebo (멱등)
# 활성화: micromamba는 shell hook을 eval해야 '명령'이 된다(설치만으론 PATH에 없음).
# 스크립트가 끝에 정확한 경로로 안내해주니 그대로 복붙. 로컬 비-root 기본:
export MAMBA_ROOT_PREFIX=$HOME/micromamba
eval "$($HOME/.local/bin/micromamba shell hook -s bash)"
micromamba activate ros2                 # 이후 셸마다 위 2줄 반복(또는 shell init로 영구등록)
```
> root(클라우드/Docker)면 경로가 `/root/micromamba`·`/opt/micromamba`. `micromamba: command
> not found`가 나면 **shell hook eval을 안 한 것** — 위 `eval ...` 줄을 먼저 실행.
> 영구 등록: `<micromamba_bin> shell init -s bash -r <root_prefix> && source ~/.bashrc`.
> **GPU 렌더 주의(특히 20.04):** RoboStack을 호스트에 직접 깔면 호스트 GPU 드라이버와
> conda `libGL` 충돌이 날 수 있다(NVIDIA 잦음). 증상: `gz sim`이 렌더 못 함/느림.
> 해결: `export MESA_GL_VERSION_OVERRIDE=3.3`(소프트웨어 폴백) 시도 → 안 되면 §1-C
> Docker. 충돌이 없으면 호스트 GPU로 하드웨어 렌더가 그냥 된다.

### 경로 C — Docker + NVIDIA (호스트 20.04 + NVIDIA GPU에 **권장**)
호스트 사전: `nvidia-smi` 정상(드라이버 OK) + Docker 설치.
```bash
# 1) nvidia-container-toolkit (호스트 20.04, 1회)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
docker run --rm --gpus all ubuntu:24.04 nvidia-smi   # GPU 표 나오면 성공

# 2) 컨테이너 실행 (레포 루트에서)
xhost +local:docker                                   # RViz 볼 때만
docker run -it --rm --gpus all --net=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $PWD:/ws -w /ws ubuntu:24.04 bash
# ── 컨테이너 안 ──
apt update && apt install -y git curl bzip2 procps
bash scripts/setup_ros2_env.sh    # Docker는 root -> /root/micromamba, /opt/micromamba
export MAMBA_ROOT_PREFIX=/opt/micromamba
eval "$(/root/micromamba shell hook -s bash)" && micromamba activate ros2
# 이후 §2 빌드, §3 실행. NVIDIA_DRIVER_CAPABILITIES=all 이 graphics/display까지 노출 →
# gz가 라이다를 하드웨어 렌더 → 컨테이너에서 막혔던 "라이다 느림→Nav2 타임아웃"이 해결됨.
```
> 먼저 §3 launch를 `headless:=True use_rviz:=False`로 띄우고 **(b) 스크립트**로 완주
> 확인(X11 불필요) → 되면 `headless:=False use_rviz:=True` + **(a) RViz**로 시각화.
> 24.04 유저스페이스 + 호스트 GPU 패스스루라, 호스트가 20.04여도 드라이버 충돌 없이
> 하드웨어 렌더가 깔끔히 잡힌다. (RoboStack 대신 `ros:jazzy` 이미지 + apt nav2도
> 가능하나 `osqp-eigen` apt 패키지가 배포마다 달라 RoboStack이 가장 확실.)

### 경로 B — 네이티브 apt (**Ubuntu 24.04 전용** — Jazzy가 24.04만 지원)
> 호스트가 20.04면 이 경로는 불가(Jazzy 미지원). A 또는 C를 쓸 것.
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-nav2-loopback-sim \
  python3-colcon-common-extensions
sudo apt install -y ros-jazzy-osqp-vendor || true   # OsqpEigen 미제공 시 소스 빌드
source /opt/ros/jazzy/setup.bash
```
> 네이티브에서 `find_package(OsqpEigen)` 가 안 잡히면 소스 빌드하거나 **경로 A** 사용.

---

## 2. 빌드
```bash
cd se-mppi-nav2
colcon build --packages-select nav2_se_controller
source install/setup.bash
colcon test --packages-select nav2_se_controller   # (선택) 164 checks green 확인
```

---

## 3. Gazebo 시뮬 실주행 (turtlebot3 + SafeEscapeController)

```bash
export TURTLEBOT3_MODEL=waffle
# (네이티브) tb3 sim 모델 경로
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix nav2_minimal_tb3_sim)/share/nav2_minimal_tb3_sim/models:${GZ_SIM_RESOURCE_PATH:-}"

ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=False use_rviz:=True \
  params_file:=$PWD/experiments/sim/nav2_se_loopback.yaml
```
- `params_file`은 **FollowPath를 `nav2_se_controller::SafeEscapeController`로 교체**한
  Nav2 설정이다(이 파일이 우리 컨트롤러를 쓰게 함).
- GPU 머신이면 라이다가 빠르게 렌더돼 AMCL/costmap이 정상 활성화된다.

### 목표 보내기 (둘 중 하나)
**(a) RViz로 (가장 쉬움):** RViz에서 **2D Pose Estimate**로 로봇 스폰 위치
(`-2.0, -0.5`) 근처를 찍어 AMCL 초기화 → **Nav2 Goal**로 목표 클릭.

**(b) 스크립트로 (자동):**
```bash
export SE_START_X=-2.0 SE_START_Y=-0.5 SE_GOAL_X=0.9 SE_GOAL_Y=-2.25 SE_LOCALIZER=amcl
python3 experiments/sim/smoke_drive.py    # 초기 pose 발행 + 목표 + dist 로깅
# SMOKE_RESULT=TaskResult.SUCCEEDED 면 완주
```
좌표는 tb3_sandbox의 자유공간 기준(BACKLOG에 warehouse 좌표도 있음).

### 컨트롤러가 실제로 쓰이는지 확인
```bash
ros2 topic echo /cmd_vel_nav --once       # 컨트롤러 출력(0이 아니면 동작)
# 로그에 "Created controller : FollowPath of type nav2_se_controller::SafeEscapeController"
```

---

## 4. 동작/조율 관찰 (escape·CBF가 실제로 작동하는지)
SE-MPPI가 동작하려면 **EscapeCritic도 MPPI critics 리스트에 등록**되어야 한다.
`experiments/sim/nav2_se_loopback.yaml`의 `FollowPath.critics`에 `"EscapeCritic"`을
추가하고 아래 블록을 둔다(샘플은 `src/nav2_se_controller/config/nav2_se_controller_params.yaml`):
```yaml
      critics: ["ConstraintCritic","CostCritic","GoalCritic","GoalAngleCritic",
                "PathAlignCritic","PathFollowCritic","PathAngleCritic",
                "PreferForwardCritic","EscapeCritic"]
      EscapeCritic:
        enabled: true
        use_apf: true
        use_gap_search: true
      # 컨트롤러(조율/CBF) 파라미터
      se_enabled: true
      se_alpha_base: 2.0
      se_alpha_escape: 6.0
      se_ttc_override_threshold: 1.5
```
좁은/U자 환경에 로봇을 두고 목표를 주면 stock MPPI는 갇히고 SE-MPPI는 탈출하는 걸
관찰할 수 있다(2D 검증 그림과 동일한 거동).

---

## 5. 실로봇 적용
1. 로봇의 Nav2 `params.yaml`에서 `controller_server.FollowPath.plugin`을
   `nav2_se_controller::SafeEscapeController`로 변경.
2. §4의 `critics`/`se_*`/`EscapeCritic` 블록을 머지.
3. `se_cbf_lookahead`, `se_alpha_*`, `se_obstacle_*`는 로봇 크기·속도에 맞게 튜닝.
4. **안전 우선:** `collision_monitor`는 켜둘 것(스모크에선 좁은 맵 때문에 껐었음).
   CBF 필터는 동적장애물용 추가 안전층이지 collision_monitor 대체가 아니다.
5. `nav2_se_controller`를 로봇 워크스페이스 `src/`에 두고 colcon 빌드.

---

## 6. 2D 알고리즘 검증 그림 재생성 (시뮬 불필요)
```bash
pip install numpy matplotlib osqp scipy
cd experiments/prototype && python3 run_validation.py
# -> figures/{utrap_escape,dynamic_cbf,coordination}.png + 메트릭 표
```
C++ 알고리즘과 패리티(자세한 건 `experiments/prototype/README.md`).

---

## 7. 트러블슈팅 (이번 세션에서 실제로 겪은 것들)

| 증상 | 원인 / 해결 |
|---|---|
| **`CONDA_BUILD: unbound variable`** (run_sim.sh 실행 중) | conda/RoboStack 활성화 스크립트가 미설정 변수를 참조하는데 셸이 `set -u`(nounset)였음. **최신 main에서 수정됨**(활성화 구간을 `set +u`로 감쌈) — `git pull origin main` 후 재실행. 수동 활성화 시에도 동일: `micromamba activate` 전에 `set +u` |
| **`micromamba: command not found`** | 설치만으론 PATH에 없음 — **shell hook을 eval해야** 명령이 됨: `eval "$(<micromamba_bin> shell hook -s bash)"` 먼저 실행(§1-A). 비-root면 bin은 `$HOME/.local/bin/micromamba`, root/Docker면 `/root/micromamba` |
| `controller_server`가 플러그인 못 찾음 | `source install/setup.bash` 안 함 / 빌드 실패. `ros2 plugin`은 없으니 로그의 "Created controller …" 확인 |
| `EscapeCritic` "no factory exists" | (이미 수정됨) 라이브러리 분리 필요 — 최신 레포면 OK |
| **로봇이 안 움직임 + dist 고정** | **loopback 시뮬의 odom/구동 플러밍 결함**(stock 데모도 동일). → **Gazebo 물리 시뮬 사용**(loopback 쓰지 말 것) |
| **`global_costmap: Failed to activate` / nav "Failed to bring up"** | 라이다가 느려 AMCL TF가 제때 안 섬(**GPU 없는 헤드리스**). → **GPU 머신** 또는 `transform_tolerance`/lifecycle `bond_timeout` 상향 |
| `amcl: cannot publish transform` / `Failed to transform initial pose` | 초기 pose 미설정. RViz 2D Pose Estimate 또는 smoke_drive가 발행(여러 번) |
| 좁은 방에서 로봇이 거의 정지 | `collision_monitor` FootprintApproach가 과도 throttle. 좁은 데모 맵 한정 — 스모크 params는 이미 비활성. 실환경은 켜둘 것 |
| planner "Failed to create plan" | 시작/목표가 **장애물·미탐색 셀**. 자유공간 좌표 사용(맵 중심이 벽일 수 있음) |
| `cmd_vel` 타입 불일치 | Jazzy는 Twist, Kilted는 TwistStamped 기본. 시뮬·로봇·Nav2 버전 일치 확인 |

---

## 참고
- 컨트롤러 코드: `src/nav2_se_controller/`
- 설계·평가·논문: `docs/architecture/`, `docs/papers/`, `docs/research/`
- 남은 일/알려진 한계: `docs/architecture/BACKLOG.md`
- 스모크 자산: `experiments/sim/` · 알고리즘 검증: `experiments/prototype/`
