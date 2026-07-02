# SE-MPPI — Gazebo 라이브 런 핸드오프 (2026-06-10)

> 이 문서 하나로 다른 세션/로컬 PC에서 작업을 그대로 이어갈 수 있도록 정리한 핸드오프.
> 브랜치: `claude/fervent-newton-lbo96` (원격 = source of truth). 최신 커밋 `e2c825b`.
> 환경: Ubuntu 20.04 + NVIDIA, RoboStack(conda-forge) ROS2 **Jazzy**, 워크스페이스 `~/ws/se-mppi-nav2`.

---

## 0. TL;DR — 지금 어디까지 왔나

- ✅ **SE-MPPI 컨트롤러가 실제 ROS2 Jazzy + Nav2 + Gazebo(tb3_sandbox) 스택에 로드·활성화되고 유효 `cmd_vel`을 생성** — 클라우드에서 GPU로 막혔던 마일스톤 ④("실 Gazebo 구동")를 로컬에서 통과.
- ✅ 라이브 런이 아니면 못 잡았을 **진짜 버그 4건**을 발견·수정 (아래 §3).
- ✅ **결정적 통합 버그 수정:** `EscapeCritic`이 잘못된 네임스페이스(`nav2_se_controller`)에 있어 MPPI critics 목록으로는 **한 번도 로드된 적이 없었음**. `mppi::critics`로 이동 → 플러그인 로드 테스트 통과. (단위테스트는 pluginlib 직접 호출이라 이 경로를 우회했었음.)
- 🔶 **진행 중(다음 작업):** EscapeCritic을 켠 채 좁은 tb3_sandbox에서 **탈출(local-minima escape) 라이브 시연**. critic 라이브 로드까지는 검증 예정, 좁은 sandbox + AMCL 까다로움으로 *완주*는 변동성 있음 → 정량 검증은 M6 벤치마크로 분리 권장.

---

## 1. 빌드 & 활성화 (로컬, 세션마다)

ROS1 Noetic 오염을 제거하고 conda ROS2(Jazzy)만 활성화하는 게 관건이었다. 활성화 헬퍼:

```bash
# /tmp/act_ros2.sh — 깨끗한 ROS2(Jazzy) 활성화 (noetic 제거 + conda ros2)
cat > /tmp/act_ros2.sh <<'EOF'
unset ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH 2>/dev/null
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '/opt/ros/noetic' | paste -sd:)
export MAMBA_ROOT_PREFIX=/opt/micromamba
eval "$(/root/micromamba shell hook --shell bash 2>/dev/null || micromamba shell hook --shell bash)"
micromamba activate ros2
EOF
source /tmp/act_ros2.sh
# 확인: ROS_DISTRO=jazzy, which ros2 -> /opt/micromamba/envs/ros2/bin/ros2
```

빌드:

```bash
cd ~/ws/se-mppi-nav2
git fetch origin claude/fervent-newton-lbo96
git checkout origin/claude/fervent-newton-lbo96 -- src experiments   # 또는 git pull
colcon build --packages-select nav2_se_controller
source install/setup.bash
```

> **언제 재빌드 필요?** `src/nav2_se_controller/**`(C++)가 바뀌면 재빌드. `experiments/sim/*.yaml`·`smoke_drive.py`만 바뀌면 **재빌드 불필요**(런치 재시작만).

빌드 산출물: `escape_critic`, `safe_escape_controller`, `se_mppi_core` 3개 `.so`.

---

## 2. 라이브 런 플레이북 (tb3_sandbox)

### 2-A. 클린 재시작 (★ 매 런 전 필수)
여러 번 런치하면 잔존 프로세스/공유메모리가 `/clock`·TF를 충돌시켜 `transformPose Exception`·`extrapolation` 에러가 난다. 매번 완전 정리:

```bash
pkill -9 -f "tb3_simulation_launch|gz sim|gzserver|component_container|robot_state_publisher|parameter_bridge|ros_gz|smoke_drive|nav2|rviz" 2>/dev/null
ros2 daemon stop 2>/dev/null
rm -rf /dev/shm/fastrtps_* /dev/shm/fast_dds_* /dev/shm/sem.* 2>/dev/null
sleep 4
pgrep -af "gz sim|component_container|ros2 launch" || echo "ALL CLEAN"
ros2 daemon start 2>/dev/null
```

### 2-B. 런치 (SE-MPPI)
```bash
cd ~/ws/se-mppi-nav2
export TURTLEBOT3_MODEL=waffle
nohup ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False \
  params_file:=$PWD/experiments/sim/nav2_se_loopback.yaml > /tmp/se_gz.log 2>&1 &
sleep 40
grep -E "EscapeCritic.*initialized|Created controller : FollowPath|Managed nodes are active|Aborting" /tmp/se_gz.log | tail -6
```
기대: `Created controller : FollowPath of type nav2_se_controller::SafeEscapeController`,
`EscapeCritic[...] initialized`, `Managed nodes are active`, `Aborting` 없음.

### 2-C. 주행
```bash
# 정면 1.5m 직진(베이스라인) — 또는 트랩 목표(0.9,-2.25)
SE_GOAL_X=-0.53 SE_GOAL_Y=-0.53 SE_LOCALIZER=amcl python3 experiments/sim/smoke_drive.py
```
- `smoke_drive.py`는 `SE_LOCALIZER=amcl`이면 **initialpose 재발행을 생략**한다(Gazebo에선 로봇이 물리적으로 안 움직이는데 AMCL 믿음만 리셋하면 로컬라이제이션이 깨지기 때문). 강제하려면 `SE_FORCE_RELOCATE=1`.
- 좌표 환경변수: `SE_START_X/Y`(스폰=-2.0,-0.5), `SE_GOAL_X/Y`, `SE_LOCALIZER`.

### 2-D. stock MPPI A/B (대조군)
동일 조건에서 컨트롤러만 stock으로:
```bash
# 2-A 클린 → 런치를 params_file=experiments/sim/nav2_stock_mppi.yaml 로
```

---

## 3. 이번 세션에서 고친 것 (커밋별)

| 커밋 | 내용 | 왜 |
|---|---|---|
| `21d740a` | **CBF를 동적 장애물로만 한정** (속도≥0.1m/s & 반경≤1.0m 게이트) | 트래커가 정적 벽(LETHAL 셀)을 거대 원형 장애물로 만들어 CBF가 영구 freeze(`linear.x=0`, 제자리 회전)시킴. 설계 의도(CBF=동적 전용)와도 일치. |
| `79523ae` | amcl `set_initial_pose`(스폰 -2.0,-0.5) | 초기포즈 없으면 nav 라이프사이클이 TF 타임아웃으로 abort. |
| `4393dbc` | `nav2_stock_mppi.yaml` 대조군 params | 컨트롤러 vs 월드/좌표 문제 격리용. |
| `0c6c750` | `pick_goal.py` 오프라인 골 파인더 | 맵 pgm을 읽어 도달가능·벽회피 목표 산출. |
| `a3dfc3c` | 인플레이션 0.70→**0.35** | tb3_sandbox가 좁아(최대 여유 0.9m) 0.70 인플레이션이 통로를 다 막아 MPPI가 전진 불가. |
| `52bf1c8` | **연산 부하 축소** + initialpose 오염 방지 | 컨트롤러가 20Hz 목표에 5.2Hz밖에 못 내(4배 부족) 배회·`Optimizer fail`. `controller_frequency 10 / model_dt 0.1 / time_steps 28 / batch 1500`(호라이즌 2.8s 유지). |
| `fd70c22` | **후진 금지 `vx_min: 0.0`** | telemetry로 확인: 정면 개방인데 MPPI가 후진 샘플을 채택해 뒷벽(0.17m)에 박힘(odom -0.58m). stock도 동일. |
| `e2c825b` | **EscapeCritic을 `mppi::critics` 네임스페이스로 이동** | MPPI CriticManager는 critics 이름을 `mppi::critics::`+이름으로 로드 → 기존 `nav2_se_controller::EscapeCritic`은 critics 목록으로 **절대 로드 불가**였음. |

> 각 수정은 단위 변화(한 번에 하나)로 telemetry 근거와 함께 적용. 자세한 근거는 각 커밋 메시지 참조.

---

## 4. 핵심 진단 결과 (다시 안 헤매도록)

- **구동 방향 정상**: 수동 `cmd_vel x:0.2`→로봇이 헤딩 방향으로 전진. 부호 반전 아님.
- **무명령 드리프트 없음**: 15초 정지 시 위치 불변.
- **"후진해서 벽에 박힘"의 정체**: 초기엔 ① CBF freeze, ② MPPI 후진 샘플 채택 두 가지였음(둘 다 수정).
- **tb3_sandbox 지오메트리** (pgm 분석): 맵 384×384@0.05, origin(-10,-10). 실제 자유공간 bbox ≈ x[-2.57,2.33] y[-2.27,2.23]. **최대 여유 0.90m**(매우 좁음). 스폰(-2.0,-0.5)은 free·safe, 정면 ~0.6m에 장애물.
- **유효 목표**: `(-0.53,-0.53)` 정면 1.5m tortuosity 1.02(직선). `(0.9,-2.25)` tortuosity 1.37(통로 꺾임, stock이 헤맴 = 트랩 후보). 골 산출은 `python3 experiments/sim/pick_goal.py <map.yaml> <sx> <sy>`.
- **남은 변동 요인**: 좁은 sandbox + AMCL이 가끔 map→odom을 점프(재수렴)시켜 dist가 진동. 컨트롤러 무관, 환경 특성.

---

## 5. 다음 할 일 (우선순위)

1. **(진행 중) EscapeCritic 라이브 로드 확인** — 2-B 런치 후 `EscapeCritic[...] initialized` 로그 확인. 이게 이번 네임스페이스 수정의 결정적 검증.
2. **탈출 시연 A/B** — 트랩 목표 `(0.9,-2.25)`로:
   - stock(`nav2_stock_mppi.yaml`, EscapeCritic 없음) → 헤맴/실패(트랩)
   - SE(`nav2_se_loopback.yaml`, EscapeCritic on) → 탈출/도달
   - entrapment 감지·escape 비용 주입이 동작하는지(거동/로그)까지 확인. 완주가 흔들리면 거기서 멈추고 3번으로.
3. **정량 평가는 M6 벤치마크로** — hand-tuned tb3_sandbox 말고 BARN/DynaBARN/HuNavSim에서 성공률·충돌률·시간 측정 (`docs/architecture/BACKLOG.md`, `experiments/README.md`의 ablation A–F). 라이브 "탈출 장면"은 명확한 U-trap 월드 1개에서.
4. **AMCL 안정화(선택)** — sandbox에서 점프가 거슬리면 초기 분산↓·`update_min_d/a`↓·빔수↑ 튜닝. 단 평가는 벤치마크 월드에서 하면 대개 불필요.

---

## 6. 알려진 함정 (반복 방지)

- **클라우드 컨테이너의 git 트리가 가끔 옛 커밋으로 리셋됨** → 항상 `git fetch && git reset --hard origin/claude/fervent-newton-lbo96`로 동기화 후 작업. **원격이 진실.**
- **여러 번 런치 시 TF/clock 충돌** → §2-A 클린 재시작 필수. 증상: `transformPose Exception`, `extrapolation`, `RTPS_TRANSPORT_SHM ... port lock failed`.
- **Gazebo에선 `/initialpose`가 로봇을 텔레포트하지 않음** → 재시도 시 AMCL 믿음만 깨짐. `SE_LOCALIZER=amcl`이면 smoke_drive가 자동 생략.
- **`micromamba: command not found`** → 셸 hook 미적용. `/tmp/act_ros2.sh` 사용.
- **EscapeCritic 안 보임** → critics 목록 이름은 반드시 `"EscapeCritic"`(짧은 이름), 클래스는 `mppi::critics::EscapeCritic`이어야 로드됨.

---

## 7. 빠른 참조 — 파일 지도

- `src/nav2_se_controller/` — 컨트롤러 패키지 (3 .so)
  - `safe_escape_controller.{hpp,cpp}` — MPPI 서브클래스 + entrapment + CBF 후처리
  - `escape_critic.{hpp,cpp}` — **`mppi::critics::EscapeCritic`** (APF+gap 탈출 critic)
  - `cbf_safety_filter.{hpp,cpp}`, `escape_safety_coordinator.{hpp,cpp}`, `dynamic_obstacle_tracker.{hpp,cpp}`, `repulsion.*`, `gap_search.*`, `entrapment_*.hpp`
- `experiments/sim/`
  - `nav2_se_loopback.yaml` — SE 런 params (EscapeCritic 포함, 호스트-핏 튜닝)
  - `nav2_stock_mppi.yaml` — stock 대조군
  - `smoke_drive.py` — 목표 전송·진행도 로깅 (AMCL 모드 자동 처리)
  - `pick_goal.py` — 오프라인 골 파인더
- `experiments/prototype/` — Python 2D 검증 + 그림(utrap_escape/dynamic_cbf/coordination)
- `docs/research/`, `docs/architecture/`, `docs/papers/` — 문제정의·설계·평가프로토콜·논문 초안·novelty 검증
- `RUN.md` — 로컬 실행 가이드(RoboStack/Docker)

---

*작성: 라이브 Gazebo 디버깅 세션 종료 시점. 이어서 §5부터 진행.*
