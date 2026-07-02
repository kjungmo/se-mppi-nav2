#!/usr/bin/env bash
# =============================================================================
# setup_ros2_env.sh — ROS2 Jazzy + Nav2 + Gazebo build environment via RoboStack
#
# packages.ros.org에 의존하지 않고(차단/OS 버전 무관) conda-forge(RoboStack)로
# ROS2 Jazzy를 설치한다. Ubuntu 20.04/22.04/24.04 어디서나 동작.
#
# 멱등(idempotent): 이미 설치돼 있으면 빠르게 통과한다.
# 사용:
#   bash scripts/setup_ros2_env.sh          # 설치 (끝나면 활성화 명령을 안내)
#   source scripts/setup_ros2_env.sh        # 설치 + 현재 셸에 바로 활성화
# 경로 override: MAMBA_BIN, MAMBA_ROOT_PREFIX 환경변수로 지정 가능.
# =============================================================================
set -uo pipefail

# 기본 경로: root(클라우드 컨테이너)면 /root + /opt(캐시), 아니면 유저 홈(로컬, sudo 불필요).
if [ "$(id -u)" = "0" ]; then
  MAMBA_BIN="${MAMBA_BIN:-/root/micromamba}"
  export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-/opt/micromamba}"
else
  MAMBA_BIN="${MAMBA_BIN:-$HOME/.local/bin/micromamba}"
  export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
fi
# 쓰기 불가 root prefix 가드: 환경(예: 세션 hook)이 stale root-소유
# /opt/micromamba 를 주입하면 설치/보강이 Permission denied로 깨지고 rviz2 등이
# 빠진 env로 launch가 실패한다. 기본 경로로 폴백.
DEFAULT_ROOT="$([ "$(id -u)" = 0 ] && echo /opt/micromamba || echo "$HOME/micromamba")"
if [ "$MAMBA_ROOT_PREFIX" != "$DEFAULT_ROOT" ] && [ -d "$MAMBA_ROOT_PREFIX" ] && [ ! -w "$MAMBA_ROOT_PREFIX" ]; then
  echo "[setup_ros2_env] 경고: MAMBA_ROOT_PREFIX=$MAMBA_ROOT_PREFIX 쓰기 불가 — $DEFAULT_ROOT 로 폴백"
  export MAMBA_ROOT_PREFIX="$DEFAULT_ROOT"
fi
ENV_NAME="${ROS2_ENV_NAME:-ros2}"
ROS_DISTRO_TARGET="jazzy"
MARKER="${MAMBA_ROOT_PREFIX}/envs/${ENV_NAME}/.se_mppi_ready"

log() { echo "[setup_ros2_env] $*"; }

# --- 1. micromamba 확보 ----------------------------------------------------
if [ ! -x "$MAMBA_BIN" ]; then
  log "micromamba 다운로드 -> $MAMBA_BIN"
  mkdir -p "$(dirname "$MAMBA_BIN")"
  curl -L -o "$MAMBA_BIN" \
    "https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64" \
    && chmod +x "$MAMBA_BIN"
fi
log "micromamba $("$MAMBA_BIN" --version)"

# --- 2. ROS2 + Nav2 + 시뮬 환경 생성 (없을 때만) ---------------------------
ENV_ROS2_BIN="${MAMBA_ROOT_PREFIX}/envs/${ENV_NAME}/bin/ros2"
# 컨테이너 캐시 등으로 env가 이미 존재하면(ros2 바이너리 확인) marker를 보정하고 통과.
if [ -x "$ENV_ROS2_BIN" ] && [ ! -f "$MARKER" ]; then
  log "기존 ROS2 env 감지 — marker 보정"; touch "$MARKER"
fi
if [ -f "$MARKER" ]; then
  log "환경 '${ENV_NAME}' 이미 준비됨 — 설치 건너뜀"
else
  log "ROS2 ${ROS_DISTRO_TARGET} + Nav2 + Gazebo 환경 생성 (수 분 소요)..."
  "$MAMBA_BIN" create -y -n "$ENV_NAME" \
    -c robostack-jazzy -c conda-forge \
    ros-jazzy-ros-base \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-mppi-controller \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-nav2-loopback-sim \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-rviz2 \
    ros-jazzy-ros-gz \
    colcon-common-extensions \
    cxx-compiler cmake ninja pkg-config \
    osqp-eigen eigen \
    && touch "$MARKER" \
    && log "환경 생성 완료" \
    || { log "환경 생성 실패"; return 1 2>/dev/null || exit 1; }
fi

# --- 2b. 라이브 시뮬 추가 패키지 (RViz + Gazebo 브리지) — 멱등 보정 ----------
# 초기 설치가 ros-base 기반이라 rviz2/ros-gz가 빠진 기존 env를 자동 보강한다
# (live 시뮬에서 "package 'rviz2' not found" / sdf 생성 실패의 원인).
ENV_DIR="${MAMBA_ROOT_PREFIX}/envs/${ENV_NAME}"
NEED_EXTRAS=()
[ -x "${ENV_DIR}/bin/rviz2" ]        || NEED_EXTRAS+=(ros-jazzy-rviz2)
[ -d "${ENV_DIR}/share/ros_gz_sim" ] || NEED_EXTRAS+=(ros-jazzy-ros-gz)
if [ "${#NEED_EXTRAS[@]}" -gt 0 ]; then
  log "라이브 시뮬 추가 패키지 설치: ${NEED_EXTRAS[*]} (수 분 소요)"
  "$MAMBA_BIN" install -y -n "$ENV_NAME" -c robostack-jazzy -c conda-forge \
    "${NEED_EXTRAS[@]}" \
    && log "추가 패키지 설치 완료" \
    || log "추가 패키지 설치 실패 — RViz/Gazebo 누락 가능(RUN.md §7)"
fi

# --- 3. 활성화 ------------------------------------------------------------
# sourced 면 부모 셸에 활성화가 전파되지만, bash 로 실행되면 서브셸에서만 적용되므로
# (활성화됐다고 잘못 보고하지 않도록) 실행 방식에 따라 분기한다.
if (return 0 2>/dev/null); then
  # source scripts/setup_ros2_env.sh 로 호출된 경우 -> 현재 셸에 바로 활성화.
  # shellcheck disable=SC1090
  eval "$("$MAMBA_BIN" shell hook --shell bash)" 2>/dev/null
  micromamba activate "$ENV_NAME" 2>/dev/null \
    && log "현재 셸에 활성화됨 (ROS_DISTRO=${ROS_DISTRO:-?}). 'ros2'/'colcon' 사용 가능."
else
  # bash scripts/setup_ros2_env.sh 로 실행된 경우 -> 활성화 명령을 안내(복붙).
  cat <<EOF
[setup_ros2_env] 설치 완료. 현재 셸에서 활성화하려면 아래를 실행:
  export MAMBA_ROOT_PREFIX=$MAMBA_ROOT_PREFIX
  eval "\$($MAMBA_BIN shell hook -s bash)"
  micromamba activate $ENV_NAME
  ros2 --version   # jazzy 확인
영구 등록(1회): $MAMBA_BIN shell init -s bash -r $MAMBA_ROOT_PREFIX && source ~/.bashrc
EOF
fi
