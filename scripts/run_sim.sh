#!/usr/bin/env bash
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
#
# One-command SE-MPPI simulation on a workstation:
#   env activate -> build (if needed) -> Gazebo + Nav2 + RViz -> (optional) goal.
#
#   bash scripts/run_sim.sh                 # GUI sim + RViz, you click the goal
#   bash scripts/run_sim.sh --drive         # + auto initial pose & goal (smoke)
#   bash scripts/run_sim.sh --headless      # no GUI (CI / remote)
#
# Visualization (RViz):
#   * MPPI candidate trajectories  : topic FollowPath/trajectories (stock MPPI viz)
#   * SE-MPPI internals            : MarkerArray FollowPath/se_markers
#       - orange discs  = CBF effective radius per dynamic obstacle,
#                         ALREADY inflated by the conformal bound q
#       - cyan lines    = predicted obstacle horizons (1.5 s)
#       - status text   = alpha / QP slack / entrapped / max q  (above the robot;
#                         turns red while escaping)
#   Add both in RViz: Add -> By topic. See RUN.md §0/§4.

set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"

HEADLESS=False
USE_RVIZ=True
DRIVE=0
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=True; USE_RVIZ=False ;;
    --drive)    DRIVE=1 ;;
    *) echo "unknown arg: $arg (use --headless / --drive)"; exit 2 ;;
  esac
done

# ---- 1. micromamba env (handles the root/non-root path split) ---------------
find_mamba() {
  for c in "$HOME/.local/bin/micromamba" /root/micromamba /usr/local/bin/micromamba \
           "$(command -v micromamba 2>/dev/null || true)"; do
    [ -n "$c" ] && [ -x "$c" ] && { echo "$c"; return; }
  done
  echo ""
}
# Always run the (idempotent) env setup: installs micromamba + the ROS2 env on
# first use, and on every run ensures the live-sim extras (rviz2, ros-gz) are
# present — the fix for "package 'rviz2' not found" on a ros-base-only env.
echo "[run_sim] ensuring ROS2 env + live-sim extras (idempotent)..."
bash scripts/setup_ros2_env.sh
MAMBA_BIN="$(find_mamba)"
[ -n "$MAMBA_BIN" ] || { echo "[run_sim] env setup failed; see RUN.md §1"; exit 1; }

# Strip any ROS1 (noetic/melodic) paths a user's .bashrc may have sourced:
# mixing distributions corrupts PYTHONPATH/CMAKE_PREFIX_PATH/AMENT_PREFIX_PATH
# and breaks the launch's world/robot SDF generation ("No such file ... .sdf").
for _v in PATH LD_LIBRARY_PATH PYTHONPATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH \
          AMENT_PREFIX_PATH COLCON_PREFIX_PATH; do
  _val="${!_v:-}"
  [ -z "$_val" ] && continue
  _out=""
  IFS=':' read -ra _parts <<< "$_val"
  for _p in "${_parts[@]}"; do
    case "$_p" in
      */noetic/*|*/melodic/*|*opt/ros/noetic*) ;;   # drop ROS1 entries
      "") ;;
      *) _out="${_out:+$_out:}$_p" ;;
    esac
  done
  export "$_v=$_out"
done
unset ROS_DISTRO ROS_VERSION ROS_ROOT ROS_PACKAGE_PATH ROS_ETC_DIR \
      ROS_MASTER_URI ROS_PYTHON_VERSION 2>/dev/null || true

export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$([ "$(id -u)" = 0 ] && echo /opt/micromamba || echo "$HOME/micromamba")}"
# Same unusable-root guard as setup_ros2_env.sh (a child process, so its
# corrected value does NOT propagate back here): if the environment injected a
# root we cannot write (stale root-owned /opt/micromamba), fall back — else we
# would activate an env missing rviz2/ros-gz and the launch dies.
DEFAULT_ROOT="$([ "$(id -u)" = 0 ] && echo /opt/micromamba || echo "$HOME/micromamba")"
if [ "$MAMBA_ROOT_PREFIX" != "$DEFAULT_ROOT" ] && [ -d "$MAMBA_ROOT_PREFIX" ] && [ ! -w "$MAMBA_ROOT_PREFIX" ]; then
  echo "[run_sim] warning: MAMBA_ROOT_PREFIX=$MAMBA_ROOT_PREFIX not writable — falling back to $DEFAULT_ROOT"
  export MAMBA_ROOT_PREFIX="$DEFAULT_ROOT"
fi
# conda/RoboStack/ROS activation + setup scripts reference unset vars
# (CONDA_BUILD, AMENT_*, ...) and are NOT `set -u` clean. Relax nounset around
# every activation/source, then restore it for our own logic.
set +u
eval "$("$MAMBA_BIN" shell hook -s bash)"
micromamba activate ros2
set -u

# ---- 2. build if the plugin library is missing or sources are newer ---------
LIB="install/nav2_se_controller/lib/libsafe_escape_controller.so"
if [ ! -f "$LIB" ] || [ -n "$(find src/nav2_se_controller -name '*.cpp' -o -name '*.hpp' -newer "$LIB" 2>/dev/null | head -1)" ]; then
  echo "[run_sim] building nav2_se_controller..."
  colcon build --packages-select nav2_se_controller
fi
set +u
source install/setup.bash
set -u

# ---- 3. launch Gazebo + Nav2 (+ RViz) with the SE-MPPI params ----------------
export TURTLEBOT3_MODEL=waffle
TB3_SIM_PREFIX="$(ros2 pkg prefix nav2_minimal_tb3_sim 2>/dev/null || true)"
if [ -n "$TB3_SIM_PREFIX" ]; then
  export GZ_SIM_RESOURCE_PATH="$TB3_SIM_PREFIX/share/nav2_minimal_tb3_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
fi

# Pre-flight: leftovers from a previous (crashed/killed) run are fatal — a
# stale ros_gz parameter_bridge discovers the NEW gz server and republishes
# its /clock, and two interleaved /clock publishers make subscribers see
# backward time jumps ("Detected jump back in time") that clear TF buffers
# until planning fails with "Unable to get start pose".
STALE="$(pgrep -f 'ros_gz_bridge/parameter_bridge|gz sim|component_container_isolated|robot_state_publisher' || true)"
if [ -n "$STALE" ]; then
  echo "[run_sim] killing stale sim processes from a previous run:"
  pgrep -af 'ros_gz_bridge/parameter_bridge|gz sim|component_container_isolated|robot_state_publisher' || true
  pkill -9 -f 'ros_gz_bridge/parameter_bridge|rviz2/rviz2|gz sim|component_container_isolated|robot_state_publisher' || true
  sleep 2
fi

# Params override (A/B tests, e.g. stock MPPI vs SE-MPPI): SE_PARAMS_FILE=<yaml>
PARAMS_FILE="${SE_PARAMS_FILE:-$REPO/experiments/sim/nav2_se_loopback.yaml}"

echo "[run_sim] launching (headless=$HEADLESS rviz=$USE_RVIZ params=$PARAMS_FILE)..."
echo "[run_sim] RViz tips: Add -> By topic -> FollowPath/se_markers (SE internals)"
echo "[run_sim]            Add -> By topic -> FollowPath/trajectories (MPPI samples)"

if [ "$DRIVE" = 1 ]; then
  ros2 launch nav2_bringup tb3_simulation_launch.py \
    headless:=$HEADLESS use_rviz:=$USE_RVIZ \
    params_file:="$PARAMS_FILE" &
  LAUNCH_PID=$!
  # SIGINT lets ros2 launch tear its children down; give it time, then make
  # sure nothing leaked (orphaned gz/bridge processes break the NEXT run).
  cleanup() {
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
    pkill -9 -f 'ros_gz_bridge/parameter_bridge|rviz2/rviz2|gz sim|component_container_isolated|robot_state_publisher' 2>/dev/null || true
  }
  trap cleanup EXIT
  sleep 25   # let lifecycle come up; smoke_drive also waits on nav2 active
  # Goal/start are env-overridable for A/B and trap scenarios (e.g. the
  # escape demo at the U-trap goal 0.9,-2.25); defaults = the 1 m smoke drive.
  export SE_START_X="${SE_START_X:--2.0}" SE_START_Y="${SE_START_Y:--0.5}" \
         SE_GOAL_X="${SE_GOAL_X:--1.0}" SE_GOAL_Y="${SE_GOAL_Y:--0.5}" \
         SE_LOCALIZER="${SE_LOCALIZER:-amcl}"
  python3 "$REPO/experiments/sim/smoke_drive.py"
  echo "[run_sim] drive finished — sim stays up (Ctrl-C to stop)"
  wait $LAUNCH_PID
else
  exec ros2 launch nav2_bringup tb3_simulation_launch.py \
    headless:=$HEADLESS use_rviz:=$USE_RVIZ \
    params_file:="$PARAMS_FILE"
fi
