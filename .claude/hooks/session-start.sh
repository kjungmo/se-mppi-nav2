#!/usr/bin/env bash
# =============================================================================
# SessionStart hook — prepare the ROS2 Jazzy + Nav2 build environment.
#
# This repo targets a ROS2 Nav2 controller plugin (SE-MPPI). The Claude Code on
# the web container has NO ROS2 by default and packages.ros.org is network-
# blocked (403), so we install ROS2 Jazzy + Nav2 + Gazebo via RoboStack
# (conda-forge), which IS reachable. The heavy install runs once; the container
# state is cached afterwards, so later sessions fast-path via the ready marker.
#
# Synchronous mode (no async banner): guarantees ros2/colcon are ready before
# the agent runs builds/tests. First run is slow (~10 min, then cached).
# =============================================================================
set -uo pipefail

PROJECT_DIR="${CLAUDE_PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

# Install / verify the ROS2 env (idempotent).
bash "${PROJECT_DIR}/scripts/setup_ros2_env.sh"

# Persist micromamba env activation for all subsequent session shells so that
# `ros2`, `colcon`, `gz` are on PATH without an explicit `micromamba run`.
if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
  {
    echo 'export MAMBA_ROOT_PREFIX=/opt/micromamba'
    echo 'eval "$(/root/micromamba shell hook --shell bash 2>/dev/null)"'
    echo 'micromamba activate ros2 2>/dev/null || true'
  } >> "$CLAUDE_ENV_FILE"
fi

echo "[session-start] ROS2 build environment ready."
