#!/usr/bin/env bash
# =============================================================================
# build_mppi_overlay.sh - nav2_mppi_controller from source, without -ffast-math
#
# Some distributed nav2_mppi_controller binaries negate cos/sin in the
# optimizer's batch rollouts and command motion away from the goal (README,
# "Affected nav2_mppi_controller builds"). nav2_se_controller refuses to
# configure on them. For ROS 2 Humble there is no corrected RoboStack binary;
# this script builds the same upstream source into a colcon workspace with the
# one change that corrects it:
#
#   - "-ffast-math" removed from add_compile_options (what RoboStack's Jazzy
#     recipe does from 1.3.12 build _22 on);
#   - "xtensor::optimize" removed from the link line (it adds -march=native;
#     RoboStack's Humble recipe removes it too, so the binary runs on other
#     CPUs than the build host).
#
# Usage (inside the activated ROS 2 environment):
#   scripts/build_mppi_overlay.sh <workspace> [nav2 tag]
#
# <workspace>  colcon workspace to build in (created if missing). Afterwards
#              build nav2_se_controller in the SAME workspace, so that it
#              compiles and links against this nav2_mppi_controller, WITH the
#              RPATH option (without it the plugin loads the environment's
#              libmppi_controller.so at run time; see the package CMakeLists.txt):
#                ln -s "$PWD/src/nav2_se_controller" <workspace>/src/
#                (cd <workspace> && colcon build --packages-select nav2_se_controller \
#                   --cmake-args -DNAV2_SE_CONTROLLER_MPPI_RPATH=ON)
#                source <workspace>/install/setup.bash
#                ros2 run nav2_se_controller check_mppi_rollout   # OK + a path under <workspace>
# [nav2 tag]   navigation2 release tag; default: the version of the installed
#              nav2_mppi_controller (e.g. 1.1.20).
# =============================================================================
set -euo pipefail

WS="${1:?usage: build_mppi_overlay.sh <workspace> [nav2 tag]}"
TAG="${2:-}"

if [ -z "$TAG" ]; then
  PREFIX="$(ros2 pkg prefix nav2_mppi_controller)"
  TAG="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' \
    "$PREFIX/share/nav2_mppi_controller/package.xml" | head -n 1)"
fi
[ -n "$TAG" ] || { echo "cannot determine the nav2_mppi_controller version; pass a tag" >&2; exit 1; }

# sha256 of https://github.com/ros-navigation/navigation2/archive/refs/tags/<tag>.tar.gz
declare -A KNOWN_SHA256=(
  [1.1.20]=c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6
)

mkdir -p "$WS/src"
WS="$(cd "$WS" && pwd)"
SRC="$WS/src/nav2_mppi_controller"
TARBALL="$WS/navigation2-$TAG.tar.gz"

echo "[build_mppi_overlay] navigation2 $TAG -> $SRC"
curl -sfL --retry 3 -o "$TARBALL" \
  "https://github.com/ros-navigation/navigation2/archive/refs/tags/$TAG.tar.gz"
GOT_SHA256="$(sha256sum "$TARBALL" | cut -d' ' -f1)"
if [ -n "${KNOWN_SHA256[$TAG]:-}" ]; then
  [ "$GOT_SHA256" = "${KNOWN_SHA256[$TAG]}" ] || {
    echo "sha256 mismatch for navigation2 $TAG: got $GOT_SHA256" >&2; exit 1; }
else
  echo "[build_mppi_overlay] note: no pinned sha256 for tag $TAG (got $GOT_SHA256)"
fi

rm -rf "$SRC"
tar -xzf "$TARBALL" -C "$WS/src" --strip-components=1 "navigation2-$TAG/nav2_mppi_controller"
rm -f "$TARBALL"

CMAKELISTS="$SRC/CMakeLists.txt"
grep -q -- "-ffast-math" "$CMAKELISTS" || {
  echo "no -ffast-math in $CMAKELISTS: this tag needs no overlay, or its build changed" >&2; exit 1; }
sed -i -e 's/ -ffast-math//g' -e 's/ xtensor::optimize//g' "$CMAKELISTS"
if grep -q -e "-ffast-math" -e "xtensor::optimize" "$CMAKELISTS"; then
  echo "could not remove -ffast-math / xtensor::optimize from $CMAKELISTS" >&2; exit 1
fi

cd "$WS"
colcon build --packages-select nav2_mppi_controller --event-handlers console_cohesion+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

echo "[build_mppi_overlay] built $WS/install/nav2_mppi_controller (no -ffast-math)."
echo "[build_mppi_overlay] next: build nav2_se_controller in $WS with"
echo "[build_mppi_overlay]   --cmake-args -DNAV2_SE_CONTROLLER_MPPI_RPATH=ON, then source $WS/install/setup.bash"
