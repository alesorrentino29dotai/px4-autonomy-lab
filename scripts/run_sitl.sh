#!/usr/bin/env bash
#
# run_sitl.sh — Start PX4 SITL + Gazebo in the official pre-built Docker container.
#
# WHAT IT DOES
#   Runs px4io/px4-sitl-gazebo (PX4 + Gazebo Harmonic in one image) using
#   --network host (Linux): PX4 sends MAVLink to localhost, which with host
#   networking IS the host's localhost, so no port mapping is needed:
#     - 14550/udp  -> QGroundControl (MAVLink "GCS" link)
#     - 14540/udp  -> MAVSDK / offboard API (MAVLink "onboard" link)
#   NOTE: plain `-p 14550:14550/udp` does NOT work here — PX4 *initiates* the
#   UDP stream toward localhost inside the container, and docker-proxy only
#   forwards inbound traffic (it even steals the host port). Host networking
#   solves both problems.
#   If $DISPLAY is set (and HEADLESS != 1) the Gazebo GUI is forwarded via X11;
#   otherwise Gazebo runs headless and you monitor the drone from QGroundControl.
#
# USAGE
#   ./run_sitl.sh                         # default: gz_x500 quadrotor, default world
#   MODEL=gz_x500_mono_cam_down ./run_sitl.sh   # downward camera variant (M5)
#   HEADLESS=1 ./run_sitl.sh              # force headless Gazebo
#   WORLD=aruco ./run_sitl.sh             # pick another Gazebo world
#
# The PX4 shell (pxh>) runs on the container's stdin/stdout: this script runs
# the container in the foreground, so you get the console directly. Ctrl-C stops
# SITL and removes the container.

set -euo pipefail

TAG="${TAG:-v1.18.0-beta1}"          # pinned: latest tagged release (no stable vX.Y.Z yet)
MODEL="${MODEL:-gz_x500}"
WORLD="${WORLD:-default}"
NAME="${NAME:-px4-sitl}"
IMAGE="px4io/px4-sitl-gazebo:${TAG}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ARGS=(
  --rm -it --name "$NAME"
  --network host       # MAVLink on host localhost: 14550 (QGC), 14540 (MAVSDK)
  -v "${REPO_DIR}:/lab" # repo visible in-container (M5 CV node runs from here)
  -e "PX4_SIM_MODEL=${MODEL}"
  -e "PX4_GZ_WORLD=${WORLD}"
)

if [[ "${HEADLESS:-0}" != "1" && -n "${DISPLAY:-}" ]]; then
  # Gazebo GUI on the host X server
  xhost +local:docker >/dev/null 2>&1 || true
  ARGS+=( -e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix )
else
  ARGS+=( -e HEADLESS=1 )
  echo "[run_sitl] running headless (no DISPLAY or HEADLESS=1) — use QGroundControl to monitor"
fi

echo "[run_sitl] starting ${IMAGE} model=${MODEL} world=${WORLD}"
exec docker run "${ARGS[@]}" "$IMAGE"
