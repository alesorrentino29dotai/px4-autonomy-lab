#!/usr/bin/env bash
# Start/refresh the ROS 2 side of the sim: gz->ROS bridge + Micro XRCE agent.
#
#   WORLD=baylands MODEL=x500_mono_cam_down_0 ./sim/run_ros2_bridge.sh
#
# Topology (all containers on the host network):
#   gz-transport (px4-sitl) ── ros_gz_bridge (ros2 ctr) ──> ROS 2 DDS
#   PX4 uxrce_dds_client :8888 ── micro-ros-agent ────────> /fmu/* topics
#
# The bridge topic set is generated from sim/bridge_config.tpl.yaml with the
# world/model substituted (gz camera topic names embed both).
set -euo pipefail
WORLD="${WORLD:-baylands}"
MODEL="${MODEL:-x500_mono_cam_down_0}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! docker ps --format '{{.Names}}' | grep -qx ros2; then
  docker rm -f ros2 >/dev/null 2>&1 || true
  docker run -d --name ros2 --network host --ipc host -v "${REPO_DIR}:/lab" \
    ros:jazzy-ros-base sleep infinity
  docker exec ros2 bash -c \
    'apt-get update -qq >/dev/null && apt-get install -y -qq ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image >/dev/null'
  echo "[bridge] ros2 container created"
fi

sed "s/@WORLD@/${WORLD}/g; s/@MODEL@/${MODEL}/g" \
  "${REPO_DIR}/sim/bridge_config.tpl.yaml" > "${REPO_DIR}/sim/.bridge_config.yaml"

docker exec ros2 bash -c "pkill -f '[p]arameter_bridge' 2>/dev/null; true"
docker exec -d ros2 bash -c "source /opt/ros/jazzy/setup.bash && \
  ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:=/lab/sim/.bridge_config.yaml"
echo "[bridge] parameter_bridge running (world=${WORLD} model=${MODEL})"

if ! docker ps --format '{{.Names}}' | grep -qx xrce-agent; then
  docker rm -f xrce-agent >/dev/null 2>&1 || true
  docker run -d --name xrce-agent --network host --ipc host \
    microros/micro-ros-agent:jazzy udp4 --port 8888 >/dev/null
  echo "[bridge] micro-ros-agent running on udp 8888"
fi
