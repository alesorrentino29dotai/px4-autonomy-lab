#!/usr/bin/env bash
# One-time setup of a freshly started px4-sitl container for the M5/M7 demos.
# Idempotent — rerun after every `docker run` of the sim container.
#
#   1. OakD-Lite depth_camera -> rgbd_camera (aligned color+depth for cuVSLAM).
#   2. python3-opencv for the in-container CV node (M5).
#
# GPU note: Gazebo renders on the AMD iGPU render node passed by run_sitl.sh.
# NEVER install the NVIDIA EGL ICD in this container: gz sim on the NVIDIA
# 595 open-module driver leaks host RAM at ~3 GiB/min and hard-froze the
# machine twice (2026-07-23). Verified fix: mesa/radeonsi on the iGPU,
# stable at 30 Hz with flat memory.
#
# A `docker restart` is required for 1 to take effect on a scene that
# already loaded; this script does it when anything changed.
set -euo pipefail
NAME="${1:-px4-sitl}"

changed=0

if docker exec "$NAME" test -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json 2>/dev/null; then
  docker exec "$NAME" rm /usr/share/glvnd/egl_vendor.d/10_nvidia.json
  echo "[extras] removed NVIDIA EGL ICD (leaky driver — see header note)"
  changed=1
fi

if ! docker exec "$NAME" grep -q 'type="rgbd_camera"' /opt/px4-gazebo/share/gz/models/OakD-Lite/model.sdf; then
  docker exec "$NAME" sed -i 's/type="depth_camera"/type="rgbd_camera"/' /opt/px4-gazebo/share/gz/models/OakD-Lite/model.sdf
  echo "[extras] OakD-Lite patched to rgbd_camera"
  changed=1
fi

if ! docker exec "$NAME" python3 -c 'import cv2' 2>/dev/null; then
  echo "[extras] installing python3-opencv (takes a minute) ..."
  docker exec "$NAME" bash -c 'apt-get update -qq >/dev/null && apt-get install -y -qq python3-opencv >/dev/null'
  changed=1
fi

if [[ "$changed" == 1 ]]; then
  echo "[extras] restarting $NAME to apply"
  docker restart "$NAME" >/dev/null
fi
echo "[extras] done"
