#!/usr/bin/env bash
# Patch the OakD-Lite model inside the px4-sitl container: turn the
# depth-only sensor into an rgbd_camera so Gazebo publishes color + depth
# ALIGNED from the same optics (what cuVSLAM's RGBD mode expects).
#
# Idempotent: safe to re-run (e.g. after recreating the container).
#   ./sim/patch_oakd_rgbd.sh [container-name]
set -euo pipefail
NAME="${1:-px4-sitl}"
docker exec "$NAME" bash -c '
  SDF=/opt/px4-gazebo/share/gz/models/OakD-Lite/model.sdf
  if grep -q "type=\"rgbd_camera\"" "$SDF"; then
    echo "already patched"
  else
    sed -i "s/type=\"depth_camera\"/type=\"rgbd_camera\"/" "$SDF"
    echo "patched: depth_camera -> rgbd_camera"
  fi
  grep -o "type=\"rgbd_camera\"" "$SDF"
'
