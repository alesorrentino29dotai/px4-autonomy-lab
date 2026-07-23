#!/usr/bin/env bash
# Spawn the AprilTag landing pad into the running Gazebo world.
#   ./sim/spawn_apriltag.sh [world] [x] [y]
# Defaults: world=baylands, position 5 m N, 3 m E of the origin.
set -euo pipefail
WORLD="${1:-baylands}"
X="${2:-5}"
Y="${3:-3}"
docker exec px4-sitl gz service -s "/world/${WORLD}/create" \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req "sdf_filename: \"/lab/sim/models/apriltag_36h11_0/model.sdf\", name: \"apriltag_pad\", pose: {position: {x: ${X}, y: ${Y}, z: 0.02}}"
echo "apriltag_pad spawned at (${X}, ${Y}) in ${WORLD}"
