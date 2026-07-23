#!/usr/bin/env python3
"""M3 — Waypoint mission: upload a 4-waypoint square, fly it, RTL home.

WHAT IT DOES
      1. reads the home position from telemetry
      2. builds a 4-waypoint square mission (80 m sides, 15 m AGL) around home,
         computed by offsetting lat/lon locally (small-angle approximation:
         1 deg lat ≈ 111 320 m; lon scaled by cos(lat))
      3. uploads it, enables RTL-after-mission, arms and starts the mission
      4. prints mission progress as waypoints are reached
      5. waits for RTL touchdown + auto-disarm

MAVLINK MESSAGES INVOLVED (mission microservice)
    Upload is a *protocol*, not a single message — the "mission microservice":
        MISSION_COUNT (GCS→FC) announces N items
        MISSION_REQUEST_INT (FC→GCS) asks for item i
        MISSION_ITEM_INT (GCS→FC) one waypoint (lat/lon ×1e7, MAV_CMD_NAV_WAYPOINT)
        MISSION_ACK closes the handshake
    Execution:
        MAV_CMD_MISSION_START, MISSION_CURRENT / MISSION_ITEM_REACHED (progress)
        HEARTBEAT custom_mode → AUTO.MISSION, then AUTO.RTL
    RTL behaviour is governed by PX4 params: RTL_RETURN_ALT (climb-to altitude),
    RTL_DESCEND_ALT, RTL_LAND_DELAY (hover before final descent).

USAGE
    ./scripts/run_sitl.sh                          # terminal 1
    .venv/bin/python scripts/m3_mission.py         # terminal 2
"""

import asyncio
import math
import sys

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

ALT_M = 15.0
SIDE_M = 80.0
SPEED_M_S = 5.0


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Offset a lat/lon by meters using a local flat-earth approximation."""
    dlat = north_m / 111_320.0
    dlon = east_m / (111_320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def waypoint(lat: float, lon: float) -> MissionItem:
    return MissionItem(
        lat, lon, ALT_M,
        SPEED_M_S,
        True,                       # fly-through (don't stop at each corner)
        float("nan"), float("nan"), # gimbal pitch/yaw: unused
        MissionItem.CameraAction.NONE,
        float("nan"),               # loiter time
        float("nan"),               # camera photo interval
        float("nan"),               # acceptance radius: PX4 default (NAV_ACC_RAD)
        float("nan"),               # yaw
        float("nan"),               # camera photo distance
        MissionItem.VehicleAction.NONE,
    )


async def main() -> int:
    drone = System()
    print("Connecting to udpin://0.0.0.0:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to PX4")
            break

    print("Waiting for home position ...")
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            break
    async for home in drone.telemetry.home():
        lat0, lon0 = home.latitude_deg, home.longitude_deg
        break
    print(f"✓ Home: {lat0:.6f}, {lon0:.6f}")

    half = SIDE_M / 2
    corners = [(+half, +half), (+half, -half), (-half, -half), (-half, +half)]
    items = [waypoint(*offset(lat0, lon0, n, e)) for n, e in corners]

    print(f"Uploading mission: {len(items)} waypoints, {SIDE_M:.0f} m square @ {ALT_M:.0f} m AGL")
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(MissionPlan(items))
    print("✓ Mission uploaded")

    print("Arming & starting mission ...")
    await drone.action.arm()
    await drone.mission.start_mission()

    async def watch_progress() -> None:
        async for p in drone.mission.mission_progress():
            print(f"  waypoint {p.current}/{p.total}")
            if p.current == p.total:
                return

    await asyncio.wait_for(watch_progress(), timeout=300)
    print("✓ Mission complete — RTL engaged")

    async for mode in drone.telemetry.flight_mode():
        print(f"  mode: {mode}")
        break

    async def until_disarmed() -> None:
        async for armed in drone.telemetry.armed():
            if not armed:
                return

    await asyncio.wait_for(until_disarmed(), timeout=180)
    print("✓ RTL touchdown — disarmed")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
