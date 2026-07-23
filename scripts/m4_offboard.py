#!/usr/bin/env python3
"""M4 — Offboard control: fly a square and a circle with velocity setpoints.

WHAT IT DOES
    Demonstrates PX4 OFFBOARD mode — the mode a companion computer uses to
    control the vehicle programmatically (the conceptual bridge to M5):

      1. arm + takeoff to 10 m (Action API, as in M2)
      2. start streaming velocity setpoints, then switch to OFFBOARD
      3. SQUARE: 4 legs of 20 m at 2 m/s in the NED frame, yaw facing travel
      4. CIRCLE: radius ≈ 10 m at 2 m/s, one full revolution, nose on tangent
         (velocity vector rotated continuously at 20 Hz)
      5. stop offboard → land

OFFBOARD MODE RULES (why the code looks like this)
    - PX4 accepts OFFBOARD only if a setpoint stream is ALREADY flowing
      (> 2 Hz): MAVSDK's offboard.start() sends one setpoint first for this
      reason, and we keep streaming at 20 Hz.
    - If the stream stops for COM_OF_LOSS_T seconds, the offboard-loss
      failsafe (COM_OBL_RC_ACT) kicks in — same philosophy as the datalink
      failsafe in M3.

MAVLINK MESSAGES INVOLVED
    SET_POSITION_TARGET_LOCAL_NED — every setpoint; a type_mask selects which
        fields are active (here: velocity + yaw, position ignored). Frame is
        NED (North-East-Down, z positive DOWN).
    HEARTBEAT custom_mode → OFFBOARD while streaming
    COMMAND_LONG(MAV_CMD_DO_SET_MODE) — the offboard.start()/stop() switch

USAGE
    ./scripts/run_sitl.sh                            # terminal 1
    .venv/bin/python scripts/m4_offboard.py          # terminal 2
"""

import asyncio
import math
import sys

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

ALT_M = 10.0
SPEED = 2.0          # m/s
SIDE_M = 20.0
RADIUS_M = 10.0
RATE_HZ = 20.0


async def fly_square(drone: System) -> None:
    """Four straight legs in NED: north, east, south, west."""
    legs = [(SPEED, 0.0, 0.0), (0.0, SPEED, 90.0),
            (-SPEED, 0.0, 180.0), (0.0, -SPEED, 270.0)]
    leg_t = SIDE_M / SPEED
    for i, (vn, ve, yaw) in enumerate(legs, 1):
        print(f"  square leg {i}/4 (yaw {yaw:.0f}°)")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, 0.0, yaw))
        await asyncio.sleep(leg_t)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 270))
    await asyncio.sleep(2)


async def fly_circle(drone: System) -> None:
    """One revolution: rotate the velocity vector continuously at RATE_HZ."""
    period = 2 * math.pi * RADIUS_M / SPEED           # ~31 s per revolution
    omega = 2 * math.pi / period                      # rad/s
    steps = int(period * RATE_HZ)
    print(f"  circle: r={RADIUS_M:.0f} m, one revolution in {period:.0f} s")
    for i in range(steps):
        theta = omega * i / RATE_HZ
        vn = SPEED * math.cos(theta)
        ve = SPEED * math.sin(theta)
        yaw = math.degrees(theta) % 360               # nose on the tangent
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, 0.0, yaw))
        await asyncio.sleep(1.0 / RATE_HZ)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await asyncio.sleep(2)


async def main() -> int:
    drone = System()
    print("Connecting to udpin://0.0.0.0:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to PX4")
            break

    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            break

    print(f"Arming & taking off to {ALT_M:.0f} m ...")
    await drone.action.set_takeoff_altitude(ALT_M)
    await drone.action.arm()
    await drone.action.takeoff()
    async for pos in drone.telemetry.position():
        if abs(pos.relative_altitude_m - ALT_M) < 0.5:
            break
    print("✓ At altitude")

    # a setpoint must be in flight BEFORE offboard start, or PX4 rejects the switch
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    try:
        await drone.offboard.start()
        print("✓ OFFBOARD engaged")
    except OffboardError as e:
        print(f"✗ Offboard start refused: {e._result.result_str}")
        await drone.action.land()
        return 1

    await fly_square(drone)
    await fly_circle(drone)

    await drone.offboard.stop()
    print("Landing ...")
    await drone.action.land()
    async for armed in drone.telemetry.armed():
        if not armed:
            break
    print("✓ Landed & disarmed")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
