#!/usr/bin/env python3
"""M2 — Full basic flight: preflight checks → arm → takeoff 10 m → hold 10 s → land.

WHAT IT DOES
    Runs the canonical minimal mission with *explicit* error handling at each
    step, printing WHY a step failed instead of just crashing:

      1. wait for MAVSDK connection and for the vehicle to become healthy
         (EKF global/home position OK) — with a timeout
      2. arm            — ActionError explained (e.g. COMMAND_DENIED means a
                          preflight check failed: no GCS heartbeat seen yet,
                          EKF not converged, safety switch, RC loss, ...)
      3. takeoff to 10 m (set via param-like MAVSDK setting, then monitored
                          until within 0.5 m of the target)
      4. hold 10 s
      5. land + wait until the flight controller reports touchdown and disarms

MAVLINK MESSAGES / COMMANDS INVOLVED
    HEARTBEAT                        — connection & mode tracking
    COMMAND_LONG:
        MAV_CMD_COMPONENT_ARM_DISARM — arm/disarm request
        MAV_CMD_NAV_TAKEOFF          — climb to MIS_TAKEOFF_ALT / set altitude
        MAV_CMD_NAV_LAND             — descend & land at current position
    COMMAND_ACK                      — result of each command (ACCEPTED /
                                       DENIED / TEMPORARILY_REJECTED...): this
                                       is what MAVSDK turns into ActionError
    GLOBAL_POSITION_INT              — altitude monitoring
    EXTENDED_SYS_STATE               — MAV_LANDED_STATE (in-air / on-ground),
                                       used by telemetry.in_air()

WHAT HAPPENS IF ARM IS REFUSED?
    PX4's commander runs "health & arming checks" (GCS connected, EKF variance,
    battery, geofence, ...). A refusal comes back as COMMAND_ACK(DENIED) and a
    STATUSTEXT explaining the reason — visible in QGC or the pxh> console.
    MAVSDK raises ActionError(COMMAND_DENIED); this script catches it and tells
    you where to look.

USAGE
    ./scripts/run_sitl.sh                              # terminal 1
    .venv/bin/python scripts/m2_takeoff_land.py        # terminal 2
"""

import asyncio
import sys

from mavsdk import System
from mavsdk.action import ActionError

TAKEOFF_ALT_M = 10.0
HOLD_S = 10.0
HEALTH_TIMEOUT_S = 60.0


async def wait_until_healthy(drone: System) -> None:
    """Block until EKF reports valid global+home position (or time out)."""
    print(f"Waiting for vehicle health (timeout {HEALTH_TIMEOUT_S:.0f}s) ...")

    async def _healthy() -> None:
        async for h in drone.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                return

    await asyncio.wait_for(_healthy(), timeout=HEALTH_TIMEOUT_S)
    print("✓ EKF healthy: global & home position OK")


async def wait_for_altitude(drone: System, target_m: float, tol_m: float = 0.5) -> None:
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        if abs(alt - target_m) < tol_m:
            print(f"✓ Reached {alt:.1f} m")
            return


async def wait_until_disarmed(drone: System) -> None:
    async for armed in drone.telemetry.armed():
        if not armed:
            return


async def main() -> int:
    drone = System()
    print("Connecting to udpin://0.0.0.0:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to PX4")
            break

    try:
        await wait_until_healthy(drone)
    except asyncio.TimeoutError:
        print("✗ Vehicle never became healthy — is the EKF getting GPS? "
              "Check `ekf2 status` / `commander check` in the pxh> console.")
        return 1

    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)

    try:
        print("Arming ...")
        await drone.action.arm()
        print("✓ Armed")
    except ActionError as e:
        print(f"✗ Arm refused: {e._result.result_str}")
        print("  Common causes: no GCS heartbeat yet, EKF not converged, "
              "RC loss failsafe, battery check. See STATUSTEXT in QGC/pxh>.")
        return 1

    try:
        print(f"Taking off to {TAKEOFF_ALT_M:.0f} m ...")
        await drone.action.takeoff()
        await asyncio.wait_for(wait_for_altitude(drone, TAKEOFF_ALT_M), timeout=60)
    except (ActionError, asyncio.TimeoutError) as e:
        print(f"✗ Takeoff failed: {e}")
        return 1

    print(f"Holding for {HOLD_S:.0f} s ...")
    await asyncio.sleep(HOLD_S)

    try:
        print("Landing ...")
        await drone.action.land()
        await asyncio.wait_for(wait_until_disarmed(drone), timeout=120)
        print("✓ Touchdown — vehicle disarmed")
    except (ActionError, asyncio.TimeoutError) as e:
        print(f"✗ Land failed: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
