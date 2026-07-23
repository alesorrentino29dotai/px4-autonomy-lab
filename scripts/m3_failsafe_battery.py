#!/usr/bin/env python3
"""M3 — Battery-low failsafe test: drain the simulated battery in flight.

WHAT IT DOES
      1. reads and prints the relevant PX4 failsafe parameters:
           BAT_LOW_THR     battery % that raises the LOW warning
           BAT_CRIT_THR    battery % that triggers COM_LOW_BAT_ACT
           BAT_EMERGEN_THR battery % that forces immediate LAND
           COM_LOW_BAT_ACT action at critical level (0 warn, 2 land, 3 RTL)
      2. configures the SITL battery simulator to drain fully:
           SIM_BAT_MIN_PCT = 0  (by default SITL never drains below 50%)
      3. takes off to 10 m and hovers while logging battery % + flight mode
      4. reports the mode transitions as thresholds are crossed
         (expected with defaults: warning at LOW, RTL/LAND at CRITICAL)
      5. waits for touchdown/disarm, then restores SIM_BAT_MIN_PCT

MAVLINK MESSAGES INVOLVED
    PARAM_REQUEST_READ / PARAM_SET / PARAM_VALUE — parameter microservice
    SYS_STATUS / BATTERY_STATUS — battery telemetry
    HEARTBEAT — flight-mode transitions (the observable failsafe effect)
    STATUSTEXT — human-readable failsafe warnings from the commander

USAGE
    ./scripts/run_sitl.sh                                   # terminal 1
    .venv/bin/python scripts/m3_failsafe_battery.py         # terminal 2
"""

import asyncio
import sys

from mavsdk import System


async def main() -> int:
    drone = System()
    print("Connecting to udpin://0.0.0.0:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to PX4")
            break

    for p in ("BAT_LOW_THR", "BAT_CRIT_THR", "BAT_EMERGEN_THR"):
        v = await drone.param.get_param_float(p)
        print(f"  {p} = {v*100:.0f}%")
    act = await drone.param.get_param_int("COM_LOW_BAT_ACT")
    print(f"  COM_LOW_BAT_ACT = {act} (0=warn 1=return 2=land 3=return-or-land)")

    old_min = await drone.param.get_param_float("SIM_BAT_MIN_PCT")
    await drone.param.set_param_float("SIM_BAT_MIN_PCT", 0.0)
    print(f"  SIM_BAT_MIN_PCT: {old_min:.0f} -> 0 (allow full drain)")

    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            break

    print("Arming & taking off to 10 m ...")
    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(12)

    print("Hovering — watching battery drain and flight mode ...")
    last_mode, last_pct = None, None
    landed = False

    async def watch() -> None:
        nonlocal last_mode, last_pct, landed
        mode_iter = drone.telemetry.flight_mode()
        bat_iter = drone.telemetry.battery()
        armed_iter = drone.telemetry.armed()

        async def modes():
            nonlocal last_mode
            async for m in mode_iter:
                if str(m) != last_mode:
                    last_mode = str(m)
                    print(f"  >> flight mode: {last_mode}  (battery {last_pct})")

        async def battery():
            nonlocal last_pct
            async for b in bat_iter:
                pct = round(b.remaining_percent)
                if pct != last_pct:
                    last_pct = pct
                    if pct % 10 == 0 or pct <= 20:
                        print(f"  battery: {pct}%")

        async def armed():
            nonlocal landed
            async for a in armed_iter:
                if not a:
                    landed = True
                    return

        await asyncio.gather(modes(), battery(), armed())

    try:
        await asyncio.wait_for(watch(), timeout=300)
    except asyncio.TimeoutError:
        pass

    if landed:
        print("✓ Failsafe sequence ended on the ground (disarmed)")
    else:
        print("✗ Timed out before touchdown")

    await drone.param.set_param_float("SIM_BAT_MIN_PCT", old_min)
    print(f"  SIM_BAT_MIN_PCT restored to {old_min:.0f}")
    return 0 if landed else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
