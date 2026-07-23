#!/usr/bin/env python3
"""Sanity check: connect to PX4 SITL over MAVSDK and print vehicle state.

WHAT IT DOES
    Connects to the SITL "onboard" MAVLink link (udpin://0.0.0.0:14540 — the port PX4
    reserves for companion computers / APIs, while 14550 is for the GCS),
    waits for the connection, then prints:
      - connection state
      - whether the vehicle passes the global-position health check
      - armed state and flight mode

MAVLINK MESSAGES INVOLVED (under the MAVSDK hood)
    HEARTBEAT           — connection discovery & flight mode (custom_mode)
    SYS_STATUS          — sensor health bitmask
    GLOBAL_POSITION_INT — used by Telemetry.health for the position check

USAGE
    ./scripts/run_sitl.sh          # in another terminal (or headless)
    .venv/bin/python scripts/check_connection.py
"""

import asyncio

from mavsdk import System


async def main() -> None:
    drone = System()
    print("Connecting to udpin://0.0.0.0:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to PX4")
            break

    async for health in drone.telemetry.health():
        print(f"  global position ok: {health.is_global_position_ok}")
        print(f"  home position ok:   {health.is_home_position_ok}")
        print(f"  armable:            {health.is_armable}")
        break

    async for armed in drone.telemetry.armed():
        print(f"  armed:              {armed}")
        break

    async for mode in drone.telemetry.flight_mode():
        print(f"  flight mode:        {mode}")
        break


if __name__ == "__main__":
    asyncio.run(main())
