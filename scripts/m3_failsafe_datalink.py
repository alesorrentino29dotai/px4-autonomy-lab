#!/usr/bin/env python3
"""M3 — Datalink-loss failsafe test: cut all GCS heartbeats mid-flight.

WHAT IT DOES
    PX4 declares "datalink lost" when no GCS-type HEARTBEAT arrives for
    COM_DL_LOSS_T seconds; the reaction is NAV_DLL_ACT (0 disabled, 2 RTL,
    3 land...). This script demonstrates it end-to-end:

      phase A (MAVSDK, this process spawns it as a subprocess):
        - set NAV_DLL_ACT = 2 (Return) and COM_DL_LOSS_T = 10 s
        - arm, take off to 15 m
        - exit abruptly -> the MAVSDK heartbeats stop (simulated link loss)
      phase B (pymavlink, PASSIVE — sends nothing):
        - listen on udp 14550 and watch HEARTBEAT.custom_mode
        - expect: ~10 s after phase A dies, mode switches to AUTO.RTL,
          the drone returns, lands, disarms
        - restore NAV_DLL_ACT afterwards

    IMPORTANT: close QGroundControl first — its heartbeats keep the link
    "alive" and the failsafe will (correctly) never trigger. The passive
    listener also needs port 14550 free.

MAVLINK MESSAGES INVOLVED
    HEARTBEAT     — its *absence* is what triggers the failsafe
    PARAM_SET     — NAV_DLL_ACT / COM_DL_LOSS_T configuration
    HEARTBEAT.custom_mode (PX4 encoding: main mode AUTO, sub-mode RTL) —
                    observed passively to detect the reaction

USAGE
    ./scripts/run_sitl.sh                                    # terminal 1
    .venv/bin/python scripts/m3_failsafe_datalink.py         # terminal 2
"""

import asyncio
import subprocess
import sys
import time

PHASE_A = r"""
import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for s in drone.core.connection_state():
        if s.is_connected: break
    await drone.param.set_param_int("NAV_DLL_ACT", 2)   # 2 = Return on datalink loss
    await drone.param.set_param_int("COM_DL_LOSS_T", 10)
    print("params set: NAV_DLL_ACT=2 COM_DL_LOSS_T=10", flush=True)
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok: break
    await drone.action.set_takeoff_altitude(15.0)
    await drone.action.arm()
    await drone.action.takeoff()
    print("airborne, climbing to 15 m", flush=True)
    await asyncio.sleep(15)
    print("phase A exiting NOW -> heartbeats stop", flush=True)

asyncio.run(main())
"""

# PX4 custom_mode encoding: main_mode in byte 2, sub_mode in byte 3
PX4_AUTO = 4
PX4_AUTO_RTL = 5
PX4_AUTO_LAND = 6


def decode_mode(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    if main == PX4_AUTO:
        return {2: "AUTO.TAKEOFF", 3: "AUTO.HOLD", 4: "AUTO.MISSION",
                PX4_AUTO_RTL: "AUTO.RTL", PX4_AUTO_LAND: "AUTO.LAND"}.get(sub, f"AUTO.{sub}")
    return f"main={main} sub={sub}"


RESTORE = r"""
import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for s in drone.core.connection_state():
        if s.is_connected: break
    await drone.param.set_param_int("NAV_DLL_ACT", 0)
    print("NAV_DLL_ACT restored to 0", flush=True)

asyncio.run(main())
"""


def main() -> int:
    print("== phase A: takeoff via MAVSDK, then drop the link ==", flush=True)
    a = subprocess.run([sys.executable, "-c", PHASE_A], timeout=120)
    if a.returncode != 0:
        print("✗ phase A failed")
        return 1

    print("== phase B: passive watch on udp 14550 (no heartbeats sent) ==", flush=True)
    from pymavlink import mavutil
    m = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

    t0 = time.time()
    last = None
    rtl_seen = False
    disarmed_after_rtl = False
    while time.time() - t0 < 240:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg is None or msg.get_srcComponent() != 1:
            continue
        mode = decode_mode(msg.custom_mode)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        state = f"{mode} armed={armed}"
        if state != last:
            last = state
            print(f"  t+{time.time()-t0:5.1f}s  {state}")
        if mode == "AUTO.RTL":
            rtl_seen = True
        if rtl_seen and not armed:
            disarmed_after_rtl = True
            break

    if disarmed_after_rtl:
        print("✓ Datalink-loss failsafe verified: RTL engaged and landed")
    else:
        print(f"✗ Expected AUTO.RTL then disarm (rtl_seen={rtl_seen})")

    m.close()  # free 14550 so phase C's link (and later QGC) can use it
    print("== phase C: restore NAV_DLL_ACT ==", flush=True)
    subprocess.run([sys.executable, "-c", RESTORE], timeout=60)
    return 0 if disarmed_after_rtl else 1


if __name__ == "__main__":
    sys.exit(main())
