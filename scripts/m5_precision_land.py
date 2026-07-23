#!/usr/bin/env python3
"""M5 (flight side) — Precision landing on an ArUco marker in Offboard mode.

WHAT IT DOES
    Closes the vision loop started by m5_cv_node.py (which streams marker
    offsets as JSON over UDP 18700):

      1. arm, take off to 8 m — the drone spawns on the marker, so it then
         flies a ~5 m lateral offset in Offboard (position setpoint) to make
         the approach non-trivial
      2. DESCEND loop @ 10 Hz, driven by the CV offsets:
           - angles → metric offset:  north = -tan(ang_y)·alt,
                                      east  = +tan(ang_x)·alt
             (down-facing camera, image "up" = vehicle nose, yaw held at 0)
           - lateral P controller:    vn = Kp·north, ve = Kp·east  (clamped)
           - descend only while centered (offset < max(0.4 m, 8% of alt)),
             descent rate tapered near the ground
           - marker lost > 1 s while high → hold; regained → continue
      3. below 1 m altitude and centered: switch to action.land() for the
         final touchdown (the marker overflows the FOV that close — classic
         drone-in-a-box endgame)
      4. after disarm, reads the local NED position: distance from origin
         ≈ landing error w.r.t. the marker (marker sits at the world origin,
         which is also the takeoff/home point)

OFFBOARD / CONTROL NOTES
    Velocity setpoints (SET_POSITION_TARGET_LOCAL_NED, velocity+yaw fields)
    at 10 Hz — above the 2 Hz minimum PX4 requires to stay in OFFBOARD.
    The camera pitch mount (+90°) maps image axes to body axes as:
    u right → body right (east at yaw 0), v down → body rear (−north).

MAVLINK MESSAGES INVOLVED
    SET_POSITION_TARGET_LOCAL_NED (velocity+yaw / position setpoints)
    COMMAND_LONG: ARM, NAV_TAKEOFF, NAV_LAND, DO_SET_MODE (offboard switch)
    LOCAL_POSITION_NED — landing-error measurement
    EXTENDED_SYS_STATE — touchdown detection

USAGE
    ./scripts/run_sitl.sh   with MODEL=gz_x500_mono_cam_down WORLD=aruco   # t1
    docker exec -d px4-sitl python3 /lab/scripts/m5_cv_node.py             # t2
    .venv/bin/python scripts/m5_precision_land.py                          # t3
"""

import asyncio
import json
import math
import socket
import sys
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

TAKEOFF_ALT = 8.0
OFFSET_N, OFFSET_E = 4.0, 3.0      # initial displacement from the marker (m)
KP = 0.6                            # lateral P gain (1/s)
V_LAT_MAX = 1.5                     # m/s
V_DOWN_HIGH = 0.7                   # m/s descent above 3 m
V_DOWN_LOW = 0.3                    # m/s descent below 3 m
FINAL_LAND_ALT = 1.0                # switch to action.land() below this
CV_PORT = 18700
CV_STALE_S = 0.5


class CvLink:
    """Non-blocking reader of the CV node's UDP JSON stream (keeps freshest)."""

    def __init__(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", CV_PORT))
        self.sock.setblocking(False)
        self.last: dict = {}
        self.rx_time = 0.0

    def poll(self) -> dict | None:
        """Drain the socket; return the freshest detection or None if stale."""
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.last = json.loads(data)
                self.rx_time = time.time()
            except BlockingIOError:
                break
        if time.time() - self.rx_time > CV_STALE_S or not self.last:
            return None
        return self.last


async def main() -> int:
    drone = System()
    print("Connecting to udpin://0.0.0.0:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for s in drone.core.connection_state():
        if s.is_connected:
            print("✓ Connected to PX4")
            break
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            break

    cv = CvLink()
    if cv.poll() is None:
        print("… waiting for the CV node stream on udp:18700 "
              "(docker exec -d px4-sitl python3 /lab/scripts/m5_cv_node.py)")
        while cv.poll() is None:
            await asyncio.sleep(0.5)
    print("✓ CV stream alive")

    alt = 0.0

    async def track_alt() -> None:
        nonlocal alt
        async for p in drone.telemetry.position():
            alt = p.relative_altitude_m

    alt_task = asyncio.create_task(track_alt())

    print(f"Arming & taking off to {TAKEOFF_ALT:.0f} m ...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.arm()
    await drone.action.takeoff()
    while abs(alt - TAKEOFF_ALT) > 0.5:
        await asyncio.sleep(0.2)

    print(f"Offboard: moving {OFFSET_N:.0f} m N, {OFFSET_E:.0f} m E of the marker ...")
    await drone.offboard.set_position_ned(PositionNedYaw(OFFSET_N, OFFSET_E, -TAKEOFF_ALT, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"✗ offboard refused: {e._result.result_str}")
        await drone.action.land()
        return 1
    await asyncio.sleep(8)

    print("Precision descent engaged")
    lost_since: float | None = None
    t_log = 0.0
    while True:
        det = cv.poll()
        if det and det["detected"]:
            lost_since = None
            north = -math.tan(det["ang_y"]) * alt
            east = math.tan(det["ang_x"]) * alt
            radial = math.hypot(north, east)
            vn = max(-V_LAT_MAX, min(V_LAT_MAX, KP * north))
            ve = max(-V_LAT_MAX, min(V_LAT_MAX, KP * east))
            centered = radial < max(0.4, 0.08 * alt)
            vd = (V_DOWN_HIGH if alt > 3 else V_DOWN_LOW) if centered else 0.0
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, 0.0))
            if time.time() - t_log > 1.5:
                print(f"  alt {alt:4.1f} m  offset {radial:4.2f} m "
                      f"({north:+.2f} N {east:+.2f} E)  {'↓' if vd else '·'}")
                t_log = time.time()
            if alt < FINAL_LAND_ALT and centered:
                print("✓ Centered at low altitude — final landing")
                break
        else:
            if lost_since is None:
                lost_since = time.time()
            elif time.time() - lost_since > 1.0:
                if alt < FINAL_LAND_ALT + 0.5:
                    print("marker overflowed FOV near ground — landing")
                    break
                print(f"  marker lost at {alt:.1f} m — holding")
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        await asyncio.sleep(0.1)

    await drone.offboard.stop()
    await drone.action.land()
    async for armed in drone.telemetry.armed():
        if not armed:
            break
    alt_task.cancel()

    async for pv in drone.telemetry.position_velocity_ned():
        n, e = pv.position.north_m, pv.position.east_m
        err = math.hypot(n, e)
        print(f"✓ TOUCHDOWN — landing error vs marker: {err:.2f} m "
              f"({n:+.2f} N, {e:+.2f} E)")
        break
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
