#!/usr/bin/env python3
"""M7 demo 1 — Precision landing on an AprilTag with GPU detection (Isaac ROS).

WHAT IT DOES
    The M5 concept upgraded to a production-grade perception stack:

      Gazebo camera ─ ros_gz_bridge ─► isaac_ros_apriltag (CUDA, NITROS)
                                            │ /tag_detections (metric 3D pose)
                                            ▼ tag_bridge.py (UDP JSON)
                              this script: MAVSDK Offboard controller

      1. arm + takeoff to 12 m over the baylands world
      2. offboard reposition ~8 m N, 6 m E (away from the pad at 5,3)
      3. descent loop @ 10 Hz on the tag pose:
           camera optical frame → BODY FRD (down camera, image-up = nose):
             (fwd, right, down) = (-y, +x, +z)
           then BODY → NED with the FULL attitude quaternion from telemetry.
           Attitude compensation is not optional: when the vehicle pitches
           15° while maneuvering, an uncompensated nadir camera "sees" the
           tag ~2.7 m off at 10 m height — the controller chases its own
           tilt and limit-cycles (observed!). Rotating by the quaternion
           makes the measurement tilt-invariant. (Yaw-only rotation fixes
           heading but keeps the tilt bug.)
           lateral P (Kp 0.6, clamp 1.5 m/s), descend only while centered
           (radial < max(0.4 m, 8 % of height)), taper below 3 m
      4. below 1.2 m above the tag → action.land() endgame
      5. report the touchdown offset using the known pad position

    Differences vs M5: the detector runs on the RTX GPU, the tag pose is
    already metric (PnP from tag size — no altitude needed), and the link
    crosses a real ROS 2 graph instead of an ad-hoc socket from the sim.

MAVLINK MESSAGES INVOLVED
    SET_POSITION_TARGET_LOCAL_NED (position, then velocity+yaw setpoints)
    COMMAND_LONG: ARM / NAV_TAKEOFF / NAV_LAND / DO_SET_MODE
    LOCAL_POSITION_NED — touchdown offset measurement

USAGE
    ./scripts/run_sitl.sh  (MODEL=gz_x500_mono_cam_down WORLD=baylands)   # t1
    WORLD=baylands MODEL=x500_mono_cam_down_0 ./sim/run_ros2_bridge.sh    # t2
    ./sim/spawn_apriltag.sh baylands 5 3
    docker exec -d isaac bash -c "source /opt/ros/jazzy/setup.bash && \
        ros2 launch /lab/ros2/apriltag_pipeline.launch.py"                # t3
    docker exec -d isaac bash -c "source /opt/ros/jazzy/setup.bash && \
        python3 /lab/ros2/tag_bridge.py"                                  # t4
    .venv/bin/python scripts/m7_apriltag_land.py                          # t5
"""

import asyncio
import json
import math
import socket
import sys
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

TAKEOFF_ALT = 12.0
START_N, START_E = 8.0, 6.0
PAD_N, PAD_E = 3.0, 5.0            # pad at gz world (x=5, y=3): gz is ENU
                                   # (x East, y North) -> NED (3 N, 5 E)
KP = 0.6
V_LAT_MAX = 1.5
V_DOWN_HIGH, V_DOWN_LOW = 0.8, 0.3
FINAL_LAND_H = 1.2                  # height above tag for the endgame
CV_PORT = 18700
CV_STALE_S = 0.6


def quat_rotate(q: list[float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate vector v by unit quaternion q=(w,x,y,z): body FRD -> NED."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * q_vec × v ; v' = v + w*t + q_vec × t
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


class TagLink:
    """Non-blocking reader of tag_bridge.py's UDP stream (keeps freshest)."""

    def __init__(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", CV_PORT))
        self.sock.setblocking(False)
        self.last: dict = {}
        self.rx = 0.0

    def poll(self) -> dict | None:
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.last = json.loads(data)
                self.rx = time.time()
            except BlockingIOError:
                break
        if time.time() - self.rx > CV_STALE_S or not self.last:
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

    link = TagLink()
    print("Waiting for the tag_bridge stream on udp:18700 ...")
    while link.poll() is None:
        await asyncio.sleep(0.5)
    print("✓ Perception stream alive (Isaac ROS AprilTag)")

    yaw_deg = [0.0]
    quat = [1.0, 0.0, 0.0, 0.0]          # w, x, y, z — body FRD -> NED

    async def track_yaw() -> None:
        async for e in drone.telemetry.attitude_euler():
            yaw_deg[0] = e.yaw_deg

    async def track_quat() -> None:
        async for q in drone.telemetry.attitude_quaternion():
            quat[0], quat[1], quat[2], quat[3] = q.w, q.x, q.y, q.z

    yaw_task = asyncio.create_task(track_yaw())
    quat_task = asyncio.create_task(track_quat())

    print(f"Arming & taking off to {TAKEOFF_ALT:.0f} m ...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.arm()
    await drone.action.takeoff()
    async for p in drone.telemetry.position():
        if abs(p.relative_altitude_m - TAKEOFF_ALT) < 0.6:
            break

    hold_yaw = yaw_deg[0]   # keep whatever heading we took off with
    print(f"Offboard: repositioning to ({START_N:.0f} N, {START_E:.0f} E), "
          f"holding yaw {hold_yaw:.0f}° ...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(START_N, START_E, -TAKEOFF_ALT, hold_yaw))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"✗ offboard refused: {e._result.result_str}")
        await drone.action.land()
        return 1
    await asyncio.sleep(9)

    print("GPU-guided descent engaged")
    lost_since = None
    t_log = 0.0
    height = TAKEOFF_ALT

    async def descent_loop() -> None:
        nonlocal lost_since, t_log, height
        while True:
            det = link.poll()
            if det and det["detected"]:
                lost_since = None
                v_body = (-det["y"], det["x"], det["z"])      # camera -> FRD
                north, east, height = quat_rotate(quat, v_body)
                if height < 0.3:
                    await asyncio.sleep(0.05)
                    continue                                   # degenerate geometry
                radial = math.hypot(north, east)
                vn = max(-V_LAT_MAX, min(V_LAT_MAX, KP * north))
                ve = max(-V_LAT_MAX, min(V_LAT_MAX, KP * east))
                centered = radial < max(0.4, 0.08 * height)
                vd = (V_DOWN_HIGH if height > 3 else V_DOWN_LOW) if centered else 0.0
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(vn, ve, vd, hold_yaw))
                if time.time() - t_log > 1.5:
                    print(f"  h {height:4.1f} m  offset {radial:4.2f} m "
                          f"({north:+.2f} N {east:+.2f} E)  {'↓' if vd else '·'}")
                    t_log = time.time()
                if height < FINAL_LAND_H and centered:
                    print("✓ Centered over the pad — final landing")
                    return
            else:
                if lost_since is None:
                    lost_since = time.time()
                elif time.time() - lost_since > 1.0:
                    if height < FINAL_LAND_H + 0.6:
                        print("tag overflowed FOV near the pad — landing")
                        return
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0, 0, 0, hold_yaw))
            await asyncio.sleep(0.1)

    try:
        await descent_loop()
    finally:
        # never leave PX4 stranded in OFFBOARD without a setpoint stream:
        # that state blocks re-arming ("no offboard signal") until a mode change
        try:
            await drone.offboard.stop()
        except Exception:
            pass
        try:
            await drone.action.land()
        except Exception:
            pass

    async for armed in drone.telemetry.armed():
        if not armed:
            break
    yaw_task.cancel()
    quat_task.cancel()

    async for pv in drone.telemetry.position_velocity_ned():
        n, e = pv.position.north_m, pv.position.east_m
        err = math.hypot(n - PAD_N, e - PAD_E)
        print(f"✓ TOUCHDOWN — offset from pad center: {err:.2f} m "
              f"(pos {n:+.2f} N {e:+.2f} E, pad {PAD_N} N {PAD_E} E)")
        break
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
