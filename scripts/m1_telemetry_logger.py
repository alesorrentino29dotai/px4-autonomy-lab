#!/usr/bin/env python3
"""M1 — Telemetry logger: stream PX4 telemetry over MAVSDK and log it to CSV.

WHAT IT DOES
    Connects to SITL on the onboard MAVLink link (udpin://0.0.0.0:14540) and
    subscribes to five telemetry streams concurrently (asyncio tasks). Once a
    second it snapshots the latest values into a CSV row:

        time, lat, lon, abs_alt_m, rel_alt_m, vn_m_s, ve_m_s, vd_m_s,
        battery_v, battery_pct, flight_mode, gps_ok, home_ok, armable

    Default output: logs/telemetry_<timestamp>.csv (dir created if missing).

MAVLINK MESSAGES INVOLVED (what MAVSDK subscribes to under the hood)
    GLOBAL_POSITION_INT — lat/lon (1e7 degrees), altitude AMSL and relative
                          altitude (mm), NED velocities (cm/s)
    VFR_HUD / ALTITUDE  — complements altitude data
    SYS_STATUS          — battery voltage/remaining, sensor health bitmask
    HEARTBEAT           — flight mode (base_mode/custom_mode, PX4-specific
                          encoding decoded by MAVSDK into FlightMode)
    These arrive at fixed stream rates configured by PX4's mavlink module
    (see `mavlink status` in the pxh> console).

WHY POLL-AND-SNAPSHOT INSTEAD OF LOGGING EVERY MESSAGE?
    Streams tick at different rates (position ~50 Hz in SITL, battery ~1 Hz).
    Snapshotting the latest value of each stream on a fixed clock gives a
    rectangular, analysis-friendly CSV — same approach as a flight-test
    telemetry recorder.

USAGE
    ./scripts/run_sitl.sh                                   # terminal 1
    .venv/bin/python scripts/m1_telemetry_logger.py         # terminal 2
    .venv/bin/python scripts/m1_telemetry_logger.py --duration 30 --rate 2
"""

import argparse
import asyncio
import csv
import datetime as dt
import pathlib

from mavsdk import System

FIELDS = [
    "time", "lat", "lon", "abs_alt_m", "rel_alt_m",
    "vn_m_s", "ve_m_s", "vd_m_s",
    "battery_v", "battery_pct", "flight_mode",
    "gps_ok", "home_ok", "armable",
]


class Latest:
    """Mutable snapshot of the most recent value of each telemetry stream."""

    def __init__(self) -> None:
        self.data: dict = {k: None for k in FIELDS}

    async def track_position(self, drone: System) -> None:
        async for p in drone.telemetry.position():
            self.data.update(
                lat=round(p.latitude_deg, 7),
                lon=round(p.longitude_deg, 7),
                abs_alt_m=round(p.absolute_altitude_m, 2),
                rel_alt_m=round(p.relative_altitude_m, 2),
            )

    async def track_velocity(self, drone: System) -> None:
        async for v in drone.telemetry.velocity_ned():
            self.data.update(
                vn_m_s=round(v.north_m_s, 2),
                ve_m_s=round(v.east_m_s, 2),
                vd_m_s=round(v.down_m_s, 2),
            )

    async def track_battery(self, drone: System) -> None:
        async for b in drone.telemetry.battery():
            self.data.update(
                battery_v=round(b.voltage_v, 2),
                battery_pct=round(b.remaining_percent, 1),
            )

    async def track_flight_mode(self, drone: System) -> None:
        async for m in drone.telemetry.flight_mode():
            self.data["flight_mode"] = str(m)

    async def track_health(self, drone: System) -> None:
        async for h in drone.telemetry.health():
            self.data.update(
                gps_ok=h.is_global_position_ok,
                home_ok=h.is_home_position_ok,
                armable=h.is_armable,
            )


async def main(duration_s: float, rate_hz: float, out_path: pathlib.Path) -> None:
    drone = System()
    print("Connecting to udpin://0.0.0.0:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to PX4")
            break

    latest = Latest()
    tasks = [
        asyncio.create_task(coro(drone))
        for coro in (latest.track_position, latest.track_velocity,
                     latest.track_battery, latest.track_flight_mode,
                     latest.track_health)
    ]

    while latest.data["lat"] is None:  # don't log rows until telemetry is flowing
        await asyncio.sleep(0.1)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = 0
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        t_end = asyncio.get_event_loop().time() + duration_s
        while asyncio.get_event_loop().time() < t_end:
            latest.data["time"] = dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")
            writer.writerow(latest.data)
            rows += 1
            await asyncio.sleep(1.0 / rate_hz)

    for t in tasks:
        t.cancel()
    print(f"✓ Wrote {rows} rows to {out_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--duration", type=float, default=20.0, help="seconds to log (default 20)")
    ap.add_argument("--rate", type=float, default=1.0, help="rows per second (default 1)")
    ap.add_argument("--out", type=pathlib.Path,
                    default=pathlib.Path("logs") / f"telemetry_{dt.datetime.now():%Y%m%d_%H%M%S}.csv")
    args = ap.parse_args()
    asyncio.run(main(args.duration, args.rate, args.out))
