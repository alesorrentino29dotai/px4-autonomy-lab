# Environment Setup

Host: **Ubuntu 26.04** (not officially supported by PX4 — only 22.04/24.04 are, as of
July 2026), so PX4 SITL runs in the official **pre-built Docker container**. No PX4
source build is needed.

## Installed versions

| Component | Version | Where |
|---|---|---|
| Docker Engine | 29.1.3 | host (apt) |
| PX4 SITL + Gazebo Harmonic | `px4io/px4-sitl-gazebo:v1.18.0-beta1` | Docker image |
| QGroundControl | v5.0.8 AppImage | `~/Applications/` |
| Python (venv) | 3.12.13 (uv-managed standalone) | `.venv/` |
| mavsdk / pymavlink | 3.17.2 / 2.4.49 | `.venv/` |
| OpenCV / numpy | 5.0.0 / 2.5.1 | `.venv/` |
| gcc/g++ | 15.2.0 | host |
| cmake / ninja | 4.4.0 / 1.13.0 (pip wheels) | `.venv/bin/` |

### Notable deviations from a stock setup

- **Image tag**: no stable `vX.Y.Z` tag of `px4io/px4-sitl-gazebo` exists yet on Docker
  Hub; `v1.18.0-beta1` is the most recent tagged release and is pinned in
  `scripts/run_sitl.sh` (override with `TAG=...`).
- **`--network host` instead of `-p` port mapping**: PX4 *initiates* the MAVLink UDP
  stream toward localhost. Inside a bridged container that traffic dies on the
  container's own loopback, and docker-proxy additionally occupies host port 14550.
  Host networking (Linux-only) makes container-localhost = host-localhost and
  everything works: 14550/udp → QGroundControl, 14540/udp → MAVSDK.
- **Python 3.12 via `uv`, not the system 3.14**: the host lacks `python3.14-venv` /
  `python3.14-dev` (no sudo available in the automated setup), so `pymavlink`'s C
  extension could not build. `uv venv --python 3.12` downloads a self-contained
  CPython with headers, and every package installs as a prebuilt wheel.
- **cmake/ninja as pip wheels** in the venv (host has none; needed later for MAVSDK C++).

## Start the simulator

```bash
./scripts/run_sitl.sh                          # gz_x500, GUI if $DISPLAY is set
HEADLESS=1 ./scripts/run_sitl.sh               # headless
MODEL=gz_x500_mono_cam_down ./scripts/run_sitl.sh   # downward-camera variant (M5)
```

## Verification commands

| Check | Command | Expected |
|---|---|---|
| Docker | `docker run --rm hello-world` | "Hello from Docker!" |
| SITL boots | `docker logs px4-sitl` | `Startup script returned successfully`, `pxh>` |
| MAVLink GCS link | listen on `udpin:0.0.0.0:14550` (pymavlink) | HEARTBEAT within seconds |
| Takeoff | `commander takeoff` in the `pxh>` console | altitude climbs (needs a GCS heartbeat on 14550 first — see NOTES.md) |
| MAVSDK link | `.venv/bin/python scripts/check_connection.py` | `✓ Connected`, health checks `True` |
| QGroundControl | launch AppImage with SITL running | vehicle appears, telemetry live |

## Python environment

```bash
uv venv --python 3.12 .venv        # or: python3 -m venv .venv on a supported host
uv pip install --python .venv -r requirements.txt
```
