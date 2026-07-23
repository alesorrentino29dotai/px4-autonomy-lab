# M5 — Vision-Based Precision Landing on an ArUco Marker

![Precision landing demo](media/precision_landing.gif)

*Downward-camera view during the autonomous descent: ArUco detection (green),
image center (red cross). Touchdown error: **2–3 cm** from the marker center.*

## The problem

"Drone-in-a-box" operations need the vehicle to land on a charging pad with
centimeter accuracy — far beyond GPS (meters). The standard solution is a
fiducial marker on the pad and a downward camera closing the loop visually
during the final descent.

## Architecture

```
┌────────────────────── Docker container ──────────────────────┐
│  Gazebo (world: aruco)          PX4 SITL                     │
│  ┌──────────────────┐           ┌─────────┐                  │
│  │ x500_mono_cam_down│ sensors → │  PX4    │ ←── MAVLink ────┼──► host: QGC (14550)
│  │  camera 1280×960  │           └─────────┘                 │
│  └───────┬──────────┘                ▲                       │
│          │ gz-transport              │ MAVLink udp 14540     │
│  ┌───────▼──────────┐               │                        │
│  │  m5_cv_node.py    │               │                        │
│  │  OpenCV ArUco     │               │                        │
│  └───────┬──────────┘               │                        │
└──────────┼──────────────────────────┼────────────────────────┘
           │ UDP 18700 (JSON offsets) │
      ┌────▼──────────────────────────┴───┐
      │  m5_precision_land.py (host)      │
      │  MAVSDK Offboard velocity control │
      └───────────────────────────────────┘
```

Two nodes, mirroring a real companion-computer split:

- **`m5_cv_node.py`** (perception, runs *inside* the container where
  gz-transport lives): subscribes to the camera, detects the marker
  (dictionary auto-discovered → `DICT_4X4_50`, id 0), converts the pixel
  offset to **angles** via the camera intrinsics (fx ≈ 539.94), and streams
  `{detected, ang_x, ang_y, px}` as JSON over UDP at frame rate.
- **`m5_precision_land.py`** (guidance, host): MAVSDK Offboard loop @ 10 Hz.

Angles — not pixels — cross the interface: combined with the current altitude
they give a metric offset regardless of image resolution. That's the same
contract as MAVLink's `LANDING_TARGET` message.

## Control law

With the camera pitched 90° down, image axes map to body axes as
*u right → east, v down → −north* (yaw held at 0):

```
north = −tan(ang_y) · alt          east = tan(ang_x) · alt
vn    = clamp(Kp · north)          ve   = clamp(Kp · east)      Kp = 0.6 s⁻¹
vd    = 0.7 m/s (alt > 3 m) or 0.3 m/s   — but ONLY while centered:
        radial offset < max(0.4 m, 8 % of alt)
```

Descend-only-when-centered makes the funnel self-correcting: any drift pauses
the descent while the lateral controller re-centers. Below 1 m the marker
overflows the field of view, so the endgame hands over to `action.land()`.

Marker lost > 1 s → hold position (don't descend blind); regained → resume.

## Results (SITL, 2 runs)

| Metric | Value |
|---|---|
| Start offset from pad | 5 m lateral @ 8 m altitude |
| Touchdown error | **0.02–0.03 m** from marker center |
| Descent time | ~45 s (taper near ground) |

Landing error is measured from PX4's `LOCAL_POSITION_NED` at disarm: the
marker sits at the world origin, which is also the arming point.

## Reproduce

```bash
MODEL=gz_x500_mono_cam_down WORLD=aruco ./scripts/run_sitl.sh   # terminal 1
docker exec -d px4-sitl python3 /lab/scripts/m5_cv_node.py       # terminal 2
.venv/bin/python scripts/m5_precision_land.py                    # terminal 3
```

(The container must be started with the repo mounted at `/lab` and
`--network host`; `run_sitl.sh` does both.)

To record the annotated camera frames (as in the GIF above):

```bash
docker exec -d px4-sitl python3 /lab/scripts/m5_cv_node.py \
    --save-dir /lab/docs/media/frames --save-every 4
ffmpeg -framerate 12 -pattern_type glob -i 'docs/media/frames/*.jpg' \
    -vf "scale=480:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
    docs/media/precision_landing.gif
```

## What I'd do differently on real hardware

- Feed PX4's **built-in precision-landing module** with `LANDING_TARGET`
  MAVLink messages instead of closing the loop externally — it handles search
  patterns and failsafes natively (params `PLD_*`, `RTL_PLD_MD`).
- A **nested marker** (large + small) so the target stays resolvable both at
  altitude and in the last meter.
- Compensate camera latency and vehicle attitude (the offset is measured in
  the image frame of a tilting vehicle; at these speeds in SITL the error is
  negligible, on a windy day it is not).
- EKF fusion of the vision offset (e.g. as an external vision aid) rather
  than a raw P controller.
