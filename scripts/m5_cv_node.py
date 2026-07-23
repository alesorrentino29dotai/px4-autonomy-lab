#!/usr/bin/env python3
"""M5 (CV side) — ArUco detector on the downward camera, offsets out via UDP.

WHAT IT DOES
    Runs INSIDE the SITL container (where the Gazebo transport lives):

      1. subscribes to the downward camera of x500_mono_cam_down through
         gz-transport (topic /world/<world>/model/.../sensor/camera/image)
      2. converts each frame to a numpy array and runs ArUco detection
         (OpenCV 4.6 legacy API). The marker dictionary is auto-discovered:
         the first frames are scanned against common dictionaries
         (DICT_4X4_50 ... DICT_ARUCO_ORIGINAL), then the winner is locked in.
      3. computes the marker-center offset from the image center and converts
         it to ANGLES using the camera intrinsics (camera_info: fx≈539.94,
         cx=640, cy=480 @ 1280x960):  ang_x = atan((u-cx)/fx)
         Angles (not pixels) are what the flight side wants: combined with
         the current altitude they give a metric offset, independent of
         image resolution.
      4. streams a JSON datagram per frame to the flight node:
         {"t": stamp, "detected": bool, "ang_x": rad, "ang_y": rad,
          "px": marker side length in pixels}
         ang_x > 0: marker to the RIGHT of the image; ang_y > 0: marker DOWN
         in the image. UDP 127.0.0.1:18700 (container runs with host network).

    This mirrors a real companion-computer split: perception near the sensor,
    guidance elsewhere, a thin telemetry contract between them.

MAVLINK MESSAGES INVOLVED
    None directly — this node is pure perception. (A production alternative
    is sending LANDING_TARGET to PX4's built-in precision-landing module;
    here the flight node closes the loop itself in Offboard, see
    m5_precision_land.py.)

USAGE (inside the container)
    docker exec -d px4-sitl python3 /lab/scripts/m5_cv_node.py
    # demo recording: annotated frames (marker outline + crosshair) for the GIF
    docker exec -d px4-sitl python3 /lab/scripts/m5_cv_node.py --save-dir /lab/docs/media/frames --save-every 5
"""

import argparse
import json
import pathlib
import socket
import time

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

WORLD = "aruco"
MODEL = "x500_mono_cam_down_0"
TOPIC = f"/world/{WORLD}/model/{MODEL}/link/camera_link/sensor/camera/image"

FX = 539.936
CX, CY = 640.0, 480.0

OUT_ADDR = ("127.0.0.1", 18700)

CANDIDATE_DICTS = [
    ("DICT_4X4_50", cv2.aruco.DICT_4X4_50),
    ("DICT_5X5_50", cv2.aruco.DICT_5X5_50),
    ("DICT_6X6_50", cv2.aruco.DICT_6X6_50),
    ("DICT_ARUCO_ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
]


class Detector:
    def __init__(self, save_dir: pathlib.Path | None = None, save_every: int = 5) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.params = cv2.aruco.DetectorParameters_create()
        self.locked = None          # (name, dictionary) once discovered
        self.frames = 0
        self.hits = 0
        self.last_log = time.time()
        self.save_dir = save_dir
        self.save_every = save_every
        if save_dir:
            save_dir.mkdir(parents=True, exist_ok=True)

    def detect(self, gray: np.ndarray):
        if self.locked is not None:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.locked[1], parameters=self.params)
            return corners, ids
        for name, enum in CANDIDATE_DICTS:
            d = cv2.aruco.Dictionary_get(enum)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=self.params)
            if ids is not None and len(ids) > 0:
                self.locked = (name, d)
                print(f"[cv] dictionary locked: {name} (id {ids.flatten()[0]})", flush=True)
                return corners, ids
        return [], None

    def on_image(self, msg: Image) -> None:
        self.frames += 1
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        corners, ids = self.detect(gray)

        out = {"t": time.time(), "detected": False, "ang_x": 0.0, "ang_y": 0.0, "px": 0.0}
        if ids is not None and len(ids) > 0:
            self.hits += 1
            c = corners[0].reshape(4, 2)          # marker corner pixels
            u, v = c.mean(axis=0)                 # marker center
            out.update(
                detected=True,
                ang_x=float(np.arctan2(u - CX, FX)),
                ang_y=float(np.arctan2(v - CY, FX)),
                px=float(np.linalg.norm(c[0] - c[1])),
            )
        self.sock.sendto(json.dumps(out).encode(), OUT_ADDR)

        if self.save_dir and self.frames % self.save_every == 0:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
            h, w = bgr.shape[:2]
            cv2.drawMarker(bgr, (w // 2, h // 2), (0, 0, 255),
                           cv2.MARKER_CROSS, 40, 2)
            small = cv2.resize(bgr, (w // 2, h // 2))
            cv2.imwrite(str(self.save_dir / f"f{self.frames:06d}.jpg"), small)

        if time.time() - self.last_log > 5:
            rate = self.hits / max(self.frames, 1) * 100
            print(f"[cv] frames={self.frames} detect-rate={rate:.0f}%", flush=True)
            self.last_log = time.time()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--save-dir", type=pathlib.Path, default=None,
                    help="save annotated frames here (for demo GIFs)")
    ap.add_argument("--save-every", type=int, default=5,
                    help="save one frame out of N (default 5)")
    args = ap.parse_args()

    det = Detector(save_dir=args.save_dir, save_every=args.save_every)
    node = Node()
    if not node.subscribe(Image, TOPIC, det.on_image):
        raise SystemExit(f"cannot subscribe to {TOPIC}")
    print(f"[cv] subscribed to {TOPIC}", flush=True)
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
