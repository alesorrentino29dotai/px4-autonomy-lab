#!/usr/bin/env python3
"""Demo-video helper: record the drone's onboard camera with AprilTag overlay.

Runs INSIDE the px4-sitl container (gz-transport + OpenCV). Subscribes to the
downward mono camera, draws the detected AprilTag (green outline, cv2.aruco
DICT_APRILTAG_36h11) and the image-center crosshair (red), and saves every
Nth frame as JPEG. Pure recorder: publishes nothing, does not interfere with
the Isaac ROS perception pipeline.

    docker exec -d px4-sitl python3 /lab/sim/onboard_recorder.py \
        --world baylands --save-dir /lab/.onboard_frames --save-every 2
"""

import argparse
import pathlib
import time

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="baylands")
    ap.add_argument("--model", default="x500_mono_cam_down_0")
    ap.add_argument("--save-dir", type=pathlib.Path, default=pathlib.Path("/lab/.onboard_frames"))
    ap.add_argument("--save-every", type=int, default=2)
    args = ap.parse_args()

    topic = (f"/world/{args.world}/model/{args.model}"
             "/link/camera_link/sensor/camera/image")
    args.save_dir.mkdir(parents=True, exist_ok=True)

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()
    state = {"n": 0, "saved": 0}

    def on_image(msg: Image) -> None:
        state["n"] += 1
        if state["n"] % args.save_every:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
        h, w = bgr.shape[:2]
        cv2.drawMarker(bgr, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 50, 2)
        small = cv2.resize(bgr, (w // 2, h // 2))
        cv2.imwrite(str(args.save_dir / f"o{state['saved']:05d}.jpg"), small,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])
        state["saved"] += 1

    node = Node()
    if not node.subscribe(Image, topic, on_image):
        raise SystemExit(f"cannot subscribe {topic}")
    print(f"[onboard-rec] {topic} -> {args.save_dir}", flush=True)
    while True:
        time.sleep(2)


if __name__ == "__main__":
    main()
