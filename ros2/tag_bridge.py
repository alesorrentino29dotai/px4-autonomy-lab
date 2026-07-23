#!/usr/bin/env python3
"""M7 — Bridge Isaac ROS AprilTag detections to the flight controller via UDP.

WHAT IT DOES
    Runs in the isaac container next to the AprilTag node. Subscribes to
    /tag_detections (isaac_ros_apriltag_interfaces/AprilTagDetectionArray)
    and forwards, per message, a JSON datagram to 127.0.0.1:18700:

        {"t": stamp, "detected": bool,
         "x": right_m, "y": down_m, "z": forward_m}   # camera optical frame

    Unlike M5 (pixel offsets + altitude → metric), the GPU detector gives the
    full metric 3D pose of the tag directly (PnP from the known tag size),
    so the flight side gets meters with no extra math.

    Camera optical frame convention: x right, y down (in the image),
    z forward along the optical axis — for the downward camera z points at
    the ground, so z ≈ height above the tag.

MAVLINK MESSAGES INVOLVED
    None — perception side. The flight side (scripts/m7_apriltag_land.py)
    turns these into Offboard velocity setpoints.

USAGE (inside the isaac container)
    source /opt/ros/jazzy/setup.bash && python3 /lab/ros2/tag_bridge.py
"""

import json
import socket

import rclpy
from rclpy.node import Node

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

OUT_ADDR = ("127.0.0.1", 18700)


class TagBridge(Node):
    def __init__(self) -> None:
        super().__init__("tag_bridge")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sub = self.create_subscription(
            AprilTagDetectionArray, "/tag_detections", self.on_detections, 10)
        self.count = 0

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        out = {"t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               "detected": False, "x": 0.0, "y": 0.0, "z": 0.0}
        if msg.detections:
            p = msg.detections[0].pose.pose.pose.position
            out.update(detected=True, x=p.x, y=p.y, z=p.z)
        self.sock.sendto(json.dumps(out).encode(), OUT_ADDR)
        self.count += 1
        if self.count % 100 == 1:
            self.get_logger().info(
                f"forwarded {self.count} msgs, last detected={out['detected']}")


def main() -> None:
    rclpy.init()
    rclpy.spin(TagBridge())


if __name__ == "__main__":
    main()
