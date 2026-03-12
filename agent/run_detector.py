#!/usr/bin/env python3
"""
Live detector monitor — subscribes to the camera, runs the two-stage
YOLOE+CLIP detector, and prints every detection to stdout.

Also saves the latest camera frame to agent/live_frame.jpg on SIGUSR1
(or just continuously, every 5 s if --save-frames is passed).

Usage:
    python agent/run_detector.py fire_extinguisher first_aid_kit hazard_sign
"""
from __future__ import annotations

import signal
import sys
import time
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector import Detector

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FRAME_PATH = os.path.join(SCRIPT_DIR, "live_frame.jpg")


class MonitorNode(Node):
    def __init__(self, targets: list[str]):
        super().__init__("detector_monitor")

        self._bridge = CvBridge()
        self._latest_frame: np.ndarray | None = None
        self._det_count = 0

        # Subscribe to raw image for frame saving
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, "/derpbot_0/rgbd/image", self._frame_cb, qos)

        # Start detector
        self._detector = Detector(self, targets)
        self.get_logger().info(f"Warming up detector for targets: {targets}")
        self._detector.start()
        self.get_logger().info("Detector ready — watching for detections...")

        # Poll detections every 0.1 s
        self.create_timer(0.1, self._poll_detections)

        # Save frame on SIGUSR1
        signal.signal(signal.SIGUSR1, self._on_save_signal)

    def _frame_cb(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _poll_detections(self) -> None:
        import queue
        while True:
            try:
                det = self._detector.detections.get_nowait()
            except queue.Empty:
                break
            self._det_count += 1
            ts = time.strftime("%H:%M:%S")
            print(
                f"[{ts}] #{self._det_count:4d}  {det.class_name:25s}  "
                f"sim={det.confidence:.3f}  "
                f"px=({det.cx_px:.0f},{det.cy_px:.0f})  "
                f"size=({det.w_px:.0f}×{det.h_px:.0f})",
                flush=True,
            )

    def save_frame(self) -> None:
        if self._latest_frame is not None:
            cv2.imwrite(FRAME_PATH, self._latest_frame)
            print(f"[frame saved → {FRAME_PATH}]", flush=True)
        else:
            print("[no frame yet]", flush=True)

    def _on_save_signal(self, signum, frame) -> None:
        self.save_frame()


def main():
    targets = sys.argv[1:] if len(sys.argv) > 1 else [
        "fire_extinguisher", "first_aid_kit", "hazard_sign"
    ]

    rclpy.init()
    node = MonitorNode(targets)

    print(f"PID {os.getpid()} — send SIGUSR1 to save current frame to {FRAME_PATH}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._detector.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
