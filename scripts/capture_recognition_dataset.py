#!/usr/bin/env python3
"""Capture timestamped camera frames for D-Racer recognition validation."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class DatasetCapture(Node):
    """Save unique compressed-camera frames at a bounded rate."""

    def __init__(self, topic: str, output: Path, capture_hz: float, max_frames: int):
        super().__init__("recognition_dataset_capture")
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.manifest = self.output / "manifest.jsonl"
        self.period = 1.0 / max(float(capture_hz), 0.1)
        self.max_frames = max(0, int(max_frames))
        self.last_save = 0.0
        self.count = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(CompressedImage, topic, self.on_image, qos)
        self.get_logger().info(
            f"Capturing {topic} at <= {capture_hz:g} Hz into {self.output}"
        )

    def on_image(self, msg: CompressedImage) -> None:
        """Decode and save a frame when the collection interval has elapsed."""

        now = time.monotonic()
        if now - self.last_save < self.period:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Dropped undecodable camera frame")
            return
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        filename = f"frame_{self.count:06d}_{stamp_ns}.jpg"
        path = self.output / filename
        if not cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
            self.get_logger().warning(f"Failed to write {path}")
            return
        record = {
            "file": filename,
            "stamp_ns": stamp_ns,
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
        }
        with self.manifest.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.last_save = now
        self.count += 1
        if self.max_frames and self.count >= self.max_frames:
            self.get_logger().info(f"Captured {self.count} frames; stopping")
            rclpy.shutdown()


def main() -> None:
    """Parse collection settings and spin the capture node."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image/compressed")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--capture-hz", type=float, default=2.0)
    parser.add_argument("--max-frames", type=int, default=0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = DatasetCapture(args.topic, args.output, args.capture_hz, args.max_frames)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
