"""PC-side viewer: shows the bisa debug streams in two OpenCV windows.

Window 1 ("Detect View") is the full color-corrected camera frame with object
detections, traffic-light ROI, and ArUco overlays — color-correction sliders
(CLAHE/saturation/brightness/...) change this picture live. Window 2
("Lane ROI Mask") is the ROI-sized binarized lane mask with the lane lines,
band centers, and steering arrow, for HSV/LAB threshold and ROI-size tuning.

Run this only on the operator PC. It never runs on the vehicle, so the car
spends no cycles on visualization.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

WINDOW_DETECT = "Detect View"
WINDOW_LANE_MASK = "Lane ROI Mask"


class VizNode(Node):
    """Subscribes to the annotated debug images and displays them with OpenCV."""

    def __init__(self):
        """Sets up both debug-image subscriptions and their display windows."""

        super().__init__("bisa_viz_node")
        self.declare_parameter("debug_image_topic", "/bisa/debug/image/compressed")
        self.declare_parameter("lane_mask_topic", "/bisa/debug/lane_mask/compressed")
        detect_topic = str(self.get_parameter("debug_image_topic").value)
        mask_topic = str(self.get_parameter("lane_mask_topic").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            CompressedImage, detect_topic,
            lambda msg: self.on_image(msg, WINDOW_DETECT), qos,
        )
        self.create_subscription(
            CompressedImage, mask_topic,
            lambda msg: self.on_image(msg, WINDOW_LANE_MASK), qos,
        )
        cv2.namedWindow(WINDOW_DETECT, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_LANE_MASK, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f"Viz node showing '{detect_topic}' + '{mask_topic}' "
            "(focus a window, press q to quit)"
        )

    def on_image(self, msg: CompressedImage, window: str) -> None:
        """Decodes and shows one debug frame; quits the process on 'q'."""

        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            return
        cv2.imshow(window, frame)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            rclpy.shutdown()


def main(args=None) -> None:
    """Initializes rclpy and spins the viz node."""

    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
