"""PC-side viewer: shows the bisa debug overlay image in an OpenCV window.

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

WINDOW = "D-Racer View"


class VizNode(Node):
    """Subscribes to the annotated debug image and displays it with OpenCV."""

    def __init__(self):
        """Sets up the debug-image subscription and the display window."""

        super().__init__("bisa_viz_node")
        self.declare_parameter("debug_image_topic", "/bisa/debug/image/compressed")
        topic = str(self.get_parameter("debug_image_topic").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(CompressedImage, topic, self.on_image, qos)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"Viz node showing '{topic}' (focus window, press q to quit)")

    def on_image(self, msg: CompressedImage) -> None:
        """Decodes and shows one debug frame; quits the process on 'q'."""

        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            return
        cv2.imshow(WINDOW, frame)
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
