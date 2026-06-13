"""
ROS2 camera node.

Publishes:
    /camera/image_raw
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 30)

        self.camera_index = self.get_parameter("camera_index").value
        self.frame_width = self.get_parameter("frame_width").value
        self.frame_height = self.get_parameter("frame_height").value
        self.fps = self.get_parameter("fps").value

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera index {self.camera_index}")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(Image, "/camera/image_raw", qos)

        timer_period = 1.0 / float(self.fps)
        self.timer = self.create_timer(timer_period, self.publish_frame)

        self.get_logger().info(
            f"Camera node started | index={self.camera_index}, "
            f"resolution={self.frame_width}x{self.frame_height}, fps={self.fps}"
        )

    def publish_frame(self) -> None:
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Failed to read frame from camera")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        self.publisher.publish(msg)

    def destroy_node(self) -> None:
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()

        self.get_logger().info("Camera node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
