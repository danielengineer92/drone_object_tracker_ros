"""ROS 2 camera node. Publishes sensor_msgs/Image on 'image_raw'."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):
    MAX_CONSECUTIVE_FAILURES = 30

    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("frame_id", "camera_optical_frame")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_width  = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.fps          = int(self.get_parameter("fps").value)
        self.frame_id     = str(self.get_parameter("frame_id").value)

        if self.fps <= 0:
            raise ValueError(f"fps must be > 0, got {self.fps}")

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.publisher = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self._consecutive_failures = 0

        self.get_logger().info(
            f"Camera node started | index={self.camera_index}, "
            f"requested={self.frame_width}x{self.frame_height}@{self.fps}, "
            f"actual={actual_w}x{actual_h}@{actual_fps:.1f}"
        )

    def publish_frame(self) -> None:
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        stamp = self.get_clock().now().to_msg()

        if not ret:
            self._consecutive_failures += 1
            self.get_logger().warning(
                f"Failed to read frame ({self._consecutive_failures})"
            )
            if self._consecutive_failures >= self.MAX_CONSECUTIVE_FAILURES:
                raise RuntimeError("Camera read failures exceeded threshold")
            return

        self._consecutive_failures = 0
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def destroy_node(self) -> None:
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        self.get_logger().info("Camera node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger("camera_node").fatal(f"Fatal: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()