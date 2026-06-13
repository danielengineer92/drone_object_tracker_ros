"""
Camera capture node for the drone vision system.

This node opens a camera device, captures frames at a configurable rate,
and publishes them as ROS 2 Image messages. It supports automatic reconnection
if the camera disconnects.
"""

import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class CameraNode(Node):
    """ROS 2 node that captures frames from a camera and publishes them."""

    def __init__(self) -> None:
        """Initialize the camera node with parameters and publishers."""
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('reconnect_interval', 3.0)
        self.declare_parameter('max_reconnect_attempts', 10)
        self.declare_parameter('camera_backend', 'v4l2')

        # Read parameters
        self._camera_index: int = self.get_parameter('camera_index').value
        self._frame_width: int = self.get_parameter('frame_width').value
        self._frame_height: int = self.get_parameter('frame_height').value
        self._fps: float = self.get_parameter('fps').value
        self._reconnect_interval: float = self.get_parameter('reconnect_interval').value
        self._max_reconnect_attempts: int = self.get_parameter('max_reconnect_attempts').value
        self._camera_backend: str = self.get_parameter('camera_backend').value

        # State
        self._capture: Optional[cv2.VideoCapture] = None
        self._bridge: CvBridge = CvBridge()
        self._connected: bool = False
        self._reconnect_attempts: int = 0
        self._frame_count: int = 0
        self._last_fps_time: float = time.time()
        self._measured_fps: float = 0.0

        # QoS for image publishing - best effort for real-time performance
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self._image_pub = self.create_publisher(Image, '/camera/image_raw', image_qos)

        # Attempt initial connection
        self._connect_camera()

        # Timer for frame capture
        timer_period: float = 1.0 / self._fps
        self._capture_timer = self.create_timer(timer_period, self._capture_callback)

        # Timer for FPS reporting
        self._fps_timer = self.create_timer(5.0, self._report_fps)

        self.get_logger().info(
            f'Camera node initialized: index={self._camera_index}, '
            f'resolution={self._frame_width}x{self._frame_height}, '
            f'fps={self._fps}'
        )

    def _get_backend(self) -> int:
        """Get the OpenCV video capture backend identifier."""
        backends = {
            'v4l2': cv2.CAP_V4L2,
            'gstreamer': cv2.CAP_GSTREAMER,
            'any': cv2.CAP_ANY,
            'ffmpeg': cv2.CAP_FFMPEG,
        }
        return backends.get(self._camera_backend, cv2.CAP_ANY)

    def _connect_camera(self) -> bool:
        """
        Attempt to open the camera device.

        Returns:
            True if the connection was successful, False otherwise.
        """
        if self._capture is not None:
            self._capture.release()
            self._capture = None

        try:
            backend = self._get_backend()
            self._capture = cv2.VideoCapture(self._camera_index, backend)

            if not self._capture.isOpened():
                self.get_logger().warning(
                    f'Failed to open camera {self._camera_index} with backend {self._camera_backend}'
                )
                self._connected = False
                return False

            # Configure camera properties
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)
            self._capture.set(cv2.CAP_PROP_FPS, self._fps)
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Verify settings
            actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self._capture.get(cv2.CAP_PROP_FPS)

            self._connected = True
            self._reconnect_attempts = 0

            self.get_logger().info(
                f'Camera connected: actual resolution={actual_width}x{actual_height}, '
                f'actual fps={actual_fps:.1f}'
            )
            return True

        except Exception as e:
            self.get_logger().error(f'Exception while connecting to camera: {e}')
            self._connected = False
            return False

    def _attempt_reconnect(self) -> None:
        """Attempt to reconnect to the camera with backoff."""
        if self._reconnect_attempts >= self._max_reconnect_attempts:
            self.get_logger().error(
                f'Max reconnect attempts ({self._max_reconnect_attempts}) reached. '
                f'Camera node will keep trying at reduced rate.'
            )
            # Reset counter but log reduced attempts
            self._reconnect_attempts = 0

        self._reconnect_attempts += 1
        self.get_logger().info(
            f'Attempting camera reconnection ({self._reconnect_attempts}/'
            f'{self._max_reconnect_attempts})...'
        )

        if self._connect_camera():
            self.get_logger().info('Camera reconnected successfully.')

    def _capture_callback(self) -> None:
        """Timer callback to capture and publish a frame."""
        if not self._connected or self._capture is None:
            self._attempt_reconnect()
            return

        try:
            ret, frame = self._capture.read()

            if not ret or frame is None:
                self.get_logger().warning('Failed to read frame from camera.')
                self._connected = False
                return

            # Resize if necessary
            h, w = frame.shape[:2]
            if w != self._frame_width or h != self._frame_height:
                frame = cv2.resize(frame, (self._frame_width, self._frame_height))

            # Convert to ROS Image message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_optical_frame'

            img_msg: Image = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = header

            self._image_pub.publish(img_msg)

            # FPS counting
            self._frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Error during frame capture: {e}')
            self._connected = False

    def _report_fps(self) -> None:
        """Report the measured frames per second."""
        current_time = time.time()
        elapsed = current_time - self._last_fps_time

        if elapsed > 0:
            self._measured_fps = self._frame_count / elapsed
            self.get_logger().info(
                f'Camera FPS: {self._measured_fps:.1f} | '
                f'Connected: {self._connected} | '
                f'Frames published: {self._frame_count}'
            )

        self._frame_count = 0
        self._last_fps_time = current_time

    def destroy_node(self) -> None:
        """Clean up resources on shutdown."""
        self.get_logger().info('Shutting down camera node...')
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the camera node."""
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()