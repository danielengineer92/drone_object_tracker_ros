"""
Fake camera node that generates synthetic test images.

This node produces test patterns with a moving circle target that simulates
a real camera feed. It publishes on the same topic as the real camera node
so the rest of the pipeline works identically.
"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class FakeCameraNode(Node):
    """ROS 2 node that generates synthetic camera images for testing."""

    def __init__(self) -> None:
        """Initialize the fake camera node."""
        super().__init__('fake_camera_node')

        # Declare parameters
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('target_type', 'person_like')
        self.declare_parameter('motion_pattern', 'sinusoidal')
        self.declare_parameter('motion_speed', 1.0)
        self.declare_parameter('add_noise', True)
        self.declare_parameter('noise_level', 10)

        # Read parameters
        self._frame_width: int = self.get_parameter('frame_width').value
        self._frame_height: int = self.get_parameter('frame_height').value
        self._fps: float = self.get_parameter('fps').value
        self._target_type: str = self.get_parameter('target_type').value
        self._motion_pattern: str = self.get_parameter('motion_pattern').value
        self._motion_speed: float = self.get_parameter('motion_speed').value
        self._add_noise: bool = self.get_parameter('add_noise').value
        self._noise_level: int = self.get_parameter('noise_level').value

        # State
        self._bridge: CvBridge = CvBridge()
        self._start_time: float = time.time()
        self._frame_count: int = 0

        # QoS for image publishing
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher
        self._image_pub = self.create_publisher(Image, '/camera/image_raw', image_qos)

        # Timer for frame generation
        timer_period: float = 1.0 / self._fps
        self._capture_timer = self.create_timer(timer_period, self._generate_frame)

        # FPS reporting
        self._fps_timer = self.create_timer(5.0, self._report_fps)
        self._last_fps_time: float = time.time()
        self._fps_count: int = 0

        self.get_logger().info(
            f'Fake camera node initialized: {self._frame_width}x{self._frame_height} @ '
            f'{self._fps}fps, pattern={self._motion_pattern}'
        )

    def _get_target_position(self, t: float) -> tuple[int, int]:
        """
        Calculate the target position based on time and motion pattern.

        Args:
            t: Current time in seconds since start.

        Returns:
            Tuple of (x, y) pixel coordinates.
        """
        speed = self._motion_speed
        cx = self._frame_width // 2
        cy = self._frame_height // 2

        if self._motion_pattern == 'sinusoidal':
            # Figure-8 pattern
            x = cx + int(math.sin(t * speed) * self._frame_width * 0.3)
            y = cy + int(math.sin(t * speed * 2.0) * self._frame_height * 0.2)
        elif self._motion_pattern == 'circular':
            x = cx + int(math.cos(t * speed) * self._frame_width * 0.25)
            y = cy + int(math.sin(t * speed) * self._frame_height * 0.25)
        elif self._motion_pattern == 'linear':
            # Bounce horizontally
            period = self._frame_width * 2.0 / (speed * 100.0)
            progress = (t % period) / period
            if progress < 0.5:
                x = int(self._frame_width * 0.1 + progress * 2.0 * self._frame_width * 0.8)
            else:
                x = int(self._frame_width * 0.9 - (progress - 0.5) * 2.0 * self._frame_width * 0.8)
            y = cy
        elif self._motion_pattern == 'stationary':
            x = cx
            y = cy
        else:
            x = cx + int(math.sin(t * speed) * self._frame_width * 0.3)
            y = cy + int(math.cos(t * speed * 0.7) * self._frame_height * 0.2)

        # Clamp to frame bounds
        x = max(50, min(self._frame_width - 50, x))
        y = max(50, min(self._frame_height - 50, y))

        return x, y

    def _draw_person_like_target(self, frame: np.ndarray, x: int, y: int) -> np.ndarray:
        """
        Draw a person-like silhouette target.

        Args:
            frame: The frame to draw on.
            x: Center x coordinate.
            y: Center y coordinate.

        Returns:
            Frame with the target drawn.
        """
        # Head
        head_radius = 15
        cv2.circle(frame, (x, y - 50), head_radius, (100, 150, 200), -1)
        cv2.circle(frame, (x, y - 50), head_radius, (50, 100, 150), 2)

        # Body
        cv2.rectangle(frame, (x - 20, y - 35), (x + 20, y + 20), (80, 80, 180), -1)
        cv2.rectangle(frame, (x - 20, y - 35), (x + 20, y + 20), (40, 40, 140), 2)

        # Arms
        cv2.rectangle(frame, (x - 35, y - 30), (x - 20, y + 5), (80, 80, 180), -1)
        cv2.rectangle(frame, (x + 20, y - 30), (x + 35, y + 5), (80, 80, 180), -1)

        # Legs
        cv2.rectangle(frame, (x - 15, y + 20), (x - 3, y + 60), (60, 60, 120), -1)
        cv2.rectangle(frame, (x + 3, y + 20), (x + 15, y + 60), (60, 60, 120), -1)

        return frame

    def _draw_simple_target(self, frame: np.ndarray, x: int, y: int) -> np.ndarray:
        """
        Draw a simple colored rectangle target.

        Args:
            frame: The frame to draw on.
            x: Center x coordinate.
            y: Center y coordinate.

        Returns:
            Frame with the target drawn.
        """
        w, h = 60, 80
        cv2.rectangle(frame, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2),
                      (0, 200, 100), -1)
        cv2.rectangle(frame, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2),
                      (0, 150, 80), 3)
        cv2.putText(frame, 'TARGET', (x - 30, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return frame

    def _generate_frame(self) -> None:
        """Generate and publish a synthetic frame."""
        t = time.time() - self._start_time

        # Create base frame with gradient background
        frame = np.zeros((self._frame_height, self._frame_width, 3), dtype=np.uint8)

        # Sky gradient
        for row in range(self._frame_height):
            ratio = row / self._frame_height
            blue = int(200 - ratio * 100)
            green = int(180 - ratio * 80)
            red = int(150 - ratio * 60)
            frame[row, :] = [max(0, blue), max(0, green), max(0, red)]

        # Ground area (lower third)
        ground_start = int(self._frame_height * 0.7)
        frame[ground_start:, :] = [40, 100, 50]

        # Add some static "features" for visual interest
        cv2.circle(frame, (100, 100), 30, (200, 200, 50), -1)  # "Sun"
        cv2.rectangle(frame, (400, ground_start - 40), (450, ground_start),
                      (60, 60, 100), -1)  # "Building"
        cv2.rectangle(frame, (200, ground_start - 25), (230, ground_start),
                      (80, 80, 120), -1)  # "Building"

        # Get target position
        target_x, target_y = self._get_target_position(t)

        # Draw target
        if self._target_type == 'person_like':
            frame = self._draw_person_like_target(frame, target_x, target_y)
        else:
            frame = self._draw_simple_target(frame, target_x, target_y)

        # Add noise
        if self._add_noise:
            noise = np.random.randint(
                -self._noise_level, self._noise_level,
                frame.shape, dtype=np.int16
            )
            frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Add timestamp text
        cv2.putText(frame, f'FAKE CAM | t={t:.1f}s',
                    (10, self._frame_height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)

        # Publish
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_optical_frame'

        img_msg: Image = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header = header

        self._image_pub.publish(img_msg)
        self._frame_count += 1
        self._fps_count += 1

    def _report_fps(self) -> None:
        """Report FPS."""
        current_time = time.time()
        elapsed = current_time - self._last_fps_time
        if elapsed > 0:
            fps = self._fps_count / elapsed
            self.get_logger().info(
                f'Fake camera: {fps:.1f} FPS, frames={self._frame_count}'
            )
        self._fps_count = 0
        self._last_fps_time = current_time

    def destroy_node(self) -> None:
        """Clean up."""
        self.get_logger().info('Shutting down fake camera node...')
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the fake camera node."""
    rclpy.init(args=args)
    node = FakeCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()