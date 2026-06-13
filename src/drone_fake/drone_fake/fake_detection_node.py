"""
Fake detection node that generates synthetic detection data.

This node simulates YOLO detection output by publishing detection messages
with a configurable moving target. It uses the same message formats as the
real YOLO node so the tracker and control nodes work identically.
"""

import math
import time
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_interfaces.msg import Detection, DetectionArray


class FakeDetectionNode(Node):
    """ROS 2 node that generates fake detection data for testing."""

    def __init__(self) -> None:
        """Initialize the fake detection node."""
        super().__init__('fake_detection_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('target_class_id', 0)
        self.declare_parameter('base_confidence', 0.85)
        self.declare_parameter('confidence_noise', 0.1)
        self.declare_parameter('motion_pattern', 'sinusoidal')
        self.declare_parameter('motion_speed', 1.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('detection_dropout_rate', 0.05)
        self.declare_parameter('add_false_detections', True)
        self.declare_parameter('false_detection_rate', 0.1)

        # Read parameters
        self._publish_rate: float = self.get_parameter('publish_rate').value
        self._target_class: str = self.get_parameter('target_class').value
        self._target_class_id: int = self.get_parameter('target_class_id').value
        self._base_confidence: float = self.get_parameter('base_confidence').value
        self._confidence_noise: float = self.get_parameter('confidence_noise').value
        self._motion_pattern: str = self.get_parameter('motion_pattern').value
        self._motion_speed: float = self.get_parameter('motion_speed').value
        self._image_width: int = self.get_parameter('image_width').value
        self._image_height: int = self.get_parameter('image_height').value
        self._detection_dropout_rate: float = self.get_parameter('detection_dropout_rate').value
        self._add_false_detections: bool = self.get_parameter('add_false_detections').value
        self._false_detection_rate: float = self.get_parameter('false_detection_rate').value

        # State
        self._start_time: float = time.time()
        self._detection_count: int = 0

        # Other class names for false detections
        self._other_classes = ['car', 'dog', 'bicycle', 'bird', 'cat', 'truck']

        # QoS
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publisher
        self._detection_pub = self.create_publisher(
            DetectionArray,
            '/detections',
            detection_qos
        )

        # Timer
        publish_period = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_detections)

        # Status timer
        self._status_timer = self.create_timer(10.0, self._report_status)

        self.get_logger().info(
            f'Fake detection node initialized: class={self._target_class}, '
            f'rate={self._publish_rate}Hz, pattern={self._motion_pattern}'
        )

    def _get_target_position(self, t: float) -> tuple[float, float]:
        """
        Calculate normalized target position (0.0 to 1.0).

        Args:
            t: Time since start.

        Returns:
            Tuple of (normalized_x, normalized_y).
        """
        speed = self._motion_speed

        if self._motion_pattern == 'sinusoidal':
            x = 0.5 + 0.3 * math.sin(t * speed)
            y = 0.5 + 0.2 * math.sin(t * speed * 2.0)
        elif self._motion_pattern == 'circular':
            x = 0.5 + 0.25 * math.cos(t * speed)
            y = 0.5 + 0.25 * math.sin(t * speed)
        elif self._motion_pattern == 'drift':
            # Slow drift across the frame
            x = 0.5 + 0.3 * math.sin(t * speed * 0.3)
            y = 0.5 + 0.15 * math.cos(t * speed * 0.2)
        elif self._motion_pattern == 'stationary':
            x = 0.5
            y = 0.5
        else:
            x = 0.5 + 0.3 * math.sin(t * speed)
            y = 0.5 + 0.2 * math.cos(t * speed * 0.7)

        # Add small jitter for realism
        x += random.gauss(0, 0.005)
        y += random.gauss(0, 0.005)

        # Clamp
        x = max(0.05, min(0.95, x))
        y = max(0.05, min(0.95, y))

        return x, y

    def _publish_detections(self) -> None:
        """Generate and publish fake detections."""
        t = time.time() - self._start_time

        detection_array = DetectionArray()
        detection_array.stamp = self.get_clock().now().to_msg()
        detection_array.image_width = self._image_width
        detection_array.image_height = self._image_height

        detections = []

        # Simulate detection dropout
        if random.random() > self._detection_dropout_rate:
            # Generate primary target detection
            x, y = self._get_target_position(t)

            detection = Detection()
            detection.stamp = detection_array.stamp
            detection.class_name = self._target_class
            detection.class_id = self._target_class_id

            # Confidence with noise
            confidence = self._base_confidence + random.gauss(0, self._confidence_noise)
            detection.confidence = float(max(0.3, min(0.99, confidence)))

            # Normalized coordinates
            detection.center_x = float(x)
            detection.center_y = float(y)

            # Target size (with some variation)
            base_width = 0.08 + 0.02 * math.sin(t * 0.5)
            base_height = 0.15 + 0.03 * math.sin(t * 0.3)
            detection.width = float(base_width)
            detection.height = float(base_height)

            # Pixel values
            detection.pixel_center_x = int(x * self._image_width)
            detection.pixel_center_y = int(y * self._image_height)
            detection.pixel_width = int(base_width * self._image_width)
            detection.pixel_height = int(base_height * self._image_height)

            detections.append(detection)

        # Add false detections occasionally
        if self._add_false_detections and random.random() < self._false_detection_rate:
            false_detection = Detection()
            false_detection.stamp = detection_array.stamp
            false_detection.class_name = random.choice(self._other_classes)
            false_detection.class_id = random.randint(1, 79)
            false_detection.confidence = float(random.uniform(0.3, 0.7))
            false_detection.center_x = float(random.uniform(0.1, 0.9))
            false_detection.center_y = float(random.uniform(0.1, 0.9))
            false_detection.width = float(random.uniform(0.03, 0.15))
            false_detection.height = float(random.uniform(0.03, 0.15))
            false_detection.pixel_center_x = int(false_detection.center_x * self._image_width)
            false_detection.pixel_center_y = int(false_detection.center_y * self._image_height)
            false_detection.pixel_width = int(false_detection.width * self._image_width)
            false_detection.pixel_height = int(false_detection.height * self._image_height)
            detections.append(false_detection)

        detection_array.detections = detections
        detection_array.count = len(detections)

        self._detection_pub.publish(detection_array)
        self._detection_count += 1

    def _report_status(self) -> None:
        """Report status."""
        self.get_logger().info(
            f'Fake detection: published={self._detection_count}, '
            f'target={self._target_class}, pattern={self._motion_pattern}'
        )

    def destroy_node(self) -> None:
        """Clean up."""
        self.get_logger().info('Shutting down fake detection node...')
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the fake detection node."""
    rclpy.init(args=args)
    node = FakeDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()