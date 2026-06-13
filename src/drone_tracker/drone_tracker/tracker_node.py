"""
Target tracking node for the drone vision system.

This node selects a target object from detections, maintains target lock,
performs reacquisition when target is lost, and calculates the image-center
tracking error for the control system.
"""

import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_interfaces.msg import DetectionArray, Detection, TargetError


class TrackingState(Enum):
    """Enumeration of possible tracking states."""
    SEARCHING = "SEARCHING"
    LOCKED = "LOCKED"
    LOST = "LOST"


class ExponentialMovingAverage:
    """Simple exponential moving average filter for smoothing."""

    def __init__(self, alpha: float = 0.3) -> None:
        """
        Initialize the EMA filter.

        Args:
            alpha: Smoothing factor (0.0 to 1.0). Higher = less smoothing.
        """
        self._alpha: float = alpha
        self._value: Optional[float] = None

    def update(self, new_value: float) -> float:
        """
        Update the filter with a new value and return the smoothed result.

        Args:
            new_value: The new measurement.

        Returns:
            The smoothed value.
        """
        if self._value is None:
            self._value = new_value
        else:
            self._value = self._alpha * new_value + (1.0 - self._alpha) * self._value
        return self._value

    def reset(self) -> None:
        """Reset the filter state."""
        self._value = None

    @property
    def value(self) -> Optional[float]:
        """Get the current filtered value."""
        return self._value


class TrackerNode(Node):
    """ROS 2 node that tracks a selected target and calculates tracking error."""

    def __init__(self) -> None:
        """Initialize the tracker node."""
        super().__init__('tracker_node')

        # Declare parameters
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('min_confidence', 0.4)
        self.declare_parameter('detection_timeout', 2.0)
        self.declare_parameter('reacquisition_timeout', 5.0)
        self.declare_parameter('smoothing_alpha', 0.4)
        self.declare_parameter('target_area_min', 0.001)
        self.declare_parameter('target_area_max', 0.8)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('proximity_threshold', 0.15)

        # Read parameters
        self._target_class: str = self.get_parameter('target_class').value
        self._min_confidence: float = self.get_parameter('min_confidence').value
        self._detection_timeout: float = self.get_parameter('detection_timeout').value
        self._reacquisition_timeout: float = self.get_parameter('reacquisition_timeout').value
        self._smoothing_alpha: float = self.get_parameter('smoothing_alpha').value
        self._target_area_min: float = self.get_parameter('target_area_min').value
        self._target_area_max: float = self.get_parameter('target_area_max').value
        self._publish_rate: float = self.get_parameter('publish_rate').value
        self._proximity_threshold: float = self.get_parameter('proximity_threshold').value

        # Tracking state
        self._state: TrackingState = TrackingState.SEARCHING
        self._last_detection_time: float = 0.0
        self._last_target_center_x: float = 0.5
        self._last_target_center_y: float = 0.5
        self._last_target_confidence: float = 0.0
        self._last_target_area: float = 0.0

        # Smoothing filters
        self._error_x_filter: ExponentialMovingAverage = ExponentialMovingAverage(self._smoothing_alpha)
        self._error_y_filter: ExponentialMovingAverage = ExponentialMovingAverage(self._smoothing_alpha)

        # QoS profiles
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        error_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriber
        self._detection_sub = self.create_subscription(
            DetectionArray,
            '/detections',
            self._detection_callback,
            detection_qos
        )

        # Publisher
        self._error_pub = self.create_publisher(
            TargetError,
            '/target_error',
            error_qos
        )

        # Publish timer
        publish_period: float = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_error)

        # Status timer
        self._status_timer = self.create_timer(5.0, self._report_status)

        self.get_logger().info(
            f'Tracker node initialized: target_class={self._target_class}, '
            f'min_confidence={self._min_confidence}, '
            f'timeout={self._detection_timeout}s'
        )

    def _detection_callback(self, msg: DetectionArray) -> None:
        """
        Process incoming detection arrays and update tracking state.

        Args:
            msg: The detection array message.
        """
        current_time = time.time()

        # Find best matching target
        best_detection: Optional[Detection] = None
        best_score: float = 0.0

        for detection in msg.detections:
            if detection.class_name != self._target_class:
                continue

            if detection.confidence < self._min_confidence:
                continue

            # Calculate area
            area = detection.width * detection.height
            if area < self._target_area_min or area > self._target_area_max:
                continue

            # Score based on confidence and proximity to last known position
            score = detection.confidence

            if self._state == TrackingState.LOCKED or self._state == TrackingState.LOST:
                # Prefer detections closer to last known position
                dx = detection.center_x - self._last_target_center_x
                dy = detection.center_y - self._last_target_center_y
                distance = (dx * dx + dy * dy) ** 0.5

                if distance < self._proximity_threshold:
                    score += 0.3  # Proximity bonus
                else:
                    score -= distance * 0.5  # Distance penalty

            if score > best_score:
                best_score = score
                best_detection = detection

        # Update state based on detection result
        if best_detection is not None:
            self._last_detection_time = current_time
            self._last_target_center_x = best_detection.center_x
            self._last_target_center_y = best_detection.center_y
            self._last_target_confidence = best_detection.confidence
            self._last_target_area = best_detection.width * best_detection.height

            if self._state != TrackingState.LOCKED:
                self.get_logger().info(
                    f'Target acquired: {self._target_class} '
                    f'(confidence={best_detection.confidence:.2f})'
                )

            self._state = TrackingState.LOCKED

        else:
            # No matching detection found
            time_since_last = current_time - self._last_detection_time

            if self._state == TrackingState.LOCKED:
                if time_since_last > self._detection_timeout:
                    self._state = TrackingState.LOST
                    self.get_logger().warning(
                        f'Target lost after {self._detection_timeout}s timeout.'
                    )

            elif self._state == TrackingState.LOST:
                if time_since_last > self._reacquisition_timeout:
                    self._state = TrackingState.SEARCHING
                    self._error_x_filter.reset()
                    self._error_y_filter.reset()
                    self.get_logger().info(
                        'Reacquisition timeout expired. Returning to SEARCHING state.'
                    )

    def _publish_error(self) -> None:
        """Publish the current tracking error."""
        current_time = time.time()

        error_msg = TargetError()
        error_msg.stamp = self.get_clock().now().to_msg()
        error_msg.target_class = self._target_class
        error_msg.tracking_state = self._state.value

        if self._state == TrackingState.LOCKED:
            # Calculate error from image center (0.5, 0.5)
            raw_error_x = self._last_target_center_x - 0.5
            raw_error_y = self._last_target_center_y - 0.5

            # Normalize to -1.0 to 1.0 range
            normalized_error_x = raw_error_x * 2.0
            normalized_error_y = raw_error_y * 2.0

            # Apply smoothing
            smoothed_x = self._error_x_filter.update(normalized_error_x)
            smoothed_y = self._error_y_filter.update(normalized_error_y)

            error_msg.error_x = float(smoothed_x)
            error_msg.error_y = float(smoothed_y)
            error_msg.target_visible = True
            error_msg.target_confidence = self._last_target_confidence
            error_msg.target_area = self._last_target_area
            error_msg.time_since_last_seen = 0.0

        elif self._state == TrackingState.LOST:
            # Report last known error but mark as not visible
            time_since = current_time - self._last_detection_time

            error_msg.error_x = 0.0
            error_msg.error_y = 0.0
            error_msg.target_visible = False
            error_msg.target_confidence = 0.0
            error_msg.target_area = 0.0
            error_msg.time_since_last_seen = float(time_since)

        else:
            # SEARCHING state
            error_msg.error_x = 0.0
            error_msg.error_y = 0.0
            error_msg.target_visible = False
            error_msg.target_confidence = 0.0
            error_msg.target_area = 0.0
            error_msg.time_since_last_seen = float(
                current_time - self._last_detection_time
            ) if self._last_detection_time > 0 else -1.0

        self._error_pub.publish(error_msg)

    def _report_status(self) -> None:
        """Report tracking status periodically."""
        self.get_logger().info(
            f'Tracker status: state={self._state.value}, '
            f'target={self._target_class}, '
            f'last_confidence={self._last_target_confidence:.2f}, '
            f'last_area={self._last_target_area:.4f}'
        )

    def destroy_node(self) -> None:
        """Clean up resources."""
        self.get_logger().info('Shutting down tracker node...')
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the tracker node."""
    rclpy.init(args=args)
    node = TrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()