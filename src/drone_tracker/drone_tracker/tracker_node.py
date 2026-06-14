"""
Target tracking node.

Subscribes:
    /detections

Publishes:
    /target_error
"""

import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import Detection, DetectionArray, TargetError


class TrackingState(Enum):
    SEARCHING = "SEARCHING"
    LOCKED = "LOCKED"
    LOST = "LOST"


class ExponentialMovingAverage:
    def __init__(self, alpha: float = 0.4) -> None:
        self.alpha = alpha
        self.value: Optional[float] = None

    def update(self, new_value: float) -> float:
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value

        return self.value

    def reset(self) -> None:
        self.value = None


class TrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("tracker_node")

        self.declare_parameter("target_class", "person")
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("detection_timeout", 2.0)
        self.declare_parameter("reacquisition_timeout", 5.0)
        self.declare_parameter("smoothing_alpha", 0.4)
        self.declare_parameter("target_area_min", 0.001)
        self.declare_parameter("target_area_max", 0.8)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("proximity_threshold", 0.15)
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("target_error_topic", "/target_error")

        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
        self.min_confidence = self.get_parameter("min_confidence").value
        self.detection_timeout = self.get_parameter("detection_timeout").value
        self.reacquisition_timeout = self.get_parameter("reacquisition_timeout").value
        self.smoothing_alpha = self.get_parameter("smoothing_alpha").value
        self.target_area_min = self.get_parameter("target_area_min").value
        self.target_area_max = self.get_parameter("target_area_max").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.proximity_threshold = self.get_parameter("proximity_threshold").value
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)

        self.state = TrackingState.SEARCHING

        self.last_detection_time = 0.0
        self.last_target_center_x = 0.5
        self.last_target_center_y = 0.5
        self.last_target_confidence = 0.0
        self.last_target_area = 0.0

        self.error_x_filter = ExponentialMovingAverage(self.smoothing_alpha)
        self.error_y_filter = ExponentialMovingAverage(self.smoothing_alpha)

        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        error_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.detection_sub = self.create_subscription(
            DetectionArray,
            self.detections_topic,
            self.detection_callback,
            detection_qos,
        )

        self.error_pub = self.create_publisher(
            TargetError,
            self.target_error_topic,
            error_qos,
        )

        self.publish_timer = self.create_timer(
            1.0 / float(self.publish_rate),
            self.publish_error,
        )

        self.status_timer = self.create_timer(
            5.0,
            self.report_status,
        )

        self.get_logger().info(
            f"Tracker node started | detections_topic={self.detections_topic}, "
            f"target_error_topic={self.target_error_topic}, "
            f"target_class={self.target_class}, min_confidence={self.min_confidence}, "
            f"timeout={self.detection_timeout}s"
        )

    def detection_callback(self, msg: DetectionArray) -> None:
        current_time = time.time()

        best_detection: Optional[Detection] = None
        best_score = -999.0

        for detection in msg.detections:
            if detection.class_name.strip().lower() != self.target_class:
                continue

            if detection.confidence < self.min_confidence:
                continue

            area = detection.width * detection.height

            if area < self.target_area_min or area > self.target_area_max:
                continue

            score = detection.confidence

            if self.state in (TrackingState.LOCKED, TrackingState.LOST):
                dx = detection.center_x - self.last_target_center_x
                dy = detection.center_y - self.last_target_center_y
                distance = (dx * dx + dy * dy) ** 0.5

                if distance < self.proximity_threshold:
                    score += 0.3
                else:
                    score -= distance * 0.5

            if score > best_score:
                best_score = score
                best_detection = detection

        if best_detection is not None:
            self.last_detection_time = current_time
            self.last_target_center_x = best_detection.center_x
            self.last_target_center_y = best_detection.center_y
            self.last_target_confidence = best_detection.confidence
            self.last_target_area = best_detection.width * best_detection.height

            if self.state != TrackingState.LOCKED:
                self.get_logger().info(
                    f"Target acquired: {self.target_class} "
                    f"confidence={best_detection.confidence:.2f}"
                )

            self.state = TrackingState.LOCKED
            return

        time_since_last = current_time - self.last_detection_time

        if self.state == TrackingState.LOCKED:
            if time_since_last > self.detection_timeout:
                self.state = TrackingState.LOST
                self.get_logger().warning("Target lost.")

        elif self.state == TrackingState.LOST:
            if time_since_last > self.reacquisition_timeout:
                self.state = TrackingState.SEARCHING
                self.error_x_filter.reset()
                self.error_y_filter.reset()
                self.get_logger().info("Reacquisition timeout. Searching again.")

    def publish_error(self) -> None:
        current_time = time.time()

        # Do not keep publishing a fresh LOCKED error if detections stop arriving.
        # This protects the controller from chasing stale target coordinates.
        if (
            self.state == TrackingState.LOCKED
            and self.last_detection_time > 0.0
            and current_time - self.last_detection_time > self.detection_timeout
        ):
            self.state = TrackingState.LOST
            self.error_x_filter.reset()
            self.error_y_filter.reset()
            self.get_logger().warning("Target lost: detection stream timed out.")

        msg = TargetError()
        msg.stamp = self.get_clock().now().to_msg()
        msg.target_class = self.target_class
        msg.tracking_state = self.state.value

        if self.state == TrackingState.LOCKED:
            raw_error_x = self.last_target_center_x - 0.5
            raw_error_y = self.last_target_center_y - 0.5

            normalized_error_x = raw_error_x * 2.0
            normalized_error_y = raw_error_y * 2.0

            msg.error_x = float(self.error_x_filter.update(normalized_error_x))
            msg.error_y = float(self.error_y_filter.update(normalized_error_y))
            msg.target_visible = True
            msg.target_confidence = float(self.last_target_confidence)
            msg.target_area = float(self.last_target_area)
            msg.time_since_last_seen = 0.0

        else:
            msg.error_x = 0.0
            msg.error_y = 0.0
            msg.target_visible = False
            msg.target_confidence = 0.0
            msg.target_area = 0.0

            if self.last_detection_time > 0.0:
                msg.time_since_last_seen = float(current_time - self.last_detection_time)
            else:
                msg.time_since_last_seen = -1.0

        self.error_pub.publish(msg)

    def report_status(self) -> None:
        self.get_logger().info(
            f"Tracker status | state={self.state.value}, "
            f"target={self.target_class}, "
            f"confidence={self.last_target_confidence:.2f}, "
            f"area={self.last_target_area:.4f}"
        )

    def destroy_node(self) -> None:
        self.get_logger().info("Tracker node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
