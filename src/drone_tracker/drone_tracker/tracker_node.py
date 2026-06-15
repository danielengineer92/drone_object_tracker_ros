"""
Target tracking node.

Subscribes:
    /detections

Publishes:
    /target_error

Behavior:
    - Filters detections by target class, confidence, and area.
    - Requires multiple consecutive detections before declaring LOCKED.
    - Uses COASTING state to survive short YOLO dropouts.
    - Publishes smooth normalized target error for the control node.
"""

import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import Detection, DetectionArray, TargetError
from drone_diagnostics.node_diagnostics import NodeDiagnostics


class TrackingState(Enum):
    SEARCHING = "SEARCHING"
    ACQUIRING = "ACQUIRING"
    LOCKED = "LOCKED"
    COASTING = "COASTING"
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

        # Core tracking parameters
        self.declare_parameter("target_class", "person")
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("reacquisition_timeout", 5.0)
        self.declare_parameter("smoothing_alpha", 0.4)
        self.declare_parameter("target_area_min", 0.001)
        self.declare_parameter("target_area_max", 0.8)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("proximity_threshold", 0.15)

        # Debounce / anti-bounce parameters
        self.declare_parameter("lock_confirm_frames", 3)
        self.declare_parameter("coast_timeout", 0.75)

        # Topics
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("target_error_topic", "/target_error")

        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.reacquisition_timeout = float(self.get_parameter("reacquisition_timeout").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.target_area_min = float(self.get_parameter("target_area_min").value)
        self.target_area_max = float(self.get_parameter("target_area_max").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.proximity_threshold = float(self.get_parameter("proximity_threshold").value)

        self.lock_confirm_frames = int(self.get_parameter("lock_confirm_frames").value)
        self.coast_timeout = float(self.get_parameter("coast_timeout").value)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)

        self.state = TrackingState.SEARCHING

        self.last_detection_time = 0.0
        self.last_target_center_x = 0.5
        self.last_target_center_y = 0.5
        self.last_target_confidence = 0.0
        self.last_target_area = 0.0

        self.lock_candidate_count = 0

        self.detection_message_count = 0
        self.target_error_publish_count = 0
        self.last_detection_array_count = 0

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
            1.0 / self.publish_rate,
            self.publish_error,
        )

        self.status_timer = self.create_timer(
            5.0,
            self.report_status,
        )

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.detections_topic, "detections")
        self.diagnostics.add_output(self.target_error_topic, "target_error")

        self.get_logger().info(
            f"Tracker node started | detections_topic={self.detections_topic}, "
            f"target_error_topic={self.target_error_topic}, "
            f"target_class={self.target_class}, "
            f"min_confidence={self.min_confidence}, "
            f"lock_confirm_frames={self.lock_confirm_frames}, "
            f"coast_timeout={self.coast_timeout}s, "
            f"reacquisition_timeout={self.reacquisition_timeout}s"
        )

    def detection_callback(self, msg: DetectionArray) -> None:
        current_time = time.time()
        self.detection_message_count += 1
        self.last_detection_array_count = msg.count

        self.diagnostics.mark_received(
            self.detections_topic,
            summary=f"messages={self.detection_message_count}, detections={msg.count}",
        )

        best_detection = self.select_best_detection(msg)

        if best_detection is not None:
            self.handle_target_seen(best_detection, current_time)
        else:
            self.handle_target_missing(current_time)

    def select_best_detection(self, msg: DetectionArray) -> Optional[Detection]:
        best_detection: Optional[Detection] = None
        best_score = -999.0

        for detection in msg.detections:
            class_name = detection.class_name.strip().lower()

            if class_name != self.target_class:
                continue

            if detection.confidence < self.min_confidence:
                continue

            area = detection.width * detection.height

            if area < self.target_area_min or area > self.target_area_max:
                continue

            score = float(detection.confidence)

            # Prefer detections near the previous locked target.
            if self.state in (TrackingState.LOCKED, TrackingState.COASTING, TrackingState.LOST):
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

        return best_detection

    def handle_target_seen(self, detection: Detection, current_time: float) -> None:
        previous_state = self.state

        self.last_detection_time = current_time
        self.last_target_center_x = float(detection.center_x)
        self.last_target_center_y = float(detection.center_y)
        self.last_target_confidence = float(detection.confidence)
        self.last_target_area = float(detection.width * detection.height)

        # Require multiple consecutive detections before declaring a fresh lock.
        if self.state in (TrackingState.SEARCHING, TrackingState.LOST, TrackingState.ACQUIRING):
            self.lock_candidate_count += 1

            if self.lock_candidate_count < self.lock_confirm_frames:
                self.state = TrackingState.ACQUIRING

                if previous_state != TrackingState.ACQUIRING:
                    self.get_logger().info(
                        f"Target candidate found: {self.target_class} "
                        f"confidence={detection.confidence:.2f} "
                        f"confirm={self.lock_candidate_count}/{self.lock_confirm_frames}"
                    )
                return

        self.lock_candidate_count = self.lock_confirm_frames
        self.state = TrackingState.LOCKED

        if previous_state != TrackingState.LOCKED:
            self.get_logger().info(
                f"Target locked: {self.target_class} "
                f"confidence={detection.confidence:.2f}"
            )

    def handle_target_missing(self, current_time: float) -> None:
        if self.last_detection_time > 0.0:
            time_since_last = current_time - self.last_detection_time
        else:
            time_since_last = 999.0

        if self.state == TrackingState.LOCKED:
            if time_since_last <= self.coast_timeout:
                self.state = TrackingState.COASTING
            else:
                self.set_lost("Target lost.")

        elif self.state == TrackingState.COASTING:
            if time_since_last > self.coast_timeout:
                self.set_lost("Target lost after coasting.")

        elif self.state == TrackingState.ACQUIRING:
            # One bad frame during acquire means we did not really lock yet.
            self.lock_candidate_count = 0
            self.state = TrackingState.SEARCHING

        elif self.state == TrackingState.LOST:
            if time_since_last > self.reacquisition_timeout:
                self.state = TrackingState.SEARCHING
                self.lock_candidate_count = 0
                self.error_x_filter.reset()
                self.error_y_filter.reset()
                self.get_logger().info("Reacquisition timeout. Searching again.")

    def set_lost(self, reason: str) -> None:
        if self.state != TrackingState.LOST:
            self.get_logger().warning(reason)

        self.state = TrackingState.LOST
        self.lock_candidate_count = 0
        self.error_x_filter.reset()
        self.error_y_filter.reset()

    def publish_error(self) -> None:
        current_time = time.time()

        # Timer-side stale protection.
        # If detections stop completely, do not keep publishing a fake fresh target.
        if (
            self.state in (TrackingState.LOCKED, TrackingState.COASTING)
            and self.last_detection_time > 0.0
            and current_time - self.last_detection_time > self.coast_timeout
        ):
            self.set_lost("Target lost: detection stream timed out.")

        msg = TargetError()
        msg.stamp = self.get_clock().now().to_msg()
        msg.target_class = self.target_class
        msg.tracking_state = self.state.value

        if self.state in (TrackingState.LOCKED, TrackingState.COASTING):
            raw_error_x = self.last_target_center_x - 0.5
            raw_error_y = self.last_target_center_y - 0.5

            normalized_error_x = raw_error_x * 2.0
            normalized_error_y = raw_error_y * 2.0

            msg.error_x = float(self.error_x_filter.update(normalized_error_x))
            msg.error_y = float(self.error_y_filter.update(normalized_error_y))
            msg.target_visible = True
            msg.target_confidence = float(self.last_target_confidence)
            msg.target_area = float(self.last_target_area)
            msg.time_since_last_seen = float(current_time - self.last_detection_time)

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
        self.target_error_publish_count += 1

        self.diagnostics.mark_published(
            self.target_error_topic,
            summary=(
                f"messages={self.target_error_publish_count}, "
                f"state={self.state.value}, "
                f"visible={msg.target_visible}"
            ),
        )

    def report_status(self) -> None:
        if self.last_detection_time > 0.0:
            target_age = time.time() - self.last_detection_time
        else:
            target_age = -1.0

        self.get_logger().info(
            f"Tracker status | state={self.state.value}, "
            f"target={self.target_class}, "
            f"detection_messages={self.detection_message_count}, "
            f"last_detection_count={self.last_detection_array_count}, "
            f"target_error_messages={self.target_error_publish_count}, "
            f"last_detection_age={self.diagnostics.format_age(self.detections_topic)}, "
            f"target_age={target_age:.2f}s, "
            f"lock_candidates={self.lock_candidate_count}/{self.lock_confirm_frames}, "
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
