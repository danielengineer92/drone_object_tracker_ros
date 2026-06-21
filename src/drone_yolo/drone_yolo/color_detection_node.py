"""
Color-based object detection node (OpenCV, no neural net).

A drop-in alternative to yolo_node for a single known-color target (e.g. a red
ball). Subscribes to camera frames and publishes the same DetectionArray, so the
tracker/control/mission stack is unchanged. Far cheaper than YOLO on a Pi and
needs no model or training.

Subscribes:
    /drone/camera/image_raw

Publishes:
    /drone/vision/detections
"""

import time
from typing import List

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from drone_interfaces.msg import Detection, DetectionArray
from drone_diagnostics.node_diagnostics import NodeDiagnostics
from drone_yolo.color_detection import ColorDetectParams, HsvBand, detect_colored_circles


class ColorDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("color_detection_node")

        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("detections_topic", "/drone/vision/detections")
        self.declare_parameter("target_class", "red_ball")
        self.declare_parameter("class_id", 0)
        self.declare_parameter("max_detections", 1)
        self.declare_parameter("process_every_n_frames", 1)

        # HSV thresholds. Red wraps the hue circle, so two hue bands share the
        # same saturation/value floors. For other colors, set hue band 2 equal to
        # band 1 (a redundant band is harmless).
        self.declare_parameter("h_min1", 0)
        self.declare_parameter("h_max1", 10)
        self.declare_parameter("h_min2", 170)
        self.declare_parameter("h_max2", 179)
        self.declare_parameter("s_min", 120)
        self.declare_parameter("s_max", 255)
        self.declare_parameter("v_min", 70)
        self.declare_parameter("v_max", 255)

        self.declare_parameter("min_radius_px", 6)
        self.declare_parameter("max_radius_px", 100000)
        self.declare_parameter("blur_ksize", 5)
        self.declare_parameter("morph_ksize", 5)
        self.declare_parameter("min_fill_ratio", 0.55)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.target_class = str(self.get_parameter("target_class").value)
        self.class_id = int(self.get_parameter("class_id").value)
        self.max_detections = max(1, int(self.get_parameter("max_detections").value))
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))

        self.params = ColorDetectParams(
            bands=[
                HsvBand(
                    int(self.get_parameter("h_min1").value), int(self.get_parameter("h_max1").value),
                    int(self.get_parameter("s_min").value), int(self.get_parameter("s_max").value),
                    int(self.get_parameter("v_min").value), int(self.get_parameter("v_max").value),
                ),
                HsvBand(
                    int(self.get_parameter("h_min2").value), int(self.get_parameter("h_max2").value),
                    int(self.get_parameter("s_min").value), int(self.get_parameter("s_max").value),
                    int(self.get_parameter("v_min").value), int(self.get_parameter("v_max").value),
                ),
            ],
            min_radius_px=int(self.get_parameter("min_radius_px").value),
            max_radius_px=int(self.get_parameter("max_radius_px").value),
            blur_ksize=int(self.get_parameter("blur_ksize").value),
            morph_ksize=int(self.get_parameter("morph_ksize").value),
            min_fill_ratio=float(self.get_parameter("min_fill_ratio").value),
        )

        self.bridge = CvBridge()
        self._received_frames = 0
        self._skipped_frames = 0
        self._published_arrays = 0
        self._last_detection_count = 0
        self._detect_count = 0
        self._total_detect_time = 0.0

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
        )
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self._image_callback, image_qos)
        self.detection_pub = self.create_publisher(DetectionArray, self.detections_topic, detection_qos)

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.image_topic, "camera_frames")
        self.diagnostics.add_output(self.detections_topic, "detections")
        self.report_timer = self.create_timer(5.0, self._report_performance)

        self.get_logger().info(
            f"Color detection node initialized | image_topic={self.image_topic}, "
            f"detections_topic={self.detections_topic}, target_class='{self.target_class}', "
            f"hue_bands=[{self.params.bands[0].h_min}-{self.params.bands[0].h_max}, "
            f"{self.params.bands[1].h_min}-{self.params.bands[1].h_max}], "
            f"s>={self.params.bands[0].s_min}, v>={self.params.bands[0].v_min}, "
            f"min_radius_px={self.params.min_radius_px}"
        )

    def _image_callback(self, msg: Image) -> None:
        self._received_frames += 1
        self.diagnostics.mark_received(
            self.image_topic, summary=f"frames={self._received_frames}"
        )
        if self._received_frames % self.process_every_n_frames != 0:
            self._skipped_frames += 1
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_height, img_width = frame.shape[:2]

            start = time.time()
            circles = detect_colored_circles(frame, self.params, max_results=self.max_detections)
            self._total_detect_time += time.time() - start
            self._detect_count += 1

            detection_array = DetectionArray()
            detection_array.stamp = msg.header.stamp
            detection_array.image_width = img_width
            detection_array.image_height = img_height

            detections: List[Detection] = []
            for circle in circles:
                pixel_w = int(round(circle.radius * 2.0))
                pixel_h = pixel_w
                detection = Detection()
                detection.stamp = msg.header.stamp
                detection.class_id = self.class_id
                detection.class_name = self.target_class
                detection.confidence = float(circle.confidence)
                detection.pixel_center_x = int(round(circle.cx))
                detection.pixel_center_y = int(round(circle.cy))
                detection.pixel_width = pixel_w
                detection.pixel_height = pixel_h
                detection.center_x = float(circle.cx) / float(img_width)
                detection.center_y = float(circle.cy) / float(img_height)
                detection.width = float(pixel_w) / float(img_width)
                detection.height = float(pixel_h) / float(img_height)
                detections.append(detection)

            detection_array.detections = detections
            detection_array.count = len(detections)

            self.detection_pub.publish(detection_array)
            self._published_arrays += 1
            self._last_detection_count = detection_array.count
            self.diagnostics.mark_published(
                self.detections_topic,
                summary=f"arrays={self._published_arrays}, last_count={detection_array.count}",
            )
        except Exception as exc:  # noqa: BLE001 - keep the node alive on a bad frame
            self.get_logger().error(f"Error during color detection: {exc}", throttle_duration_sec=2.0)

    def _report_performance(self) -> None:
        if self._detect_count == 0:
            self.get_logger().info(
                f"Color detect status | no frames processed yet, received={self._received_frames}, "
                f"last_image_age={self.diagnostics.format_age(self.image_topic)}"
            )
            return
        avg_ms = (self._total_detect_time / self._detect_count) * 1000.0
        avg_fps = 1000.0 / avg_ms if avg_ms > 0.0 else 0.0
        self.get_logger().info(
            f"Color detect status | avg={avg_ms:.2f} ms, fps={avg_fps:.0f}, "
            f"processed={self._detect_count}, received={self._received_frames}, "
            f"skipped={self._skipped_frames}, arrays={self._published_arrays}, "
            f"last_count={self._last_detection_count}, "
            f"last_image_age={self.diagnostics.format_age(self.image_topic)}"
        )
        self._detect_count = 0
        self._total_detect_time = 0.0

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down color detection node.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ColorDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
