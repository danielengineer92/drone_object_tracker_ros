"""
YOLO object detection node for the drone vision system.

Subscribes:
    /drone/camera/image_raw

Publishes:
    /drone/vision/detections
"""

import time
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from drone_interfaces.msg import Detection, DetectionArray
from drone_diagnostics.node_diagnostics import NodeDiagnostics


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_node")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("half_precision", False)
        self.declare_parameter("verbose", False)
        self.declare_parameter("target_class", "")
        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("detections_topic", "/drone/vision/detections")
        self.declare_parameter("process_every_n_frames", 1)

        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.iou_threshold = self.get_parameter("iou_threshold").value
        self.device = self.get_parameter("device").value
        self.input_size = self.get_parameter("input_size").value
        self.max_detections = self.get_parameter("max_detections").value
        self.half_precision = self.get_parameter("half_precision").value
        self.verbose = bool(self.get_parameter("verbose").value)
        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self._received_frames = 0
        self._skipped_frames = 0
        self._published_detection_arrays = 0
        self._last_detection_count = 0
        self._last_image_time = 0.0
        self._target_class_ids: Optional[List[int]] = None

        self.bridge = CvBridge()
        self.model = None
        self.model_loaded = False

        self.inference_count = 0
        self.total_inference_time = 0.0

        self._load_model()
        self._target_class_ids = self._resolve_target_class_ids()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            image_qos,
        )

        self.detection_pub = self.create_publisher(
            DetectionArray,
            self.detections_topic,
            detection_qos,
        )

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.image_topic, "camera_frames")
        self.diagnostics.add_output(self.detections_topic, "detections")

        self.report_timer = self.create_timer(5.0, self._report_performance)

        self.get_logger().info(
            f"YOLO node initialized | image_topic={self.image_topic}, "
            f"detections_topic={self.detections_topic}, model={self.model_path}, "
            f"confidence={self.confidence_threshold}, device={self.device}, "
            f"target_class='{self.target_class or 'all'}', "
            f"process_every_n_frames={self.process_every_n_frames}"
        )

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO

            self.get_logger().info(f"Loading YOLO model: {self.model_path}")
            self.model = YOLO(self.model_path)

            dummy = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)

            self.model.predict(
                dummy,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                device=self.device,
                half=self.half_precision,
                verbose=False,
                imgsz=self.input_size,
            )

            self.model_loaded = True
            self.get_logger().info("YOLO model loaded successfully.")

        except ImportError:
            self.get_logger().error(
                "Ultralytics package not installed. Install with: pip install ultralytics"
            )
            self.model_loaded = False

        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            self.model_loaded = False


    def _resolve_target_class_ids(self) -> Optional[List[int]]:
        """Resolve the configured target class name to Ultralytics class ids.

        Passing class ids into model.predict lets YOLO skip unused classes and
        reduces post-processing work on the Raspberry Pi. We still keep the
        name filter in _image_callback as a safety check.
        """
        if not self.target_class or not self.model_loaded or self.model is None:
            return None

        names = getattr(self.model, "names", {})
        matches = [
            int(class_id)
            for class_id, class_name in names.items()
            if str(class_name).strip().lower() == self.target_class
        ]

        if not matches:
            self.get_logger().warning(
                f"target_class={self.target_class!r} was not found in model names; "
                "YOLO will run all classes and filter by name afterward."
            )
            return None

        self.get_logger().info(
            f"YOLO class filter active: {self.target_class} -> class ids {matches}"
        )
        return matches

    def _image_callback(self, msg: Image) -> None:
        self._received_frames += 1
        self._last_image_time = time.time()
        self.diagnostics.mark_received(
            self.image_topic,
            summary=f"frames={self._received_frames}, stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
        )

        if not self.model_loaded:
            self.get_logger().warning("Received image but YOLO model is not loaded; dropping frame.", throttle_duration_sec=2.0)
            return

        if self._received_frames % self.process_every_n_frames != 0:
            self._skipped_frames += 1
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_height, img_width = frame.shape[:2]

            start_time = time.time()

            predict_kwargs = {
                "conf": self.confidence_threshold,
                "iou": self.iou_threshold,
                "device": self.device,
                "half": self.half_precision,
                "verbose": self.verbose,
                "imgsz": self.input_size,
                "max_det": self.max_detections,
            }
            if self._target_class_ids is not None:
                predict_kwargs["classes"] = self._target_class_ids

            results = self.model.predict(frame, **predict_kwargs)

            inference_time = time.time() - start_time
            self.total_inference_time += inference_time
            self.inference_count += 1

            detection_array = DetectionArray()
            detection_array.stamp = msg.header.stamp
            detection_array.image_width = img_width
            detection_array.image_height = img_height

            detections: List[Detection] = []

            if results and len(results) > 0:
                result = results[0]

                if result.boxes is not None and len(result.boxes) > 0:
                    boxes = result.boxes

                    for i in range(len(boxes)):
                        cls_id = int(boxes.cls[i].item())
                        class_name = self.model.names.get(cls_id, f"class_{cls_id}")

                        if self.target_class and class_name.lower() != self.target_class:
                            continue

                        x1, y1, x2, y2 = boxes.xyxy[i].tolist()

                        pixel_cx = int((x1 + x2) / 2.0)
                        pixel_cy = int((y1 + y2) / 2.0)
                        pixel_w = int(x2 - x1)
                        pixel_h = int(y2 - y1)

                        detection = Detection()
                        detection.stamp = msg.header.stamp
                        detection.class_id = cls_id
                        detection.class_name = class_name
                        detection.confidence = float(boxes.conf[i].item())

                        detection.pixel_center_x = pixel_cx
                        detection.pixel_center_y = pixel_cy
                        detection.pixel_width = pixel_w
                        detection.pixel_height = pixel_h

                        detection.center_x = float(pixel_cx) / float(img_width)
                        detection.center_y = float(pixel_cy) / float(img_height)
                        detection.width = float(pixel_w) / float(img_width)
                        detection.height = float(pixel_h) / float(img_height)

                        detections.append(detection)

            detection_array.detections = detections
            detection_array.count = len(detections)

            self.detection_pub.publish(detection_array)
            self._published_detection_arrays += 1
            self._last_detection_count = detection_array.count
            self.diagnostics.mark_published(
                self.detections_topic,
                summary=f"arrays={self._published_detection_arrays}, last_count={detection_array.count}",
            )

        except Exception as exc:
            self.get_logger().error(f"Error during YOLO inference: {exc}")

    def _report_performance(self) -> None:
        if self.inference_count == 0:
            self.get_logger().info(
                f"YOLO status | no inferences yet, frames_received={self._received_frames}, "
                f"frames_skipped={self._skipped_frames}, arrays_published={self._published_detection_arrays}, "
                f"last_image_age={self.diagnostics.format_age(self.image_topic)}"
            )
            return

        avg_time = self.total_inference_time / self.inference_count
        avg_fps = 1.0 / avg_time if avg_time > 0.0 else 0.0

        self.get_logger().info(
            f"YOLO status | avg_inference={avg_time * 1000:.1f} ms, "
            f"inference_fps={avg_fps:.1f}, inference_count={self.inference_count}, "
            f"frames_received={self._received_frames}, frames_skipped={self._skipped_frames}, "
            f"arrays_published={self._published_detection_arrays}, last_detection_count={self._last_detection_count}, "
            f"last_image_age={self.diagnostics.format_age(self.image_topic)}"
        )

        self.inference_count = 0
        self.total_inference_time = 0.0

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down YOLO node.")
        self.model = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
