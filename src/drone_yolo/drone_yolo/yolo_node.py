"""
YOLO object detection node for the drone vision system.

This node subscribes to camera images, runs Ultralytics YOLO inference,
and publishes detection results as custom DetectionArray messages.
"""

import time
from typing import Optional, List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from drone_interfaces.msg import Detection, DetectionArray


class YoloNode(Node):
    """ROS 2 node that performs YOLO object detection on camera images."""

    def __init__(self) -> None:
        """Initialize the YOLO detection node."""
        super().__init__('yolo_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('max_detections', 20)
        self.declare_parameter('half_precision', False)
        self.declare_parameter('verbose', False)

        # Read parameters
        self._model_path: str = self.get_parameter('model_path').value
        self._confidence_threshold: float = self.get_parameter('confidence_threshold').value
        self._iou_threshold: float = self.get_parameter('iou_threshold').value
        self._device: str = self.get_parameter('device').value
        self._input_size: int = self.get_parameter('input_size').value
        self._max_detections: int = self.get_parameter('max_detections').value
        self._half_precision: bool = self.get_parameter('half_precision').value
        self._verbose: bool = self.get_parameter('verbose').value

        # State
        self._bridge: CvBridge = CvBridge()
        self._model = None
        self._model_loaded: bool = False
        self._inference_count: int = 0
        self._total_inference_time: float = 0.0
        self._last_report_time: float = time.time()

        # Load model
        self._load_model()

        # QoS profiles
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriber
        self._image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            image_qos
        )

        # Publisher
        self._detection_pub = self.create_publisher(
            DetectionArray,
            '/detections',
            detection_qos
        )

        # Performance reporting timer
        self._report_timer = self.create_timer(10.0, self._report_performance)

        self.get_logger().info(
            f'YOLO node initialized: model={self._model_path}, '
            f'confidence={self._confidence_threshold}, device={self._device}'
        )

    def _load_model(self) -> None:
        """Load the YOLO model from the specified path."""
        try:
            from ultralytics import YOLO

            self.get_logger().info(f'Loading YOLO model from: {self._model_path}')
            self._model = YOLO(self._model_path)

            # Warm up the model with a dummy inference
            dummy_input = np.zeros((self._input_size, self._input_size, 3), dtype=np.uint8)
            self._model.predict(
                dummy_input,
                conf=self._confidence_threshold,
                iou=self._iou_threshold,
                device=self._device,
                half=self._half_precision,
                verbose=False
            )

            self._model_loaded = True
            self.get_logger().info('YOLO model loaded and warmed up successfully.')

        except ImportError:
            self.get_logger().error(
                'Ultralytics package not installed. Install with: pip install ultralytics'
            )
            self._model_loaded = False
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self._model_loaded = False

    def _image_callback(self, msg: Image) -> None:
        """
        Process incoming camera images through YOLO.

        Args:
            msg: The incoming ROS Image message.
        """
        if not self._model_loaded:
            return

        try:
            # Convert ROS Image to OpenCV
            frame: np.ndarray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_height, img_width = frame.shape[:2]

            # Run inference
            start_time = time.time()

            results = self._model.predict(
                frame,
                conf=self._confidence_threshold,
                iou=self._iou_threshold,
                device=self._device,
                half=self._half_precision,
                verbose=self._verbose,
                imgsz=self._input_size,
                max_det=self._max_detections
            )

            inference_time = time.time() - start_time
            self._total_inference_time += inference_time
            self._inference_count += 1

            # Build detection array message
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
                        detection = Detection()
                        detection.stamp = msg.header.stamp

                        # Class information
                        cls_id = int(boxes.cls[i].item())
                        detection.class_id = cls_id
                        detection.class_name = self._model.names.get(cls_id, f'class_{cls_id}')
                        detection.confidence = float(boxes.conf[i].item())

                        # Bounding box (xyxy format)
                        x1, y1, x2, y2 = boxes.xyxy[i].tolist()

                        # Pixel values
                        pixel_cx = int((x1 + x2) / 2.0)
                        pixel_cy = int((y1 + y2) / 2.0)
                        pixel_w = int(x2 - x1)
                        pixel_h = int(y2 - y1)

                        detection.pixel_center_x = pixel_cx
                        detection.pixel_center_y = pixel_cy
                        detection.pixel_width = pixel_w
                        detection.pixel_height = pixel_h

                        # Normalized values (0.0 to 1.0)
                        detection.center_x = float(pixel_cx) / float(img_width)
                        detection.center_y = float(pixel_cy) / float(img_height)
                        detection.width = float(pixel_w) / float(img_width)
                        detection.height = float(pixel_h) / float(img_height)

                        detections.append(detection)

            detection_array.detections = detections
            detection_array.count = len(detections)

            self._detection_pub.publish(detection_array)

        except Exception as e:
            self.get_logger().error(f'Error during YOLO inference: {e}')

    def _report_performance(self) -> None:
        """Report inference performance metrics."""
        if self._inference_count == 0:
            self.get_logger().info('YOLO node: No inferences performed yet.')
            return

        avg_time = self._total_inference_time / self._inference_count
        avg_fps = 1.0 / avg_time if avg_time > 0 else 0.0

        self.get_logger().info(
            f'YOLO Performance: avg_inference={avg_time*1000:.1f}ms, '
            f'avg_fps={avg_fps:.1f}, total_inferences={self._inference_count}'
        )

        # Reset counters
        self._inference_count = 0
        self._total_inference_time = 0.0

    def destroy_node(self) -> None:
        """Clean up resources."""
        self.get_logger().info('Shutting down YOLO node...')
        self._model = None
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the YOLO node."""
    rclpy.init(args=args)
    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()