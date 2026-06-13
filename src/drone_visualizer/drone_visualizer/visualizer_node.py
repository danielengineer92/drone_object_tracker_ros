"""
Real-time visualization node for the drone vision system.

This node displays the camera feed with overlaid detection bounding boxes,
target lock status, tracking error indicators, and telemetry information.
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

from drone_interfaces.msg import DetectionArray, Detection, TargetError, DroneTelemetry, ControlCommand


class VisualizerNode(Node):
    """ROS 2 node that provides real-time visualization of the drone vision system."""

    def __init__(self) -> None:
        """Initialize the visualizer node."""
        super().__init__('visualizer_node')

        # Declare parameters
        self.declare_parameter('window_name', 'Drone Vision System')
        self.declare_parameter('display_width', 960)
        self.declare_parameter('display_height', 720)
        self.declare_parameter('show_detections', True)
        self.declare_parameter('show_tracking', True)
        self.declare_parameter('show_telemetry', True)
        self.declare_parameter('show_control', True)
        self.declare_parameter('display_fps', 30.0)
        self.declare_parameter('headless', False)

        # Read parameters
        self._window_name: str = self.get_parameter('window_name').value
        self._display_width: int = self.get_parameter('display_width').value
        self._display_height: int = self.get_parameter('display_height').value
        self._show_detections: bool = self.get_parameter('show_detections').value
        self._show_tracking: bool = self.get_parameter('show_tracking').value
        self._show_telemetry: bool = self.get_parameter('show_telemetry').value
        self._show_control: bool = self.get_parameter('show_control').value
        self._display_fps: float = self.get_parameter('display_fps').value
        self._headless: bool = self.get_parameter('headless').value

        # State
        self._bridge: CvBridge = CvBridge()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_detections: Optional[DetectionArray] = None
        self._latest_target_error: Optional[TargetError] = None
        self._latest_telemetry: Optional[DroneTelemetry] = None
        self._latest_command: Optional[ControlCommand] = None
        self._frame_count: int = 0
        self._last_fps_time: float = time.time()
        self._display_measured_fps: float = 0.0

        # Colors (BGR)
        self._color_green = (0, 255, 0)
        self._color_red = (0, 0, 255)
        self._color_blue = (255, 128, 0)
        self._color_yellow = (0, 255, 255)
        self._color_white = (255, 255, 255)
        self._color_cyan = (255, 255, 0)
        self._color_orange = (0, 165, 255)
        self._color_magenta = (255, 0, 255)

        # QoS profiles
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self._image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            image_qos
        )

        self._detection_sub = self.create_subscription(
            DetectionArray,
            '/detections',
            self._detection_callback,
            reliable_qos
        )

        self._target_error_sub = self.create_subscription(
            TargetError,
            '/target_error',
            self._target_error_callback,
            reliable_qos
        )

        self._telemetry_sub = self.create_subscription(
            DroneTelemetry,
            '/drone/telemetry',
            self._telemetry_callback,
            reliable_qos
        )

        self._command_sub = self.create_subscription(
            ControlCommand,
            '/control_command',
            self._command_callback,
            reliable_qos
        )

        # Display timer
        display_period = 1.0 / self._display_fps
        self._display_timer = self.create_timer(display_period, self._display_callback)

        if not self._headless:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, self._display_width, self._display_height)

        self.get_logger().info(
            f'Visualizer node initialized: {self._display_width}x{self._display_height}, '
            f'headless={self._headless}'
        )

    def _image_callback(self, msg: Image) -> None:
        """Store the latest camera frame."""
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def _detection_callback(self, msg: DetectionArray) -> None:
        """Store the latest detections."""
        self._latest_detections = msg

    def _target_error_callback(self, msg: TargetError) -> None:
        """Store the latest target error."""
        self._latest_target_error = msg

    def _telemetry_callback(self, msg: DroneTelemetry) -> None:
        """Store the latest telemetry."""
        self._latest_telemetry = msg

    def _command_callback(self, msg: ControlCommand) -> None:
        """Store the latest control command."""
        self._latest_command = msg

    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """
        Draw detection bounding boxes on the frame.

        Args:
            frame: The input frame.

        Returns:
            Frame with detection overlays.
        """
        if self._latest_detections is None:
            return frame

        img_h, img_w = frame.shape[:2]
        target_class = ""
        if self._latest_target_error is not None:
            target_class = self._latest_target_error.target_class

        for detection in self._latest_detections.detections:
            # Calculate pixel coordinates
            cx = int(detection.center_x * img_w)
            cy = int(detection.center_y * img_h)
            w = int(detection.width * img_w)
            h = int(detection.height * img_h)

            x1 = cx - w // 2
            y1 = cy - h // 2
            x2 = cx + w // 2
            y2 = cy + h // 2

            # Choose color based on whether this is the tracked target
            is_target = (detection.class_name == target_class)
            if is_target and self._latest_target_error is not None and self._latest_target_error.target_visible:
                color = self._color_green
                thickness = 3
            else:
                color = self._color_blue
                thickness = 2

            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

            # Draw label
            label = f'{detection.class_name} {detection.confidence:.2f}'
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]

            cv2.rectangle(
                frame,
                (x1, y1 - label_size[1] - 8),
                (x1 + label_size[0] + 4, y1),
                color,
                -1
            )
            cv2.putText(
                frame, label,
                (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0), 1, cv2.LINE_AA
            )

            # Draw center point
            cv2.circle(frame, (cx, cy), 4, color, -1)

        return frame

    def _draw_tracking(self, frame: np.ndarray) -> np.ndarray:
        """
        Draw tracking information on the frame.

        Args:
            frame: The input frame.

        Returns:
            Frame with tracking overlays.
        """
        if self._latest_target_error is None:
            return frame

        img_h, img_w = frame.shape[:2]
        target = self._latest_target_error

        # Draw image center crosshair
        center_x = img_w // 2
        center_y = img_h // 2
        crosshair_size = 20

        cv2.line(frame,
                 (center_x - crosshair_size, center_y),
                 (center_x + crosshair_size, center_y),
                 self._color_white, 1)
        cv2.line(frame,
                 (center_x, center_y - crosshair_size),
                 (center_x, center_y + crosshair_size),
                 self._color_white, 1)

        # Draw tracking error vector
        if target.target_visible:
            error_end_x = center_x + int(target.error_x * img_w * 0.5)
            error_end_y = center_y + int(target.error_y * img_h * 0.5)

            cv2.arrowedLine(
                frame,
                (center_x, center_y),
                (error_end_x, error_end_y),
                self._color_yellow, 2, tipLength=0.3
            )

        # Draw tracking state indicator
        state = target.tracking_state
        if state == "LOCKED":
            state_color = self._color_green
        elif state == "LOST":
            state_color = self._color_red
        else:
            state_color = self._color_yellow

        # State text
        state_text = f'Track: {state}'
        cv2.putText(frame, state_text, (10, img_h - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, state_color, 2, cv2.LINE_AA)

        # Error values
        error_text = f'Error: X={target.error_x:.3f} Y={target.error_y:.3f}'
        cv2.putText(frame, error_text, (10, img_h - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self._color_white, 1, cv2.LINE_AA)

        # Target info
        if target.target_visible:
            info_text = f'Target: {target.target_class} ({target.target_confidence:.2f})'
            cv2.putText(frame, info_text, (10, img_h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self._color_green, 1, cv2.LINE_AA)

        return frame

    def _draw_telemetry(self, frame: np.ndarray) -> np.ndarray:
        """
        Draw telemetry information on the frame.

        Args:
            frame: The input frame.

        Returns:
            Frame with telemetry overlay.
        """
        if self._latest_telemetry is None:
            cv2.putText(frame, 'Telemetry: No Data', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self._color_red, 1, cv2.LINE_AA)
            return frame

        tel = self._latest_telemetry
        x_offset = 10
        y_start = 20
        line_height = 20

        # Connection status
        conn_color = self._color_green if tel.connected else self._color_red
        cv2.putText(frame, f'PX4: {tel.connection_status}', (x_offset, y_start),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, conn_color, 1, cv2.LINE_AA)

        # Battery
        batt_color = self._color_green if tel.battery_remaining_percent > 30 else self._color_red
        cv2.putText(frame,
                    f'Batt: {tel.battery_remaining_percent:.0f}% ({tel.battery_voltage:.1f}V)',
                    (x_offset, y_start + line_height),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, batt_color, 1, cv2.LINE_AA)

        # GPS
        gps_color = self._color_green if tel.health_gps_ok else self._color_yellow
        cv2.putText(frame,
                    f'GPS: {tel.gps_num_satellites} sats (fix:{tel.gps_fix_type})',
                    (x_offset, y_start + line_height * 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, gps_color, 1, cv2.LINE_AA)

        # Altitude
        cv2.putText(frame,
                    f'Alt: {tel.relative_altitude:.1f}m (rel)',
                    (x_offset, y_start + line_height * 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self._color_white, 1, cv2.LINE_AA)

        # Armed state and mode
        armed_color = self._color_red if tel.armed else self._color_green
        armed_text = 'ARMED' if tel.armed else 'DISARMED'
        cv2.putText(frame,
                    f'{armed_text} | {tel.flight_mode}',
                    (x_offset, y_start + line_height * 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, armed_color, 1, cv2.LINE_AA)

        return frame

    def _draw_control(self, frame: np.ndarray) -> np.ndarray:
        """
        Draw control command information on the frame.

        Args:
            frame: The input frame.

        Returns:
            Frame with control overlay.
        """
        if self._latest_command is None:
            return frame

        cmd = self._latest_command
        img_h, img_w = frame.shape[:2]

        # Draw in top-right corner
        x_offset = img_w - 260
        y_start = 20
        line_height = 20

        # Command status
        if cmd.executed:
            status_color = self._color_red  # Red because commands are active!
            status_text = 'CMD: ACTIVE'
        else:
            status_color = self._color_green  # Green because safe
            status_text = f'CMD: {cmd.execution_status[:25]}'

        cv2.putText(frame, status_text, (x_offset, y_start),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1, cv2.LINE_AA)

        # Velocity commands
        cv2.putText(frame,
                    f'Fwd: {cmd.velocity_forward:+.2f} m/s',
                    (x_offset, y_start + line_height),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self._color_cyan, 1, cv2.LINE_AA)

        cv2.putText(frame,
                    f'Right: {cmd.velocity_right:+.2f} m/s',
                    (x_offset, y_start + line_height * 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self._color_cyan, 1, cv2.LINE_AA)

        cv2.putText(frame,
                    f'Down: {cmd.velocity_down:+.2f} m/s',
                    (x_offset, y_start + line_height * 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self._color_cyan, 1, cv2.LINE_AA)

        cv2.putText(frame,
                    f'Yaw: {cmd.yaw_rate:+.2f} rad/s',
                    (x_offset, y_start + line_height * 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self._color_cyan, 1, cv2.LINE_AA)

        # Draw a mini compass/movement indicator
        indicator_center_x = img_w - 50
        indicator_center_y = 150
        indicator_radius = 30

        cv2.circle(frame, (indicator_center_x, indicator_center_y),
                   indicator_radius, self._color_white, 1)

        # Draw velocity vector on indicator
        vec_x = int(cmd.velocity_right * indicator_radius / 2.0)
        vec_y = int(-cmd.velocity_forward * indicator_radius / 2.0)  # Negative because up is forward

        cv2.arrowedLine(
            frame,
            (indicator_center_x, indicator_center_y),
            (indicator_center_x + vec_x, indicator_center_y + vec_y),
            self._color_orange, 2, tipLength=0.4
        )

        return frame

    def _draw_fps(self, frame: np.ndarray) -> np.ndarray:
        """Draw FPS counter on the frame."""
        img_h, img_w = frame.shape[:2]
        fps_text = f'Display: {self._display_measured_fps:.0f} FPS'
        cv2.putText(frame, fps_text, (img_w - 160, img_h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self._color_white, 1, cv2.LINE_AA)
        return frame

    def _display_callback(self) -> None:
        """Main display callback - compose and show the visualization."""
        if self._latest_frame is None:
            # Show a blank frame with "waiting" message
            frame = np.zeros((self._display_height, self._display_width, 3), dtype=np.uint8)
            cv2.putText(frame, 'Waiting for camera feed...',
                        (self._display_width // 4, self._display_height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, self._color_white, 2, cv2.LINE_AA)
        else:
            frame = self._latest_frame.copy()

            # Apply overlays
            if self._show_detections:
                frame = self._draw_detections(frame)

            if self._show_tracking:
                frame = self._draw_tracking(frame)

            if self._show_telemetry:
                frame = self._draw_telemetry(frame)

            if self._show_control:
                frame = self._draw_control(frame)

        frame = self._draw_fps(frame)

        # FPS measurement
        self._frame_count += 1
        current_time = time.time()
        elapsed = current_time - self._last_fps_time
        if elapsed >= 1.0:
            self._display_measured_fps = self._frame_count / elapsed
            self._frame_count = 0
            self._last_fps_time = current_time

        # Display
        if not self._headless:
            cv2.imshow(self._window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed. Shutting down visualizer.')
                raise SystemExit()

    def destroy_node(self) -> None:
        """Clean up resources."""
        self.get_logger().info('Shutting down visualizer node...')
        if not self._headless:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the visualizer node."""
    rclpy.init(args=args)
    node = VisualizerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()