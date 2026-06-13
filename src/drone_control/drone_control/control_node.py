"""
Control command generation node for the drone vision system.

This node subscribes to tracking error and telemetry, computes desired
movement commands using configurable PID-like gains, and enforces safety
constraints. When autonomous_enabled is False, commands are logged but
never sent to the aircraft.
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_interfaces.msg import TargetError, DroneTelemetry, ControlCommand


class ControlNode(Node):
    """
    ROS 2 node that generates movement commands from tracking error.

    This node implements proportional control with deadband, rate limiting,
    and comprehensive safety checks. The autonomous_enabled parameter gates
    all actual command execution.
    """

    def __init__(self) -> None:
        """Initialize the control node."""
        super().__init__('control_node')

        # CRITICAL SAFETY PARAMETER
        self.declare_parameter('autonomous_enabled', False)

        # Control gains
        self.declare_parameter('gain_forward', 1.0)
        self.declare_parameter('gain_right', 1.0)
        self.declare_parameter('gain_down', 0.5)
        self.declare_parameter('gain_yaw', 0.8)

        # Deadband (errors below this are ignored)
        self.declare_parameter('deadband_x', 0.05)
        self.declare_parameter('deadband_y', 0.05)

        # Rate limiting (max m/s or rad/s change per control cycle)
        self.declare_parameter('max_velocity_forward', 2.0)
        self.declare_parameter('max_velocity_right', 2.0)
        self.declare_parameter('max_velocity_down', 1.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('rate_limit', 0.5)

        # Safety parameters
        self.declare_parameter('min_battery_percent', 20.0)
        self.declare_parameter('require_gps', False)
        self.declare_parameter('require_armed', True)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('target_timeout', 3.0)

        # Read parameters
        self._autonomous_enabled: bool = self.get_parameter('autonomous_enabled').value
        self._gain_forward: float = self.get_parameter('gain_forward').value
        self._gain_right: float = self.get_parameter('gain_right').value
        self._gain_down: float = self.get_parameter('gain_down').value
        self._gain_yaw: float = self.get_parameter('gain_yaw').value
        self._deadband_x: float = self.get_parameter('deadband_x').value
        self._deadband_y: float = self.get_parameter('deadband_y').value
        self._max_velocity_forward: float = self.get_parameter('max_velocity_forward').value
        self._max_velocity_right: float = self.get_parameter('max_velocity_right').value
        self._max_velocity_down: float = self.get_parameter('max_velocity_down').value
        self._max_yaw_rate: float = self.get_parameter('max_yaw_rate').value
        self._rate_limit: float = self.get_parameter('rate_limit').value
        self._min_battery_percent: float = self.get_parameter('min_battery_percent').value
        self._require_gps: bool = self.get_parameter('require_gps').value
        self._require_armed: bool = self.get_parameter('require_armed').value
        self._control_rate: float = self.get_parameter('control_rate').value
        self._target_timeout: float = self.get_parameter('target_timeout').value

        # State
        self._last_target_error: Optional[TargetError] = None
        self._last_telemetry: Optional[DroneTelemetry] = None
        self._last_target_error_time: float = 0.0
        self._last_telemetry_time: float = 0.0
        self._last_command_forward: float = 0.0
        self._last_command_right: float = 0.0
        self._last_command_down: float = 0.0
        self._last_command_yaw: float = 0.0
        self._command_count: int = 0

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self._error_sub = self.create_subscription(
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

        # Publisher
        self._command_pub = self.create_publisher(
            ControlCommand,
            '/control_command',
            reliable_qos
        )

        # Control loop timer
        control_period = 1.0 / self._control_rate
        self._control_timer = self.create_timer(control_period, self._control_loop)

        # Status timer
        self._status_timer = self.create_timer(5.0, self._report_status)

        # Parameter change callback
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Log safety state
        self.get_logger().warning(
            f'Control node initialized: autonomous_enabled={self._autonomous_enabled}'
        )
        if not self._autonomous_enabled:
            self.get_logger().warning(
                'AUTONOMOUS MODE DISABLED - Commands will be logged only, '
                'no flight commands will be sent.'
            )

    def _on_parameter_change(self, params) -> rclpy.parameter.SetParametersResult:
        """Handle parameter changes at runtime."""
        for param in params:
            if param.name == 'autonomous_enabled':
                old_value = self._autonomous_enabled
                self._autonomous_enabled = param.value
                if param.value and not old_value:
                    self.get_logger().warning(
                        '*** AUTONOMOUS MODE ENABLED - Commands will be sent to aircraft ***'
                    )
                elif not param.value and old_value:
                    self.get_logger().warning(
                        'Autonomous mode DISABLED - Commands will be logged only.'
                    )
            elif param.name == 'gain_forward':
                self._gain_forward = param.value
            elif param.name == 'gain_right':
                self._gain_right = param.value
            elif param.name == 'gain_down':
                self._gain_down = param.value
            elif param.name == 'gain_yaw':
                self._gain_yaw = param.value
            elif param.name == 'deadband_x':
                self._deadband_x = param.value
            elif param.name == 'deadband_y':
                self._deadband_y = param.value

        return rclpy.parameter.SetParametersResult(successful=True)

    def _target_error_callback(self, msg: TargetError) -> None:
        """Store the latest target error."""
        self._last_target_error = msg
        self._last_target_error_time = time.time()

    def _telemetry_callback(self, msg: DroneTelemetry) -> None:
        """Store the latest telemetry data."""
        self._last_telemetry = msg
        self._last_telemetry_time = time.time()

    def _apply_deadband(self, value: float, deadband: float) -> float:
        """
        Apply deadband to a value.

        Args:
            value: The input value.
            deadband: The deadband threshold.

        Returns:
            0.0 if abs(value) < deadband, otherwise value with deadband subtracted.
        """
        if abs(value) < deadband:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadband) / (1.0 - deadband)

    def _rate_limit_value(self, new_value: float, old_value: float, max_change: float) -> float:
        """
        Apply rate limiting to a command value.

        Args:
            new_value: The desired new value.
            old_value: The previous value.
            max_change: Maximum allowed change per cycle.

        Returns:
            The rate-limited value.
        """
        change = new_value - old_value
        if abs(change) > max_change:
            change = max_change if change > 0 else -max_change
        return old_value + change

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp a value to a range."""
        return max(min_val, min(value, max_val))

    def _check_safety(self) -> tuple[bool, str]:
        """
        Perform safety checks before allowing command execution.

        Returns:
            Tuple of (safe_to_execute, reason_if_not_safe).
        """
        if not self._autonomous_enabled:
            return False, "BLOCKED_AUTONOMOUS_DISABLED"

        if self._last_telemetry is None:
            return False, "BLOCKED_SAFETY: No telemetry data"

        telemetry = self._last_telemetry

        # Battery check
        if telemetry.battery_remaining_percent < self._min_battery_percent:
            return False, f"BLOCKED_SAFETY: Battery low ({telemetry.battery_remaining_percent:.1f}%)"

        # GPS check
        if self._require_gps and not telemetry.health_gps_ok:
            return False, "BLOCKED_SAFETY: GPS not healthy"

        # Armed check
        if self._require_armed and not telemetry.armed:
            return False, "BLOCKED_SAFETY: Drone not armed"

        # Connection check
        if not telemetry.connected:
            return False, "BLOCKED_SAFETY: Drone not connected"

        return True, "PASSED"

    def _control_loop(self) -> None:
        """Main control loop - compute and publish commands."""
        current_time = time.time()

        command = ControlCommand()
        command.stamp = self.get_clock().now().to_msg()

        # Check if we have recent target error data
        if self._last_target_error is None:
            command.command_type = "IDLE"
            command.velocity_forward = 0.0
            command.velocity_right = 0.0
            command.velocity_down = 0.0
            command.yaw_rate = 0.0
            command.executed = False
            command.execution_status = "NO_TARGET_DATA"
            command.source_error_x = 0.0
            command.source_error_y = 0.0
            self._command_pub.publish(command)
            return

        # Check target data freshness
        target_age = current_time - self._last_target_error_time
        if target_age > self._target_timeout:
            command.command_type = "IDLE"
            command.velocity_forward = 0.0
            command.velocity_right = 0.0
            command.velocity_down = 0.0
            command.yaw_rate = 0.0
            command.executed = False
            command.execution_status = "TARGET_DATA_STALE"
            command.source_error_x = 0.0
            command.source_error_y = 0.0
            self._command_pub.publish(command)
            return

        target = self._last_target_error

        # If target not visible, send idle
        if not target.target_visible:
            command.command_type = "IDLE"
            command.velocity_forward = 0.0
            command.velocity_right = 0.0
            command.velocity_down = 0.0
            command.yaw_rate = 0.0
            command.executed = False
            command.execution_status = "TARGET_NOT_VISIBLE"
            command.source_error_x = target.error_x
            command.source_error_y = target.error_y
            self._last_command_forward = 0.0
            self._last_command_right = 0.0
            self._last_command_down = 0.0
            self._last_command_yaw = 0.0
            self._command_pub.publish(command)
            return

        # Compute desired velocity commands from tracking error
        # error_x: positive = target is right of center -> yaw right
        # error_y: positive = target is below center -> pitch down (move forward & down)
        error_x = self._apply_deadband(target.error_x, self._deadband_x)
        error_y = self._apply_deadband(target.error_y, self._deadband_y)

        # Map errors to velocity commands
        # X error -> yaw rate (turn to face target)
        desired_yaw = error_x * self._gain_yaw

        # Y error -> forward/down velocity (approach target vertically)
        desired_down = error_y * self._gain_down

        # For forward velocity, use target area as a proxy for distance
        # Small area = far away -> move forward
        # This is a simplified approach
        desired_forward = 0.0  # No forward movement based on error alone for safety
        desired_right = error_x * self._gain_right

        # Apply rate limiting
        limited_forward = self._rate_limit_value(
            desired_forward, self._last_command_forward, self._rate_limit
        )
        limited_right = self._rate_limit_value(
            desired_right, self._last_command_right, self._rate_limit
        )
        limited_down = self._rate_limit_value(
            desired_down, self._last_command_down, self._rate_limit
        )
        limited_yaw = self._rate_limit_value(
            desired_yaw, self._last_command_yaw, self._rate_limit
        )

        # Apply velocity limits
        limited_forward = self._clamp(
            limited_forward, -self._max_velocity_forward, self._max_velocity_forward
        )
        limited_right = self._clamp(
            limited_right, -self._max_velocity_right, self._max_velocity_right
        )
        limited_down = self._clamp(
            limited_down, -self._max_velocity_down, self._max_velocity_down
        )
        limited_yaw = self._clamp(
            limited_yaw, -self._max_yaw_rate, self._max_yaw_rate
        )

        # Store for next iteration's rate limiting
        self._last_command_forward = limited_forward
        self._last_command_right = limited_right
        self._last_command_down = limited_down
        self._last_command_yaw = limited_yaw

        # Build command message
        command.command_type = "VELOCITY"
        command.velocity_forward = float(limited_forward)
        command.velocity_right = float(limited_right)
        command.velocity_down = float(limited_down)
        command.yaw_rate = float(limited_yaw)
        command.source_error_x = float(target.error_x)
        command.source_error_y = float(target.error_y)

        # Safety check
        safe, reason = self._check_safety()

        if safe:
            command.executed = True
            command.execution_status = "SENT"
            # In a real system, this is where MAVSDK offboard commands would be sent
            self.get_logger().debug(
                f'EXECUTING command: fwd={limited_forward:.3f}, '
                f'right={limited_right:.3f}, down={limited_down:.3f}, '
                f'yaw={limited_yaw:.3f}'
            )
        else:
            command.executed = False
            if not self._autonomous_enabled:
                command.execution_status = "BLOCKED_AUTONOMOUS_DISABLED"
            else:
                command.execution_status = reason

            # Log intended command
            self.get_logger().info(
                f'[SIMULATED] Command: fwd={limited_forward:.3f}, '
                f'right={limited_right:.3f}, down={limited_down:.3f}, '
                f'yaw={limited_yaw:.3f} | '
                f'Status: {command.execution_status}'
            )

        self._command_pub.publish(command)
        self._command_count += 1

    def _report_status(self) -> None:
        """Report control node status."""
        self.get_logger().info(
            f'Control status: autonomous={self._autonomous_enabled}, '
            f'commands_generated={self._command_count}, '
            f'last_fwd={self._last_command_forward:.3f}, '
            f'last_right={self._last_command_right:.3f}, '
            f'last_yaw={self._last_command_yaw:.3f}'
        )

    def destroy_node(self) -> None:
        """Clean up resources."""
        self.get_logger().info('Shutting down control node...')
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the control node."""
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()