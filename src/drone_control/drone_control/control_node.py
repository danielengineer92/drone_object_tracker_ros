"""
Control command generation node.

Subscribes:
    /target_error
    /drone/telemetry

Publishes:
    /control_command
"""

import time
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import ControlCommand, DroneTelemetry, TargetError


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        self.declare_parameter("autonomous_enabled", False)

        self.declare_parameter("gain_forward", 1.0)
        self.declare_parameter("gain_right", 1.0)
        self.declare_parameter("gain_down", 0.5)
        self.declare_parameter("gain_yaw", 0.8)

        self.declare_parameter("deadband_x", 0.05)
        self.declare_parameter("deadband_y", 0.05)

        self.declare_parameter("max_velocity_forward", 2.0)
        self.declare_parameter("max_velocity_right", 2.0)
        self.declare_parameter("max_velocity_down", 1.0)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("rate_limit", 0.5)

        self.declare_parameter("min_battery_percent", 20.0)
        self.declare_parameter("require_gps", False)
        self.declare_parameter("require_armed", True)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("target_timeout", 3.0)

        self.autonomous_enabled = self.get_parameter("autonomous_enabled").value

        self.gain_forward = self.get_parameter("gain_forward").value
        self.gain_right = self.get_parameter("gain_right").value
        self.gain_down = self.get_parameter("gain_down").value
        self.gain_yaw = self.get_parameter("gain_yaw").value

        self.deadband_x = self.get_parameter("deadband_x").value
        self.deadband_y = self.get_parameter("deadband_y").value

        self.max_velocity_forward = self.get_parameter("max_velocity_forward").value
        self.max_velocity_right = self.get_parameter("max_velocity_right").value
        self.max_velocity_down = self.get_parameter("max_velocity_down").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value
        self.rate_limit = self.get_parameter("rate_limit").value

        self.min_battery_percent = self.get_parameter("min_battery_percent").value
        self.require_gps = self.get_parameter("require_gps").value
        self.require_armed = self.get_parameter("require_armed").value
        self.control_rate = self.get_parameter("control_rate").value
        self.target_timeout = self.get_parameter("target_timeout").value

        self.last_target_error: Optional[TargetError] = None
        self.last_telemetry: Optional[DroneTelemetry] = None

        self.last_target_error_time = 0.0
        self.last_telemetry_time = 0.0

        self.last_command_forward = 0.0
        self.last_command_right = 0.0
        self.last_command_down = 0.0
        self.last_command_yaw = 0.0

        self.command_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.error_sub = self.create_subscription(
            TargetError,
            "/target_error",
            self.target_error_callback,
            qos,
        )

        self.telemetry_sub = self.create_subscription(
            DroneTelemetry,
            "/drone/telemetry",
            self.telemetry_callback,
            qos,
        )

        self.command_pub = self.create_publisher(
            ControlCommand,
            "/control_command",
            qos,
        )

        self.control_timer = self.create_timer(
            1.0 / float(self.control_rate),
            self.control_loop,
        )

        self.status_timer = self.create_timer(5.0, self.report_status)

        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().warning(
            f"Control node started | autonomous_enabled={self.autonomous_enabled}"
        )

        if not self.autonomous_enabled:
            self.get_logger().warning(
                "AUTONOMOUS MODE DISABLED - publishing simulated commands only."
            )

    def on_parameter_change(self, params) -> SetParametersResult:
        for param in params:
            if param.name == "autonomous_enabled":
                self.autonomous_enabled = bool(param.value)

            elif param.name == "gain_forward":
                self.gain_forward = float(param.value)

            elif param.name == "gain_right":
                self.gain_right = float(param.value)

            elif param.name == "gain_down":
                self.gain_down = float(param.value)

            elif param.name == "gain_yaw":
                self.gain_yaw = float(param.value)

            elif param.name == "deadband_x":
                self.deadband_x = float(param.value)

            elif param.name == "deadband_y":
                self.deadband_y = float(param.value)

        return SetParametersResult(successful=True)

    def target_error_callback(self, msg: TargetError) -> None:
        self.last_target_error = msg
        self.last_target_error_time = time.time()

    def telemetry_callback(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.time()

    def apply_deadband(self, value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0.0

        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - deadband) / (1.0 - deadband)

    def rate_limit_value(self, new_value: float, old_value: float, max_change: float) -> float:
        change = new_value - old_value

        if abs(change) > max_change:
            change = max_change if change > 0.0 else -max_change

        return old_value + change

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def check_safety(self) -> tuple[bool, str]:
        if not self.autonomous_enabled:
            return False, "BLOCKED_AUTONOMOUS_DISABLED"

        if self.last_telemetry is None:
            return False, "BLOCKED_NO_TELEMETRY"

        telemetry = self.last_telemetry

        if telemetry.battery_remaining_percent < self.min_battery_percent:
            return False, "BLOCKED_LOW_BATTERY"

        if self.require_gps and not telemetry.health_gps_ok:
            return False, "BLOCKED_GPS_NOT_HEALTHY"

        if self.require_armed and not telemetry.armed:
            return False, "BLOCKED_NOT_ARMED"

        if not telemetry.connected:
            return False, "BLOCKED_NOT_CONNECTED"

        return True, "PASSED"

    def make_idle_command(self, status: str) -> ControlCommand:
        command = ControlCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command_type = "IDLE"

        command.velocity_forward = 0.0
        command.velocity_right = 0.0
        command.velocity_down = 0.0
        command.yaw_rate = 0.0

        command.executed = False
        command.execution_status = status

        command.source_error_x = 0.0
        command.source_error_y = 0.0

        return command

    def control_loop(self) -> None:
        current_time = time.time()

        if self.last_target_error is None:
            self.command_pub.publish(self.make_idle_command("NO_TARGET_DATA"))
            return

        target_age = current_time - self.last_target_error_time

        if target_age > self.target_timeout:
            self.command_pub.publish(self.make_idle_command("TARGET_DATA_STALE"))
            return

        target = self.last_target_error

        if not target.target_visible:
            self.last_command_forward = 0.0
            self.last_command_right = 0.0
            self.last_command_down = 0.0
            self.last_command_yaw = 0.0

            command = self.make_idle_command("TARGET_NOT_VISIBLE")
            command.source_error_x = float(target.error_x)
            command.source_error_y = float(target.error_y)
            self.command_pub.publish(command)
            return

        error_x = self.apply_deadband(target.error_x, self.deadband_x)
        error_y = self.apply_deadband(target.error_y, self.deadband_y)

        desired_forward = 0.0
        desired_right = error_x * self.gain_right
        desired_down = error_y * self.gain_down
        desired_yaw = error_x * self.gain_yaw

        limited_forward = self.rate_limit_value(
            desired_forward,
            self.last_command_forward,
            self.rate_limit,
        )

        limited_right = self.rate_limit_value(
            desired_right,
            self.last_command_right,
            self.rate_limit,
        )

        limited_down = self.rate_limit_value(
            desired_down,
            self.last_command_down,
            self.rate_limit,
        )

        limited_yaw = self.rate_limit_value(
            desired_yaw,
            self.last_command_yaw,
            self.rate_limit,
        )

        limited_forward = self.clamp(
            limited_forward,
            -self.max_velocity_forward,
            self.max_velocity_forward,
        )

        limited_right = self.clamp(
            limited_right,
            -self.max_velocity_right,
            self.max_velocity_right,
        )

        limited_down = self.clamp(
            limited_down,
            -self.max_velocity_down,
            self.max_velocity_down,
        )

        limited_yaw = self.clamp(
            limited_yaw,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )

        self.last_command_forward = limited_forward
        self.last_command_right = limited_right
        self.last_command_down = limited_down
        self.last_command_yaw = limited_yaw

        safe, reason = self.check_safety()

        command = ControlCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command_type = "VELOCITY"

        command.velocity_forward = float(limited_forward)
        command.velocity_right = float(limited_right)
        command.velocity_down = float(limited_down)
        command.yaw_rate = float(limited_yaw)

        command.source_error_x = float(target.error_x)
        command.source_error_y = float(target.error_y)

        command.executed = bool(safe)
        command.execution_status = "SENT" if safe else reason

        self.command_pub.publish(command)
        self.command_count += 1

    def report_status(self) -> None:
        self.get_logger().info(
            f"Control status | autonomous={self.autonomous_enabled}, "
            f"commands={self.command_count}, "
            f"forward={self.last_command_forward:.3f}, "
            f"right={self.last_command_right:.3f}, "
            f"down={self.last_command_down:.3f}, "
            f"yaw={self.last_command_yaw:.3f}"
        )

    def destroy_node(self) -> None:
        self.get_logger().info("Control node shut down.")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
