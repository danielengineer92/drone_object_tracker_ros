"""
Control command generation node.

Subscribes:
    /target_error
    /drone/telemetry

Publishes:
    /control_command

This node is intentionally conservative for early flight testing:
- horizontal image error drives yaw only
- strafe and forward velocity are held at zero
- commands are zeroed unless all safety gates pass
"""

import time
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import ControlCommand, DroneTelemetry, TargetError


CMD_IDLE = "IDLE"
CMD_VELOCITY = "VELOCITY"

STATUS_SENT = "SENT"
STATUS_NO_TARGET = "NO_TARGET_DATA"
STATUS_TARGET_STALE = "TARGET_DATA_STALE"
STATUS_TARGET_NOT_VISIBLE = "TARGET_NOT_VISIBLE"
STATUS_BLOCKED_DISABLED = "BLOCKED_AUTONOMOUS_DISABLED"
STATUS_BLOCKED_NO_TELEM = "BLOCKED_NO_TELEMETRY"
STATUS_BLOCKED_TELEM_STALE = "BLOCKED_TELEMETRY_STALE"
STATUS_BLOCKED_LOW_BATTERY = "BLOCKED_LOW_BATTERY"
STATUS_BLOCKED_GPS = "BLOCKED_GPS_NOT_HEALTHY"
STATUS_BLOCKED_DISARMED = "BLOCKED_NOT_ARMED"
STATUS_BLOCKED_DISCONNECTED = "BLOCKED_NOT_CONNECTED"
STATUS_ALTITUDE_CLAMPED = "ALTITUDE_FLOOR_CLAMPED"

DYNAMIC_PARAMS = {
    "autonomous_enabled",
    "gain_forward", "gain_right", "gain_down", "gain_yaw",
    "deadband_x", "deadband_y",
    "max_velocity_forward", "max_velocity_right", "max_velocity_down", "max_yaw_rate",
    "max_accel_forward", "max_accel_right", "max_accel_down", "max_yaw_accel",
    "min_battery_percent", "require_gps", "require_armed",
    "min_altitude_m", "target_timeout", "telemetry_timeout",
}


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        self.declare_parameter("autonomous_enabled", False)

        self.declare_parameter("gain_forward", 1.0)  # reserved for future distance/area control
        self.declare_parameter("gain_right", 1.0)    # reserved; strafe disabled for now
        self.declare_parameter("gain_down", 0.5)
        self.declare_parameter("gain_yaw", 0.8)

        self.declare_parameter("deadband_x", 0.05)
        self.declare_parameter("deadband_y", 0.05)

        self.declare_parameter("max_velocity_forward", 2.0)
        self.declare_parameter("max_velocity_right", 2.0)
        self.declare_parameter("max_velocity_down", 0.5)
        self.declare_parameter("max_yaw_rate", 1.0)

        self.declare_parameter("max_accel_forward", 1.5)
        self.declare_parameter("max_accel_right", 1.5)
        self.declare_parameter("max_accel_down", 0.5)
        self.declare_parameter("max_yaw_accel", 2.0)

        self.declare_parameter("min_battery_percent", 25.0)
        self.declare_parameter("require_gps", False)
        self.declare_parameter("require_armed", True)
        self.declare_parameter("min_altitude_m", 2.0)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("target_timeout", 1.0)
        self.declare_parameter("telemetry_timeout", 2.0)

        self.autonomous_enabled = bool(self.get_parameter("autonomous_enabled").value)

        self.gain_forward = float(self.get_parameter("gain_forward").value)
        self.gain_right = float(self.get_parameter("gain_right").value)
        self.gain_down = float(self.get_parameter("gain_down").value)
        self.gain_yaw = float(self.get_parameter("gain_yaw").value)

        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)

        self.max_velocity_forward = float(self.get_parameter("max_velocity_forward").value)
        self.max_velocity_right = float(self.get_parameter("max_velocity_right").value)
        self.max_velocity_down = float(self.get_parameter("max_velocity_down").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        self.max_accel_forward = float(self.get_parameter("max_accel_forward").value)
        self.max_accel_right = float(self.get_parameter("max_accel_right").value)
        self.max_accel_down = float(self.get_parameter("max_accel_down").value)
        self.max_yaw_accel = float(self.get_parameter("max_yaw_accel").value)

        self.min_battery_percent = float(self.get_parameter("min_battery_percent").value)
        self.require_gps = bool(self.get_parameter("require_gps").value)
        self.require_armed = bool(self.get_parameter("require_armed").value)
        self.min_altitude_m = float(self.get_parameter("min_altitude_m").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.target_timeout = float(self.get_parameter("target_timeout").value)
        self.telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)

        self.validate_parameters()
        self.control_period = 1.0 / self.control_rate

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
            "target_error",
            self.target_error_callback,
            qos,
        )

        self.telemetry_sub = self.create_subscription(
            DroneTelemetry,
            "drone/telemetry",
            self.telemetry_callback,
            qos,
        )

        self.command_pub = self.create_publisher(
            ControlCommand,
            "control_command",
            qos,
        )

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.status_timer = self.create_timer(5.0, self.report_status)
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().warning(
            f"Control node started | autonomous_enabled={self.autonomous_enabled}, "
            "mode=YAW_ONLY, forward=0, strafe=0"
        )

        if not self.autonomous_enabled:
            self.get_logger().warning(
                "AUTONOMOUS MODE DISABLED - output commands will be zeroed."
            )

    def validate_parameters(self) -> None:
        if self.control_rate <= 0.0:
            raise ValueError(f"control_rate must be > 0, got {self.control_rate}")
        if not 0.0 <= self.deadband_x < 1.0:
            raise ValueError(f"deadband_x must be in [0, 1), got {self.deadband_x}")
        if not 0.0 <= self.deadband_y < 1.0:
            raise ValueError(f"deadband_y must be in [0, 1), got {self.deadband_y}")
        if self.target_timeout <= 0.0:
            raise ValueError(f"target_timeout must be > 0, got {self.target_timeout}")
        if self.telemetry_timeout <= 0.0:
            raise ValueError(f"telemetry_timeout must be > 0, got {self.telemetry_timeout}")

        nonnegative = {
            "max_velocity_forward": self.max_velocity_forward,
            "max_velocity_right": self.max_velocity_right,
            "max_velocity_down": self.max_velocity_down,
            "max_yaw_rate": self.max_yaw_rate,
            "max_accel_forward": self.max_accel_forward,
            "max_accel_right": self.max_accel_right,
            "max_accel_down": self.max_accel_down,
            "max_yaw_accel": self.max_yaw_accel,
            "min_battery_percent": self.min_battery_percent,
            "min_altitude_m": self.min_altitude_m,
        }
        for name, value in nonnegative.items():
            if value < 0.0:
                raise ValueError(f"{name} must be >= 0, got {value}")

    def on_parameter_change(self, params) -> SetParametersResult:
        for param in params:
            if param.name not in DYNAMIC_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} is not runtime-reconfigurable",
                )

            if param.name in ("deadband_x", "deadband_y"):
                if not 0.0 <= float(param.value) < 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be in [0, 1)",
                    )

            if param.name.startswith(("max_", "min_")):
                if float(param.value) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be >= 0",
                    )

            if param.name in ("target_timeout", "telemetry_timeout"):
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be > 0",
                    )

        for param in params:
            old_value = getattr(self, param.name, None)
            setattr(self, param.name, param.value)

            if param.name == "autonomous_enabled":
                if bool(param.value) and not bool(old_value):
                    self.get_logger().warning("*** AUTONOMOUS MODE ENABLED ***")
                elif not bool(param.value) and bool(old_value):
                    self.get_logger().warning("Autonomous mode disabled.")

        return SetParametersResult(successful=True)

    def target_error_callback(self, msg: TargetError) -> None:
        self.last_target_error = msg
        self.last_target_error_time = time.time()

    def telemetry_callback(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.time()

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0.0

        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - deadband) / (1.0 - deadband)

    @staticmethod
    def rate_limit_value(new_value: float, old_value: float, max_change: float) -> float:
        change = new_value - old_value

        if change > max_change:
            change = max_change
        elif change < -max_change:
            change = -max_change

        return old_value + change

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def check_safety(self, current_time: float) -> tuple[bool, str]:
        if not self.autonomous_enabled:
            return False, STATUS_BLOCKED_DISABLED

        if self.last_telemetry is None:
            return False, STATUS_BLOCKED_NO_TELEM

        telemetry_age = current_time - self.last_telemetry_time
        if telemetry_age > self.telemetry_timeout:
            return False, f"{STATUS_BLOCKED_TELEM_STALE} ({telemetry_age:.2f}s)"

        telemetry = self.last_telemetry

        if not telemetry.connected:
            return False, STATUS_BLOCKED_DISCONNECTED

        if telemetry.battery_remaining_percent < self.min_battery_percent:
            return False, STATUS_BLOCKED_LOW_BATTERY

        if self.require_gps and not telemetry.health_gps_ok:
            return False, STATUS_BLOCKED_GPS

        if self.require_armed and not telemetry.armed:
            return False, STATUS_BLOCKED_DISARMED

        return True, STATUS_SENT

    def make_command(
        self,
        command_type: str,
        status: str,
        executed: bool,
        velocity_forward: float = 0.0,
        velocity_right: float = 0.0,
        velocity_down: float = 0.0,
        yaw_rate: float = 0.0,
        source_error_x: float = 0.0,
        source_error_y: float = 0.0,
    ) -> ControlCommand:
        command = ControlCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command_type = command_type

        command.velocity_forward = float(velocity_forward)
        command.velocity_right = float(velocity_right)
        command.velocity_down = float(velocity_down)
        command.yaw_rate = float(yaw_rate)

        command.executed = bool(executed)
        command.execution_status = status

        command.source_error_x = float(source_error_x)
        command.source_error_y = float(source_error_y)

        return command

    def publish_idle(self, status: str, source_error_x: float = 0.0, source_error_y: float = 0.0) -> None:
        self.last_command_forward = 0.0
        self.last_command_right = 0.0
        self.last_command_down = 0.0
        self.last_command_yaw = 0.0

        self.command_pub.publish(
            self.make_command(
                CMD_IDLE,
                status,
                executed=False,
                source_error_x=source_error_x,
                source_error_y=source_error_y,
            )
        )

    def control_loop(self) -> None:
        current_time = time.time()

        if self.last_target_error is None:
            self.publish_idle(STATUS_NO_TARGET)
            return

        target_age = current_time - self.last_target_error_time
        if target_age > self.target_timeout:
            self.publish_idle(f"{STATUS_TARGET_STALE} ({target_age:.2f}s)")
            return

        target = self.last_target_error

        if not target.target_visible:
            self.publish_idle(
                STATUS_TARGET_NOT_VISIBLE,
                source_error_x=target.error_x,
                source_error_y=target.error_y,
            )
            return

        error_x = self.apply_deadband(float(target.error_x), self.deadband_x)
        error_y = self.apply_deadband(float(target.error_y), self.deadband_y)

        # Conservative first-flight mode:
        # target horizontal offset -> yaw only, no strafe
        # target vertical offset -> slow up/down command
        # forward is held at 0 until distance/area control is intentionally added
        desired_forward = 0.0
        desired_right = 0.0
        desired_down = error_y * self.gain_down
        desired_yaw = error_x * self.gain_yaw

        limited_forward = self.rate_limit_value(
            desired_forward,
            self.last_command_forward,
            self.max_accel_forward * self.control_period,
        )
        limited_right = self.rate_limit_value(
            desired_right,
            self.last_command_right,
            self.max_accel_right * self.control_period,
        )
        limited_down = self.rate_limit_value(
            desired_down,
            self.last_command_down,
            self.max_accel_down * self.control_period,
        )
        limited_yaw = self.rate_limit_value(
            desired_yaw,
            self.last_command_yaw,
            self.max_yaw_accel * self.control_period,
        )

        limited_forward = self.clamp(limited_forward, -self.max_velocity_forward, self.max_velocity_forward)
        limited_right = self.clamp(limited_right, -self.max_velocity_right, self.max_velocity_right)
        limited_down = self.clamp(limited_down, -self.max_velocity_down, self.max_velocity_down)
        limited_yaw = self.clamp(limited_yaw, -self.max_yaw_rate, self.max_yaw_rate)

        altitude_clamped = False
        if self.last_telemetry is not None:
            altitude = float(self.last_telemetry.relative_altitude)
            if altitude < self.min_altitude_m and limited_down > 0.0:
                limited_down = 0.0
                altitude_clamped = True

        # Check safety before committing command state.
        # Important: when safety blocks output, the rate limiter must remember
        # the last *actual* output (zero), not the intended blocked command.
        # Otherwise enabling autonomous later can create a sudden jump.
        safe, reason = self.check_safety(current_time)
        if altitude_clamped and safe:
            reason = f"{STATUS_SENT}_{STATUS_ALTITUDE_CLAMPED}"

        output_forward = limited_forward if safe else 0.0
        output_right = limited_right if safe else 0.0
        output_down = limited_down if safe else 0.0
        output_yaw = limited_yaw if safe else 0.0

        self.last_command_forward = output_forward
        self.last_command_right = output_right
        self.last_command_down = output_down
        self.last_command_yaw = output_yaw

        if not safe:
            self.get_logger().info(
                f"blocked={reason} intended fwd={limited_forward:+.3f} "
                f"right={limited_right:+.3f} down={limited_down:+.3f} yaw={limited_yaw:+.3f}",
                throttle_duration_sec=1.0,
            )

        command = self.make_command(
            CMD_VELOCITY if safe else CMD_IDLE,
            reason,
            executed=safe,
            velocity_forward=output_forward,
            velocity_right=output_right,
            velocity_down=output_down,
            yaw_rate=output_yaw,
            source_error_x=target.error_x,
            source_error_y=target.error_y,
        )

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
    node = None
    try:
        node = ControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("control_node").fatal(f"Fatal: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
