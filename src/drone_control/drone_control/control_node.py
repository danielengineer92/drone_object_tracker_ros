"""
Control command generation node.

Subscribes:
    /target_error
    /drone/telemetry
    /autonomy_enable

Publishes:
    /control_command

This node is intentionally conservative for early flight testing:
- target tracking is gated by /autonomy_enable
- horizontal image error drives yaw only
- forward/back, strafe, altitude, and orbit commands are held at zero
- commands are zeroed unless all safety gates pass
"""

import time
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from drone_interfaces.msg import ControlCommand, DroneTelemetry, TargetError
from drone_diagnostics.node_diagnostics import NodeDiagnostics


CMD_IDLE = "IDLE"
CMD_VELOCITY = "VELOCITY"
CMD_YAW_RATE_ONLY_TEST = "YAW_RATE_ONLY_TEST"

CONTROL_MODE_VELOCITY = "VELOCITY"
CONTROL_MODE_YAW_RATE_ONLY_TEST = "YAW_RATE_ONLY_TEST"

STATUS_SENT = "SENT"
STATUS_NO_TARGET = "NO_TARGET_DATA"
STATUS_TARGET_STALE = "TARGET_DATA_STALE"
STATUS_TARGET_NOT_VISIBLE = "TARGET_NOT_VISIBLE"
STATUS_BLOCKED_DISABLED = "BLOCKED_AUTONOMY_DISABLED"
STATUS_BLOCKED_NO_TELEM = "BLOCKED_NO_TELEMETRY"
STATUS_BLOCKED_TELEM_STALE = "BLOCKED_TELEMETRY_STALE"
STATUS_BLOCKED_LOW_BATTERY = "BLOCKED_LOW_BATTERY"
STATUS_BLOCKED_GPS = "BLOCKED_GPS_NOT_HEALTHY"
STATUS_BLOCKED_DISARMED = "BLOCKED_NOT_ARMED"
STATUS_BLOCKED_DISCONNECTED = "BLOCKED_NOT_CONNECTED"
STATUS_ALTITUDE_CLAMPED = "ALTITUDE_FLOOR_CLAMPED"

DYNAMIC_PARAMS = {
    "autonomy_enabled", "autonomous_enabled", "control_output_mode",
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

        # New preferred parameter name is autonomy_enabled.
        # Keep autonomous_enabled as a backward-compatible alias for older launch/config files.
        self.declare_parameter("autonomy_enabled", False)
        self.declare_parameter("autonomous_enabled", False)
        # VELOCITY keeps the old MAVSDK VelocityBodyYawspeed path.
        # YAW_RATE_ONLY_TEST tells the bridge to use MAVSDK AttitudeRate instead,
        # which avoids pulling PX4 into X/Y velocity control for bench testing.
        self.declare_parameter("control_output_mode", CONTROL_MODE_YAW_RATE_ONLY_TEST)

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
        self.declare_parameter("target_error_topic", "/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("control_command_topic", "/control_command")
        self.declare_parameter("autonomy_enable_topic", "/autonomy_enable")

        autonomy_param = bool(self.get_parameter("autonomy_enabled").value)
        legacy_autonomous_param = bool(self.get_parameter("autonomous_enabled").value)
        self.autonomy_enabled = autonomy_param or legacy_autonomous_param
        self.autonomous_enabled = self.autonomy_enabled  # compatibility for existing status/log tooling
        self.control_output_mode = str(self.get_parameter("control_output_mode").value).upper()

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
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self.control_command_topic = str(self.get_parameter("control_command_topic").value)
        self.autonomy_enable_topic = str(self.get_parameter("autonomy_enable_topic").value)

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
        self.idle_command_count = 0
        self.executed_command_count = 0
        self.target_error_count = 0
        self.telemetry_count = 0
        self.autonomy_enable_count = 0
        self.target_locked = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.error_sub = self.create_subscription(
            TargetError,
            self.target_error_topic,
            self.target_error_callback,
            qos,
        )

        self.telemetry_sub = self.create_subscription(
            DroneTelemetry,
            self.telemetry_topic,
            self.telemetry_callback,
            qos,
        )

        self.autonomy_sub = self.create_subscription(
            Bool,
            self.autonomy_enable_topic,
            self.autonomy_enable_callback,
            qos,
        )

        self.command_pub = self.create_publisher(
            ControlCommand,
            self.control_command_topic,
            qos,
        )

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.status_timer = self.create_timer(5.0, self.report_status)

        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.target_error_topic, "target_error", stale_seconds=self.target_timeout)
        self.diagnostics.add_input(self.telemetry_topic, "telemetry", stale_seconds=self.telemetry_timeout)
        self.diagnostics.add_output(self.control_command_topic, "control_command")

        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().warning(
            f"Control node started | target_error_topic={self.target_error_topic}, "
            f"telemetry_topic={self.telemetry_topic}, "
            f"control_command_topic={self.control_command_topic}, "
            f"autonomy_enable_topic={self.autonomy_enable_topic}, "
            f"autonomy_enabled={self.autonomy_enabled}, "
            f"control_output_mode={self.control_output_mode}, "
            "mode=YAW_ONLY, forward=0, right=0, down=0"
        )

        if not self.autonomy_enabled:
            self.get_logger().warning(
                "AUTONOMY DISABLED - publishing IDLE/zero commands until /autonomy_enable is true."
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
        if self.control_output_mode not in (CONTROL_MODE_VELOCITY, CONTROL_MODE_YAW_RATE_ONLY_TEST):
            raise ValueError(
                "control_output_mode must be VELOCITY or YAW_RATE_ONLY_TEST, "
                f"got {self.control_output_mode}"
            )

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

            if param.name == "control_output_mode":
                mode = str(param.value).upper()
                if mode not in (CONTROL_MODE_VELOCITY, CONTROL_MODE_YAW_RATE_ONLY_TEST):
                    return SetParametersResult(
                        successful=False,
                        reason="control_output_mode must be VELOCITY or YAW_RATE_ONLY_TEST",
                    )

        for param in params:
            old_value = getattr(self, param.name, None)
            value = param.value
            if param.name == "control_output_mode":
                value = str(value).upper()
            setattr(self, param.name, value)

            if param.name in ("autonomy_enabled", "autonomous_enabled"):
                self.set_autonomy_enabled(bool(param.value), source=f"parameter:{param.name}")

        return SetParametersResult(successful=True)

    def set_autonomy_enabled(self, enabled: bool, source: str) -> None:
        enabled = bool(enabled)
        old_enabled = self.autonomy_enabled
        self.autonomy_enabled = enabled
        self.autonomous_enabled = enabled  # compatibility alias

        if enabled and not old_enabled:
            self.get_logger().warning(f"*** AUTONOMY ENABLED by {source} ***")
        elif not enabled and old_enabled:
            self.get_logger().warning(f"Autonomy disabled by {source}; publishing IDLE/zero commands.")

        if not enabled:
            self.last_command_forward = 0.0
            self.last_command_right = 0.0
            self.last_command_down = 0.0
            self.last_command_yaw = 0.0

            # Push a zero command immediately on disable instead of waiting for
            # the next control timer tick. During __init__, command_pub does not
            # exist yet, so guard this for startup safety.
            if hasattr(self, "command_pub"):
                source_error_x = 0.0
                source_error_y = 0.0
                if self.last_target_error is not None:
                    source_error_x = float(self.last_target_error.error_x)
                    source_error_y = float(self.last_target_error.error_y)
                self.publish_idle(
                    STATUS_BLOCKED_DISABLED,
                    source_error_x=source_error_x,
                    source_error_y=source_error_y,
                )

    def autonomy_enable_callback(self, msg: Bool) -> None:
        self.autonomy_enable_count += 1
        self.set_autonomy_enabled(bool(msg.data), source=self.autonomy_enable_topic)
        self.diagnostics.mark_received(
            self.autonomy_enable_topic,
            summary=f"messages={self.autonomy_enable_count}, enabled={self.autonomy_enabled}",
        )

    def target_error_callback(self, msg: TargetError) -> None:
        self.last_target_error = msg
        self.last_target_error_time = time.time()
        self.target_error_count += 1

        is_locked = bool(msg.target_visible and msg.tracking_state == "LOCKED")
        if is_locked and not self.target_locked:
            self.get_logger().info(
                f"Target acquired by control node | class={msg.target_class}, "
                f"confidence={msg.target_confidence:.2f}, error_x={msg.error_x:+.3f}, error_y={msg.error_y:+.3f}"
            )
        elif not is_locked and self.target_locked:
            self.get_logger().warning(
                f"Target lost by control node | state={msg.tracking_state}, visible={msg.target_visible}"
            )
        self.target_locked = is_locked

        self.diagnostics.mark_received(
            self.target_error_topic,
            summary=f"messages={self.target_error_count}, state={msg.tracking_state}, visible={msg.target_visible}",
        )

    def telemetry_callback(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.time()
        self.telemetry_count += 1
        self.diagnostics.mark_received(
            self.telemetry_topic,
            summary=f"messages={self.telemetry_count}, connected={msg.connected}, battery={msg.battery_remaining_percent:.1f}%",
        )

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


    def publish_idle(
        self,
        status: str,
        source_error_x: float = 0.0,
        source_error_y: float = 0.0,
        desired_yaw: float | None = None,
    ) -> None:
      # IDLE must always mean no real movement command is being sent.
      # desired_yaw is debug-only so we can see what yaw WOULD have been commanded
      # if autonomy/safety gates allowed movement.
        self.last_command_forward = 0.0
        self.last_command_right = 0.0
        self.last_command_down = 0.0
        self.last_command_yaw = 0.0

        debug_status = status
        if desired_yaw is not None:
            debug_status = f"{status} | desired_yaw={desired_yaw:.3f}"

        command = self.make_command(
            CMD_IDLE,
            debug_status,
            executed=False,
            source_error_x=source_error_x,
            source_error_y=source_error_y,
        )

        # Extra safety: make sure idle command cannot accidentally carry motion.
        command.velocity_forward = 0.0
        command.velocity_right = 0.0
        command.velocity_down = 0.0
        command.yaw_rate = 0.0

        self.command_pub.publish(command)
        self.command_count += 1
        self.idle_command_count += 1

        self.diagnostics.mark_published(
            self.control_command_topic,
            summary=(
                f"commands={self.command_count}, "
                f"idle={self.idle_command_count}, "
                f"status={debug_status}"
            ),
        )

    def control_loop(self) -> None:
        current_time = time.time()
        desired_yaw = None

        if self.last_target_error is None:
            self.publish_idle(STATUS_NO_TARGET)
            return

        target_age = current_time - self.last_target_error_time
        if target_age > self.target_timeout:
            if self.target_locked:
                self.get_logger().warning(
                    f"Target lost by control node | target_error stale for {target_age:.2f}s"
                )
                self.target_locked = False

            self.publish_idle(
                f"{STATUS_TARGET_STALE} ({target_age:.2f}s)",
                source_error_x=float(self.last_target_error.error_x),
                source_error_y=float(self.last_target_error.error_y),
            )
            return

        target = self.last_target_error

        # Calculate debug yaw BEFORE autonomy/safety blocks.
        # This lets /control_command show what yaw WOULD be commanded,
        # while actual yaw_rate still stays zero when blocked.
        error_x = self.apply_deadband(float(target.error_x), self.deadband_x)
        desired_yaw = error_x * self.gain_yaw

        if not self.autonomy_enabled:
            self.publish_idle(
                STATUS_BLOCKED_DISABLED,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                desired_yaw=desired_yaw,
            )
            return

        if not (target.target_visible and target.tracking_state == "LOCKED"):
            self.publish_idle(
                STATUS_TARGET_NOT_VISIBLE,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                desired_yaw=desired_yaw,
            )
            return

        # Current safe autonomy mode is yaw-only:
        #   error_x < 0 => yaw negative
        #   error_x > 0 => yaw positive
        # No forward/back, right/left, altitude, or orbit commands are generated here.
        desired_yaw = self.clamp(
            desired_yaw,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )

        desired_yaw = self.rate_limit_value(
            desired_yaw,
            self.last_command_yaw,
            self.max_yaw_accel * self.control_period,
        )

        safe, reason = self.check_safety(current_time)
        if not safe:
            self.publish_idle(
                reason,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                desired_yaw=desired_yaw,
            )
            return

        # Tell the bridge which MAVSDK setpoint family to use.
        # YAW_RATE_ONLY_TEST is the indoor/bench path: yaw-rate only, no X/Y velocity setpoint.
        # VELOCITY keeps the older VelocityBodyYawspeed path for later outdoor/GPS testing.
        command_type = (
            CMD_YAW_RATE_ONLY_TEST
            if self.control_output_mode == CONTROL_MODE_YAW_RATE_ONLY_TEST
            else CMD_VELOCITY
        )

        command = self.make_command(
            command_type,
            STATUS_SENT,
            executed=True,
            velocity_forward=0.0,
            velocity_right=0.0,
            velocity_down=0.0,
            yaw_rate=desired_yaw,
            source_error_x=float(target.error_x),
            source_error_y=float(target.error_y),
        )

        self.last_command_forward = 0.0
        self.last_command_right = 0.0
        self.last_command_down = 0.0
        self.last_command_yaw = desired_yaw

        self.command_pub.publish(command)
        self.command_count += 1
        self.executed_command_count += 1

        self.diagnostics.mark_published(
            self.control_command_topic,
            summary=(
                f"commands={self.command_count}, "
                f"executed={self.executed_command_count}, "
                f"type={command_type}, "
                f"yaw={desired_yaw:.3f}, "
                f"status={STATUS_SENT}"
            ),
        )

    def report_status(self) -> None:
        self.get_logger().info(
            f"Control status | autonomy={self.autonomy_enabled}, target_locked={self.target_locked}, "
            f"commands={self.command_count}, executed={self.executed_command_count}, idle={self.idle_command_count}, "
            f"target_msgs={self.target_error_count}, telemetry_msgs={self.telemetry_count}, "
            f"autonomy_msgs={self.autonomy_enable_count}, "
            f"output_mode={self.control_output_mode}, "
            f"target_age={self.diagnostics.format_age(self.target_error_topic)}, "
            f"telemetry_age={self.diagnostics.format_age(self.telemetry_topic)}, "
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
