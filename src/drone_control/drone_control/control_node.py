"""
Control command generation node.

Subscribes:
    /drone/tracking/target_error
    /drone/telemetry
    /drone/autonomy/enabled
    /drone/mission/command

Publishes:
    /drone/control/command

This node is intentionally conservative for early flight testing:
- target tracking is gated by /drone/autonomy/enabled
- horizontal image error drives yaw in TRACK_CENTER
- mission commands can request FLY_FORWARD, APPROACH_TARGET, or ORBIT_TARGET
- translation is still blocked downstream unless the MAVSDK bridge explicitly allows it
- commands are zeroed unless all safety gates pass
"""

import time
from typing import Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from drone_interfaces.msg import ControlCommand, DroneTelemetry, MissionCommand, TargetError
from drone_diagnostics.node_diagnostics import NodeDiagnostics


CMD_IDLE = "IDLE"
CMD_VELOCITY = "VELOCITY"

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
STATUS_MISSION_STALE = "MISSION_COMMAND_STALE"

DYNAMIC_PARAMS = {
    "autonomy_enabled", "autonomous_enabled",
    "gain_forward", "gain_right", "gain_down", "gain_yaw",
    "deadband_x", "deadband_y",
    "max_velocity_forward", "max_velocity_right", "max_velocity_down", "max_yaw_rate",
    "max_accel_forward", "max_accel_right", "max_accel_down", "max_yaw_accel",
    "min_battery_percent", "require_gps", "require_armed",
    "min_altitude_m", "target_timeout", "telemetry_timeout",
    "mission_command_timeout", "desired_distance_m", "distance_gain_forward", "target_area_goal",
    "orbit_speed_m_s",
}


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        # New preferred parameter name is autonomy_enabled.
        # Keep autonomous_enabled as a backward-compatible alias for older launch/config files.
        self.declare_parameter("autonomy_enabled", False)
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
        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("control_command_topic", "/drone/control/command")
        self.declare_parameter("autonomy_enable_topic", "/drone/autonomy/enabled")
        self.declare_parameter("mission_command_topic", "/drone/mission/command")
        self.declare_parameter("mission_command_timeout", 1.0)
        self.declare_parameter("desired_distance_m", 2.0)
        self.declare_parameter("distance_gain_forward", 0.6)
        self.declare_parameter("target_area_goal", 0.08)
        self.declare_parameter("orbit_speed_m_s", 0.30)

        autonomy_param = bool(self.get_parameter("autonomy_enabled").value)
        legacy_autonomous_param = bool(self.get_parameter("autonomous_enabled").value)
        self.autonomy_enabled = autonomy_param or legacy_autonomous_param
        self.autonomous_enabled = self.autonomy_enabled  # compatibility for existing status/log tooling

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
        self.mission_command_topic = str(self.get_parameter("mission_command_topic").value)
        self.mission_command_timeout = float(self.get_parameter("mission_command_timeout").value)
        self.desired_distance_m = float(self.get_parameter("desired_distance_m").value)
        self.distance_gain_forward = float(self.get_parameter("distance_gain_forward").value)
        self.target_area_goal = float(self.get_parameter("target_area_goal").value)
        self.orbit_speed_m_s = float(self.get_parameter("orbit_speed_m_s").value)

        self.validate_parameters()
        self.control_period = 1.0 / self.control_rate

        self.last_target_error: Optional[TargetError] = None
        self.last_telemetry: Optional[DroneTelemetry] = None
        self.last_mission_command: Optional[MissionCommand] = None

        self.last_target_error_time = 0.0
        self.last_telemetry_time = 0.0
        self.last_mission_command_time = 0.0

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
        self.mission_command_count = 0
        self.last_mission_mode = "TRACK_CENTER"
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

        self.mission_sub = self.create_subscription(
            MissionCommand,
            self.mission_command_topic,
            self.mission_command_callback,
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
        self.diagnostics.add_input(self.mission_command_topic, "mission_command", stale_seconds=self.mission_command_timeout)
        self.diagnostics.add_output(self.control_command_topic, "control_command")

        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().warning(
            f"Control node started | target_error_topic={self.target_error_topic}, "
            f"telemetry_topic={self.telemetry_topic}, "
            f"control_command_topic={self.control_command_topic}, "
            f"autonomy_enable_topic={self.autonomy_enable_topic}, "
            f"mission_command_topic={self.mission_command_topic}, "
            f"autonomy_enabled={self.autonomy_enabled}, "
            "mode=MISSION_AWARE, default=TRACK_CENTER/yaw-only"
        )

        if not self.autonomy_enabled:
            self.get_logger().warning(
                "AUTONOMY DISABLED - publishing IDLE/zero commands until /drone/autonomy/enabled is true."
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
        if self.mission_command_timeout <= 0.0:
            raise ValueError(f"mission_command_timeout must be > 0, got {self.mission_command_timeout}")

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

            if param.name in ("target_timeout", "telemetry_timeout", "mission_command_timeout"):
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be > 0",
                    )

        for param in params:
            old_value = getattr(self, param.name, None)
            setattr(self, param.name, param.value)

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

    def mission_command_callback(self, msg: MissionCommand) -> None:
        self.last_mission_command = msg
        self.last_mission_command_time = time.time()
        self.mission_command_count += 1
        self.last_mission_mode = str(msg.mode)
        self.diagnostics.mark_received(
            self.mission_command_topic,
            summary=(
                f"messages={self.mission_command_count}, active={msg.active}, "
                f"mode={msg.mode}, step={msg.step_index}:{msg.step_name}, status={msg.status}"
            ),
        )

    def target_error_callback(self, msg: TargetError) -> None:
        self.last_target_error = msg
        self.last_target_error_time = time.time()
        self.target_error_count += 1

        is_locked = bool(msg.target_visible and msg.tracking_state == "LOCKED")
        if is_locked and not self.target_locked:
            self.get_logger().info(
                f"Target acquired by control node | class={msg.target_class}, "
                f"confidence={msg.target_confidence:.2f}, error_x={msg.error_x:+.3f}, error_y={msg.error_y:+.3f}, "
                f"distance_valid={getattr(msg, 'distance_valid', False)}, distance_m={getattr(msg, 'distance_m', 0.0):.2f}"
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


    def get_active_mission_command(self, current_time: float) -> Optional[MissionCommand]:
        if self.last_mission_command is None:
            return None
        mission_age = current_time - self.last_mission_command_time
        if mission_age > self.mission_command_timeout:
            return None
        if not self.last_mission_command.active:
            return None
        return self.last_mission_command

    def get_distance_forward_correction(self, target: TargetError, desired_distance_m: float) -> float:
        if bool(getattr(target, "distance_valid", False)) and float(getattr(target, "distance_m", 0.0)) > 0.0:
            distance_error_m = float(target.distance_m) - float(desired_distance_m)
            return self.distance_gain_forward * distance_error_m

        # Fallback for old TargetError messages or before distance calibration:
        # if target area is smaller than goal, move forward; if larger, back up.
        area_error = float(self.target_area_goal) - float(target.target_area)
        return self.gain_forward * area_error

    def limit_motion(self, forward: float, right: float, down: float, yaw: float) -> tuple[float, float, float, float]:
        forward = self.clamp(forward, -self.max_velocity_forward, self.max_velocity_forward)
        right = self.clamp(right, -self.max_velocity_right, self.max_velocity_right)
        down = self.clamp(down, -self.max_velocity_down, self.max_velocity_down)
        yaw = self.clamp(yaw, -self.max_yaw_rate, self.max_yaw_rate)

        forward = self.rate_limit_value(forward, self.last_command_forward, self.max_accel_forward * self.control_period)
        right = self.rate_limit_value(right, self.last_command_right, self.max_accel_right * self.control_period)
        down = self.rate_limit_value(down, self.last_command_down, self.max_accel_down * self.control_period)
        yaw = self.rate_limit_value(yaw, self.last_command_yaw, self.max_yaw_accel * self.control_period)
        return forward, right, down, yaw

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
        desired_yaw = 0.0
        mission = self.get_active_mission_command(current_time)
        mission_mode = "TRACK_CENTER" if mission is None else str(mission.mode).strip().upper()

        if not self.autonomy_enabled:
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
            return

        # Scripted non-vision mode for mission step 2. Still gated by telemetry/arming.
        if mission_mode == "FLY_FORWARD" and mission is not None:
            safe, reason = self.check_safety(current_time)
            forward, right, down, yaw = self.limit_motion(
                float(mission.velocity_forward),
                float(mission.velocity_right),
                float(mission.velocity_down),
                float(mission.yaw_rate),
            )
            if not safe:
                self.publish_idle(reason, desired_yaw=yaw)
                return

            command = self.make_command(
                CMD_VELOCITY,
                STATUS_SENT,
                executed=True,
                velocity_forward=forward,
                velocity_right=right,
                velocity_down=down,
                yaw_rate=yaw,
            )
            self.last_command_forward = forward
            self.last_command_right = right
            self.last_command_down = down
            self.last_command_yaw = yaw
            self.command_pub.publish(command)
            self.command_count += 1
            self.executed_command_count += 1
            self.diagnostics.mark_published(
                self.control_command_topic,
                summary=f"commands={self.command_count}, mode={mission_mode}, forward={forward:.3f}, status={STATUS_SENT}",
            )
            return

        if mission_mode in ("IDLE", "HOLD"):
            self.publish_idle(f"MISSION_{mission_mode}")
            return

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
        error_x = self.apply_deadband(float(target.error_x), self.deadband_x)
        desired_yaw = error_x * self.gain_yaw

        if not (target.target_visible and target.tracking_state == "LOCKED"):
            self.publish_idle(
                STATUS_TARGET_NOT_VISIBLE,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                desired_yaw=desired_yaw,
            )
            return

        desired_forward = 0.0
        desired_right = 0.0
        desired_down = 0.0

        if mission_mode == "APPROACH_TARGET":
            desired_distance = float(mission.desired_distance_m) if mission is not None and mission.desired_distance_m > 0.0 else self.desired_distance_m
            desired_forward = self.get_distance_forward_correction(target, desired_distance)
        elif mission_mode == "ORBIT_TARGET":
            desired_distance = float(mission.orbit_radius_m) if mission is not None and mission.orbit_radius_m > 0.0 else self.desired_distance_m
            desired_forward = self.get_distance_forward_correction(target, desired_distance)
            orbit_speed = float(mission.orbit_speed_m_s) if mission is not None and mission.orbit_speed_m_s != 0.0 else self.orbit_speed_m_s
            desired_right = orbit_speed
        elif mission_mode == "TRACK_CENTER":
            desired_forward = 0.0
            desired_right = 0.0
        else:
            self.publish_idle(
                f"UNKNOWN_MISSION_MODE:{mission_mode}",
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                desired_yaw=desired_yaw,
            )
            return

        forward, right, down, yaw = self.limit_motion(desired_forward, desired_right, desired_down, desired_yaw)

        safe, reason = self.check_safety(current_time)
        if not safe:
            self.publish_idle(
                reason,
                source_error_x=float(target.error_x),
                source_error_y=float(target.error_y),
                desired_yaw=yaw,
            )
            return

        command = self.make_command(
            CMD_VELOCITY,
            STATUS_SENT,
            executed=True,
            velocity_forward=forward,
            velocity_right=right,
            velocity_down=down,
            yaw_rate=yaw,
            source_error_x=float(target.error_x),
            source_error_y=float(target.error_y),
        )

        self.last_command_forward = forward
        self.last_command_right = right
        self.last_command_down = down
        self.last_command_yaw = yaw

        self.command_pub.publish(command)
        self.command_count += 1
        self.executed_command_count += 1

        self.diagnostics.mark_published(
            self.control_command_topic,
            summary=(
                f"commands={self.command_count}, executed={self.executed_command_count}, "
                f"mode={mission_mode}, forward={forward:.3f}, right={right:.3f}, yaw={yaw:.3f}, "
                f"status={STATUS_SENT}"
            ),
        )

    def report_status(self) -> None:
        self.get_logger().info(
            f"Control status | autonomy={self.autonomy_enabled}, target_locked={self.target_locked}, "
            f"commands={self.command_count}, executed={self.executed_command_count}, idle={self.idle_command_count}, "
            f"target_msgs={self.target_error_count}, telemetry_msgs={self.telemetry_count}, "
            f"autonomy_msgs={self.autonomy_enable_count}, mission_msgs={self.mission_command_count}, "
            f"mission_mode={self.last_mission_mode}, "
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
