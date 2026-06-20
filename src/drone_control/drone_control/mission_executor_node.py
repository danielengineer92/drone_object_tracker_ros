"""
Smart mission executor node for the PX4/ROS 2 object-tracking drone.

Operator intent is now simple:
    System Ready     -> autonomy request true
    Start Mission    -> this node owns the sequence
    Abort / Hold     -> stop mission motion and request HOLD
    Land             -> request LAND through the MAVSDK action gate

Start Mission sequence:
    1. Check telemetry connected/fresh
    2. Check PX4 is armed
    3. If landed or below the airborne threshold, request TAKEOFF
    4. Wait until airborne
    5. Request MAVSDK Offboard through the autonomy manager
    6. Prime Offboard with zero/HOLD setpoints
    7. Publish TRACK_CENTER so the control node can yaw toward the YOLO target

This node does not arm the vehicle. MAVSDK actions are still gated by telemetry_node
through allow_mavsdk_actions, so real hardware can keep those actions disabled while
SITL/dev setups can enable them intentionally.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from drone_interfaces.msg import DroneTelemetry, MavsdkActionCommand, MissionCommand, TargetError
from drone_diagnostics.node_diagnostics import NodeDiagnostics


class MissionState(Enum):
    DISABLED = "DISABLED"
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    PRIME_OFFBOARD = "PRIME_OFFBOARD"
    TRACK_CENTER = "TRACK_CENTER"
    APPROACH_TARGET = "APPROACH_TARGET"
    DO_ORBIT = "DO_ORBIT"
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    LAND = "LAND"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


STEP_NAMES = {
    MissionState.PREFLIGHT: "0_preflight",
    MissionState.TAKEOFF: "1_takeoff_if_needed",
    MissionState.PRIME_OFFBOARD: "2_prime_offboard",
    MissionState.TRACK_CENTER: "3_track_center_yaw",
    MissionState.APPROACH_TARGET: "4_approach_ball_later",
    MissionState.DO_ORBIT: "5_orbit_ball_later",
    MissionState.RETURN_TO_LAUNCH: "6_return_home_later",
    MissionState.LAND: "7_land",
}


class MissionExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor_node")

        self.declare_parameter("mission_enabled", False)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("mission_request_topic", "/drone/mission/request")
        self.declare_parameter("mission_command_topic", "/drone/mission/command")
        self.declare_parameter("mission_state_topic", "/drone/mission/state")
        self.declare_parameter("mavsdk_action_topic", "/drone/mavsdk/action_command")
        self.declare_parameter("autonomy_request_topic", "/drone/autonomy/request")
        self.declare_parameter("offboard_request_topic", "/drone/mavsdk/offboard_request")
        self.declare_parameter("target_error_topic", "/drone/tracking/target_error")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("publish_rate", 10.0)

        # Smart mission behavior.
        self.declare_parameter("require_connected_for_mission", True)
        self.declare_parameter("require_armed_for_mission", True)
        self.declare_parameter("telemetry_timeout_s", 2.0)
        self.declare_parameter("takeoff_altitude_m", 2.0)
        self.declare_parameter("airborne_altitude_m", 0.75)
        self.declare_parameter("takeoff_timeout_s", 20.0)
        self.declare_parameter("offboard_prime_time_s", 1.5)
        self.declare_parameter("track_center_timeout_s", 0.0)  # 0 = keep yaw tracking until stopped.
        self.declare_parameter("run_full_orbit_after_track_center", False)

        # Later/full mission behavior parameters. Kept so the old orbit path can be re-enabled later.
        self.declare_parameter("target_timeout_s", 2.0)
        self.declare_parameter("desired_approach_distance_m", 2.0)
        self.declare_parameter("approach_distance_tolerance_m", 0.25)
        self.declare_parameter("approach_timeout_s", 20.0)
        self.declare_parameter("orbit_radius_m", 2.0)
        self.declare_parameter("orbit_speed_m_s", 0.4)
        self.declare_parameter("orbit_revolutions", 1.0)
        self.declare_parameter("orbit_timeout_s", 45.0)
        self.declare_parameter("rtl_wait_s", 10.0)
        self.declare_parameter("land_wait_s", 10.0)
        self.declare_parameter("use_mavsdk_do_orbit", True)
        self.declare_parameter("require_distance_for_orbit", True)
        self.declare_parameter("require_target_centered_for_orbit", True)
        self.declare_parameter("center_error_threshold", 0.15)

        self.mission_enabled = bool(self.get_parameter("mission_enabled").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.mission_request_topic = str(self.get_parameter("mission_request_topic").value)
        self.mission_command_topic = str(self.get_parameter("mission_command_topic").value)
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        self.mavsdk_action_topic = str(self.get_parameter("mavsdk_action_topic").value)
        self.autonomy_request_topic = str(self.get_parameter("autonomy_request_topic").value)
        self.offboard_request_topic = str(self.get_parameter("offboard_request_topic").value)
        self.target_error_topic = str(self.get_parameter("target_error_topic").value)
        self.telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self.publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))

        self.require_connected_for_mission = bool(self.get_parameter("require_connected_for_mission").value)
        self.require_armed_for_mission = bool(self.get_parameter("require_armed_for_mission").value)
        self.telemetry_timeout_s = float(self.get_parameter("telemetry_timeout_s").value)
        self.takeoff_altitude_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.airborne_altitude_m = float(self.get_parameter("airborne_altitude_m").value)
        self.takeoff_timeout_s = float(self.get_parameter("takeoff_timeout_s").value)
        self.offboard_prime_time_s = float(self.get_parameter("offboard_prime_time_s").value)
        self.track_center_timeout_s = float(self.get_parameter("track_center_timeout_s").value)
        self.run_full_orbit_after_track_center = bool(self.get_parameter("run_full_orbit_after_track_center").value)

        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.desired_approach_distance_m = float(self.get_parameter("desired_approach_distance_m").value)
        self.approach_distance_tolerance_m = float(self.get_parameter("approach_distance_tolerance_m").value)
        self.approach_timeout_s = float(self.get_parameter("approach_timeout_s").value)
        self.orbit_radius_m = float(self.get_parameter("orbit_radius_m").value)
        self.orbit_speed_m_s = float(self.get_parameter("orbit_speed_m_s").value)
        self.orbit_revolutions = float(self.get_parameter("orbit_revolutions").value)
        self.orbit_timeout_s = float(self.get_parameter("orbit_timeout_s").value)
        self.rtl_wait_s = float(self.get_parameter("rtl_wait_s").value)
        self.land_wait_s = float(self.get_parameter("land_wait_s").value)
        self.use_mavsdk_do_orbit = bool(self.get_parameter("use_mavsdk_do_orbit").value)
        self.require_distance_for_orbit = bool(self.get_parameter("require_distance_for_orbit").value)
        self.require_target_centered_for_orbit = bool(self.get_parameter("require_target_centered_for_orbit").value)
        self.center_error_threshold = float(self.get_parameter("center_error_threshold").value)

        self.state = MissionState.IDLE if self.mission_enabled else MissionState.DISABLED
        self.state_enter_time = time.time()
        self.mission_active = bool(self.auto_start and self.mission_enabled)
        self.action_command_id = 0
        self.actions_sent: set[str] = set()
        self._last_autonomy_request: Optional[bool] = None
        self._last_offboard_request: Optional[bool] = None
        self._last_autonomy_request_publish_time = 0.0
        self._last_offboard_request_publish_time = 0.0

        self.last_target: Optional[TargetError] = None
        self.last_target_time = 0.0
        self.last_telemetry: Optional[DroneTelemetry] = None
        self.last_telemetry_time = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.request_sub = self.create_subscription(Bool, self.mission_request_topic, self.mission_request_callback, qos)
        self.target_sub = self.create_subscription(TargetError, self.target_error_topic, self.target_callback, qos)
        self.telemetry_sub = self.create_subscription(DroneTelemetry, self.telemetry_topic, self.telemetry_callback, qos)
        self.command_pub = self.create_publisher(MissionCommand, self.mission_command_topic, qos)
        self.state_pub = self.create_publisher(String, self.mission_state_topic, qos)
        self.action_pub = self.create_publisher(MavsdkActionCommand, self.mavsdk_action_topic, qos)
        self.autonomy_request_pub = self.create_publisher(Bool, self.autonomy_request_topic, qos)
        self.offboard_request_pub = self.create_publisher(Bool, self.offboard_request_topic, qos)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.loop)
        self.diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self.diagnostics.add_input(self.mission_request_topic, "mission_request", stale_seconds=60.0)
        self.diagnostics.add_input(self.target_error_topic, "target_error", stale_seconds=self.target_timeout_s)
        self.diagnostics.add_input(self.telemetry_topic, "telemetry", stale_seconds=self.telemetry_timeout_s)
        self.diagnostics.add_output(self.mission_command_topic, "mission_command")
        self.diagnostics.add_output(self.mission_state_topic, "mission_state")
        self.diagnostics.add_output(self.mavsdk_action_topic, "mavsdk_action_command")
        self.diagnostics.add_output(self.autonomy_request_topic, "autonomy_request")
        self.diagnostics.add_output(self.offboard_request_topic, "mavsdk_offboard_request")

        self.get_logger().warning(
            f"Mission executor started | enabled={self.mission_enabled}, active={self.mission_active}, "
            f"request_topic={self.mission_request_topic}, state_topic={self.mission_state_topic}, "
            f"command_topic={self.mission_command_topic}, mavsdk_action_topic={self.mavsdk_action_topic}, "
            f"autonomy_request_topic={self.autonomy_request_topic}, offboard_request_topic={self.offboard_request_topic}, "
            f"takeoff_altitude={self.takeoff_altitude_m:.1f}m, airborne_altitude={self.airborne_altitude_m:.1f}m"
        )
        if not self.mission_enabled:
            self.get_logger().warning("Mission executor is disabled by parameter. Start Mission will stay DISABLED until enabled in config.")

    def mission_request_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested:
            if not self.mission_enabled:
                self.get_logger().warning("Mission start requested but mission_enabled is false; staying DISABLED.")
                self.state = MissionState.DISABLED
                self.mission_active = False
                self.publish_autonomy_request(False)
                self.publish_offboard_request(False)
                return
            self.get_logger().warning("*** SMART YOLO TRACK-CENTER MISSION REQUESTED ***")
            self.mission_active = True
            self.actions_sent.clear()
            self.publish_autonomy_request(True)
            self.publish_offboard_request(False)
            self.transition(MissionState.PREFLIGHT)
        else:
            self.get_logger().warning("Mission stop requested; publishing HOLD/IDLE and dropping autonomy/offboard requests.")
            self.mission_active = False
            self.actions_sent.clear()
            self.publish_offboard_request(False)
            self.publish_autonomy_request(False)
            self.transition(MissionState.IDLE if self.mission_enabled else MissionState.DISABLED)
        self.diagnostics.mark_received(self.mission_request_topic, summary=f"request={requested}, active={self.mission_active}")

    def target_callback(self, msg: TargetError) -> None:
        self.last_target = msg
        self.last_target_time = time.time()
        self.diagnostics.mark_received(
            self.target_error_topic,
            summary=f"visible={msg.target_visible}, state={msg.tracking_state}, dist={getattr(msg, 'distance_m', 0.0):.2f}",
        )

    def telemetry_callback(self, msg: DroneTelemetry) -> None:
        self.last_telemetry = msg
        self.last_telemetry_time = time.time()
        self.diagnostics.mark_received(
            self.telemetry_topic,
            summary=(
                f"connected={msg.connected}, armed={msg.armed}, mode={msg.flight_mode}, "
                f"landed={msg.landed_state}, alt={msg.relative_altitude:.2f}"
            ),
        )

    def publish_autonomy_request(self, enabled: bool) -> None:
        enabled = bool(enabled)
        now = time.time()
        changed = self._last_autonomy_request != enabled
        if not changed and now - self._last_autonomy_request_publish_time < 1.0:
            return
        self._last_autonomy_request = enabled
        self._last_autonomy_request_publish_time = now
        msg = Bool()
        msg.data = enabled
        self.autonomy_request_pub.publish(msg)
        self.diagnostics.mark_published(self.autonomy_request_topic, summary=f"requested={enabled}")
        if changed:
            self.get_logger().warning(f"Mission executor autonomy request: {enabled}")

    def publish_offboard_request(self, enabled: bool) -> None:
        enabled = bool(enabled)
        now = time.time()
        changed = self._last_offboard_request != enabled
        if not changed and now - self._last_offboard_request_publish_time < 1.0:
            return
        self._last_offboard_request = enabled
        self._last_offboard_request_publish_time = now
        msg = Bool()
        msg.data = enabled
        self.offboard_request_pub.publish(msg)
        self.diagnostics.mark_published(self.offboard_request_topic, summary=f"requested={enabled}")
        if changed:
            self.get_logger().warning(f"Mission executor MAVSDK Offboard request: {enabled}")

    def transition(self, new_state: MissionState) -> None:
        if new_state == self.state:
            return
        self.get_logger().warning(f"Mission state: {self.state.value} -> {new_state.value}")
        self.state = new_state
        self.state_enter_time = time.time()

    def state_age(self) -> float:
        return time.time() - self.state_enter_time

    def telemetry_is_fresh(self) -> bool:
        if self.last_telemetry is None:
            return False
        return time.time() - self.last_telemetry_time <= self.telemetry_timeout_s

    def preflight_ok(self) -> tuple[bool, str]:
        if self.last_telemetry is None:
            return False, "NO_TELEMETRY"
        age = time.time() - self.last_telemetry_time
        if age > self.telemetry_timeout_s:
            return False, f"TELEMETRY_STALE ({age:.2f}s)"
        if self.require_connected_for_mission and not bool(self.last_telemetry.connected):
            return False, "PX4_NOT_CONNECTED"
        if self.require_armed_for_mission and not bool(self.last_telemetry.armed):
            return False, "PX4_NOT_ARMED"
        return True, "PREFLIGHT_OK"

    def is_airborne(self) -> bool:
        if self.last_telemetry is None:
            return False
        altitude = float(self.last_telemetry.relative_altitude)
        landed_state = str(self.last_telemetry.landed_state).upper()
        if "IN_AIR" in landed_state or "TAKING_OFF" in landed_state:
            return True
        if "ON_GROUND" in landed_state or "LANDED" in landed_state:
            return False
        return altitude >= self.airborne_altitude_m

    def target_is_fresh_locked(self) -> bool:
        if self.last_target is None:
            return False
        if time.time() - self.last_target_time > self.target_timeout_s:
            return False
        return bool(self.last_target.target_visible and self.last_target.tracking_state == "LOCKED")

    def target_distance_ready(self) -> bool:
        if not self.target_is_fresh_locked() or self.last_target is None:
            return False
        if not bool(getattr(self.last_target, "distance_valid", False)):
            return False
        return float(getattr(self.last_target, "distance_m", 0.0)) > 0.0

    def target_centered(self) -> bool:
        if not self.target_is_fresh_locked() or self.last_target is None:
            return False
        return abs(float(self.last_target.error_x)) <= self.center_error_threshold

    def approach_done(self) -> bool:
        if not self.target_distance_ready() or self.last_target is None:
            return False
        return abs(float(self.last_target.distance_m) - self.desired_approach_distance_m) <= self.approach_distance_tolerance_m

    def send_action_once(self, key: str, action: str, note: str = "") -> None:
        if key in self.actions_sent:
            return
        self.actions_sent.add(key)
        self.action_command_id += 1
        msg = MavsdkActionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = int(self.action_command_id)
        msg.action = action
        msg.execute = True
        msg.takeoff_altitude_m = float(self.takeoff_altitude_m)
        msg.radius_m = float(self.orbit_radius_m)
        msg.velocity_m_s = float(self.orbit_speed_m_s)
        msg.orbit_revolutions = float(self.orbit_revolutions)
        msg.yaw_behavior = "FRONT_TO_CIRCLE_CENTER"
        msg.latitude_deg = math.nan
        msg.longitude_deg = math.nan
        msg.absolute_altitude_m = math.nan
        msg.note = note

        if action == "DO_ORBIT":
            center = self.estimate_target_global_center()
            if center is not None:
                msg.latitude_deg, msg.longitude_deg, msg.absolute_altitude_m = center
                msg.note = note + " | center=estimated_target_global"
            else:
                msg.note = note + " | center=current_position_nan"

        self.action_pub.publish(msg)
        self.diagnostics.mark_published(self.mavsdk_action_topic, summary=f"id={msg.command_id}, action={action}, note={msg.note}")
        self.get_logger().warning(f"Mission requested MAVSDK action: id={msg.command_id}, action={action}, note={msg.note}")

    def estimate_target_global_center(self) -> Optional[tuple[float, float, float]]:
        if self.last_target is None or self.last_telemetry is None:
            return None
        if not self.target_distance_ready():
            return None
        lat = float(self.last_telemetry.latitude)
        lon = float(self.last_telemetry.longitude)
        alt = float(self.last_telemetry.absolute_altitude)
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return None

        distance_m = float(self.last_target.distance_m)
        bearing_x = float(getattr(self.last_target, "bearing_x_rad", 0.0))
        yaw = float(self.last_telemetry.yaw)
        global_bearing = yaw + bearing_x
        north_m = distance_m * math.cos(global_bearing)
        east_m = distance_m * math.sin(global_bearing)

        earth_radius_m = 6378137.0
        lat_rad = math.radians(lat)
        out_lat = lat + math.degrees(north_m / earth_radius_m)
        out_lon = lon + math.degrees(east_m / (earth_radius_m * max(math.cos(lat_rad), 1e-6)))
        return out_lat, out_lon, alt

    def publish_state(self, detail: str) -> None:
        msg = String()
        msg.data = f"{self.state.value}: {detail}"
        self.state_pub.publish(msg)
        self.diagnostics.mark_published(self.mission_state_topic, summary=msg.data)

    def publish_mission_command(self, mode: str, active: bool, status: str) -> None:
        msg = MissionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        msg.active = bool(active)
        msg.velocity_forward = 0.0
        msg.velocity_right = 0.0
        msg.velocity_down = 0.0
        msg.yaw_rate = 0.0
        msg.desired_distance_m = float(self.desired_approach_distance_m)
        msg.orbit_radius_m = float(self.orbit_radius_m)
        msg.orbit_speed_m_s = float(self.orbit_speed_m_s)
        msg.step_index = self.step_index_for_state(self.state)
        msg.step_name = STEP_NAMES.get(self.state, self.state.value.lower())
        msg.status = status

        if mode == "ORBIT_TARGET" and not self.use_mavsdk_do_orbit:
            msg.velocity_right = float(self.orbit_speed_m_s)

        self.command_pub.publish(msg)
        self.diagnostics.mark_published(self.mission_command_topic, summary=f"mode={mode}, active={active}, status={status}")

    @staticmethod
    def step_index_for_state(state: MissionState) -> int:
        order = [
            MissionState.PREFLIGHT,
            MissionState.TAKEOFF,
            MissionState.PRIME_OFFBOARD,
            MissionState.TRACK_CENTER,
            MissionState.APPROACH_TARGET,
            MissionState.DO_ORBIT,
            MissionState.RETURN_TO_LAUNCH,
            MissionState.LAND,
        ]
        try:
            return order.index(state)
        except ValueError:
            return 0

    def loop(self) -> None:
        if not self.mission_enabled:
            self.state = MissionState.DISABLED
            self.publish_offboard_request(False)
            self.publish_autonomy_request(False)
            self.publish_mission_command("IDLE", False, "mission disabled")
            self.publish_state("mission_enabled=false")
            return

        if not self.mission_active:
            self.publish_mission_command("IDLE", False, "waiting for /drone/mission/request true")
            self.publish_state("idle")
            return

        age = self.state_age()
        self.publish_autonomy_request(True)

        if self.state == MissionState.PREFLIGHT:
            self.publish_offboard_request(False)
            self.publish_mission_command("HOLD", True, "checking PX4 link, telemetry, and armed state")
            ok, reason = self.preflight_ok()
            if not ok:
                self.publish_state(f"blocked: {reason}")
                return
            if self.is_airborne():
                self.publish_state("already airborne; skipping takeoff")
                self.transition(MissionState.PRIME_OFFBOARD)
            else:
                self.publish_state("armed on ground/low altitude; requesting takeoff")
                self.transition(MissionState.TAKEOFF)
            return

        if self.state == MissionState.TAKEOFF:
            ok, reason = self.preflight_ok()
            if not ok:
                self.publish_state(f"takeoff blocked: {reason}")
                self.publish_mission_command("HOLD", True, f"takeoff blocked: {reason}")
                return
            self.publish_offboard_request(False)
            self.send_action_once("takeoff", "TAKEOFF", "smart mission takeoff before Offboard")
            self.publish_mission_command("HOLD", True, "takeoff action requested; waiting until airborne")
            self.publish_state(f"takeoff requested, airborne={self.is_airborne()}, age={age:.1f}/{self.takeoff_timeout_s:.1f}s")
            if self.is_airborne():
                self.transition(MissionState.PRIME_OFFBOARD)
            elif age > self.takeoff_timeout_s:
                # Stay in TAKEOFF instead of yawing on the ground. This catches disabled action gates.
                self.publish_state("takeoff timeout; still waiting for airborne telemetry")
            return

        if self.state == MissionState.PRIME_OFFBOARD:
            ok, reason = self.preflight_ok()
            if not ok:
                self.publish_state(f"prime blocked: {reason}")
                self.publish_mission_command("HOLD", True, f"prime blocked: {reason}")
                return
            self.publish_offboard_request(True)
            self.publish_mission_command("HOLD", True, "priming PX4 Offboard with zero/hold setpoints")
            self.publish_state(f"priming offboard, age={age:.1f}/{self.offboard_prime_time_s:.1f}s")
            if age >= self.offboard_prime_time_s:
                self.transition(MissionState.TRACK_CENTER)
            return

        if self.state == MissionState.TRACK_CENTER:
            ok, reason = self.preflight_ok()
            if not ok:
                self.publish_state(f"track blocked: {reason}")
                self.publish_mission_command("HOLD", True, f"track blocked: {reason}")
                return
            self.publish_offboard_request(True)
            locked = self.target_is_fresh_locked()
            self.publish_mission_command("TRACK_CENTER", True, "yaw toward YOLO target; no forward motion")
            self.publish_state(f"track center yaw, locked={locked}, age={age:.1f}s")
            if self.run_full_orbit_after_track_center and locked and self.target_centered():
                self.transition(MissionState.APPROACH_TARGET)
            elif self.track_center_timeout_s > 0.0 and age > self.track_center_timeout_s:
                self.transition(MissionState.COMPLETE)
            return

        # Later/full mission path. Disabled by default via run_full_orbit_after_track_center=false.
        if self.state == MissionState.APPROACH_TARGET:
            self.publish_offboard_request(True)
            self.publish_mission_command("APPROACH_TARGET", True, "later step: approach using distance estimate")
            dist_text = "none"
            if self.last_target is not None and bool(getattr(self.last_target, "distance_valid", False)):
                dist_text = f"{self.last_target.distance_m:.2f}m"
            self.publish_state(f"approaching, distance={dist_text}, age={age:.1f}s")
            if not self.target_is_fresh_locked():
                self.transition(MissionState.TRACK_CENTER)
            elif self.approach_done():
                self.transition(MissionState.DO_ORBIT)
            elif age > self.approach_timeout_s:
                self.transition(MissionState.DO_ORBIT)
            return

        if self.state == MissionState.DO_ORBIT:
            self.publish_offboard_request(True)
            if self.require_distance_for_orbit and not self.target_distance_ready():
                self.publish_mission_command("TRACK_CENTER", True, "waiting for valid distance before orbit")
                self.publish_state("orbit hold: distance not ready")
                return
            if self.require_target_centered_for_orbit and not self.target_centered():
                self.publish_mission_command("TRACK_CENTER", True, "centering target before orbit")
                self.publish_state("orbit hold: target not centered")
                return

            if self.use_mavsdk_do_orbit:
                self.send_action_once("do_orbit", "DO_ORBIT", "later step: MAV_CMD_DO_ORBIT around estimated ball center")
                self.publish_mission_command("HOLD", True, "DO_ORBIT requested; PX4 owns orbit if accepted")
            else:
                self.publish_mission_command("ORBIT_TARGET", True, "visual-servo orbit fallback")
            self.publish_state(f"orbiting/requested, age={age:.1f}/{self.orbit_timeout_s:.1f}s")
            if age > self.orbit_timeout_s:
                self.transition(MissionState.RETURN_TO_LAUNCH)
            return

        if self.state == MissionState.RETURN_TO_LAUNCH:
            self.publish_offboard_request(False)
            self.send_action_once("rtl", "RETURN_TO_LAUNCH", "later step: return to launch")
            self.publish_mission_command("HOLD", True, "RTL requested")
            self.publish_state(f"returning, age={age:.1f}/{self.rtl_wait_s:.1f}s")
            if age > self.rtl_wait_s:
                self.transition(MissionState.LAND)
            return

        if self.state == MissionState.LAND:
            self.publish_offboard_request(False)
            self.send_action_once("land", "LAND", "later step: land")
            self.publish_mission_command("HOLD", True, "land requested")
            self.publish_state(f"landing, age={age:.1f}/{self.land_wait_s:.1f}s")
            if age > self.land_wait_s:
                self.transition(MissionState.COMPLETE)
            return

        if self.state == MissionState.COMPLETE:
            self.publish_offboard_request(False)
            self.publish_mission_command("IDLE", False, "mission complete")
            self.publish_state("complete")
            self.mission_active = False
            return

        self.publish_mission_command("IDLE", False, f"unhandled state {self.state.value}")
        self.publish_state(f"unhandled state {self.state.value}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionExecutorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("mission_executor_node").fatal(f"Fatal: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
