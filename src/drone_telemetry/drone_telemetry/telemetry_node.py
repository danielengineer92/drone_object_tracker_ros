"""
PX4 MAVSDK bridge node for the drone vision system.

This node owns the single MAVSDK connection to the Pixhawk. It publishes
telemetry and, when explicitly enabled, consumes /drone/control/command and sends
safe yaw-only Offboard velocity setpoints to PX4.

Safety posture for this first command bridge:
- Does NOT arm the drone.
- Does NOT take off.
- Does NOT command forward/right/down motion by default.
- Requires /drone/mavsdk/offboard_enable true before starting Offboard.
- Sends zero body velocity/yawspeed unless the latest ControlCommand is fresh,
  command_type == VELOCITY, executed == true, and execution_status == SENT.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from drone_interfaces.msg import ControlCommand, DroneTelemetry
from drone_diagnostics.node_diagnostics import NodeDiagnostics


CMD_VELOCITY = "VELOCITY"
STATUS_SENT = "SENT"

BRIDGE_DISABLED = "MAVSDK_OFFBOARD_DISABLED"
BRIDGE_NO_COMMAND = "NO_CONTROL_COMMAND"
BRIDGE_COMMAND_STALE = "CONTROL_COMMAND_STALE"
BRIDGE_COMMAND_IDLE = "CONTROL_COMMAND_IDLE"
BRIDGE_COMMAND_NOT_APPROVED = "CONTROL_COMMAND_NOT_APPROVED"
BRIDGE_NOT_CONNECTED = "PX4_NOT_CONNECTED"
BRIDGE_NOT_ARMED = "PX4_NOT_ARMED"
BRIDGE_LOW_BATTERY = "PX4_LOW_BATTERY"
BRIDGE_READY = "READY_TO_SEND"
BRIDGE_SENT = "SENT_TO_PX4"
BRIDGE_ZERO_SENT = "ZERO_SENT_TO_PX4"
BRIDGE_OFFBOARD_STARTED = "OFFBOARD_STARTED"
BRIDGE_OFFBOARD_STOPPED = "OFFBOARD_STOPPED"
BRIDGE_OFFBOARD_START_FAILED = "OFFBOARD_START_FAILED"
BRIDGE_SEND_FAILED = "OFFBOARD_SEND_FAILED"


class TelemetryNode(Node):
    """ROS 2 node that bridges PX4/MAVSDK telemetry and yaw-only commands."""

    def __init__(self) -> None:
        super().__init__('telemetry_node')

        # Connection and telemetry parameters
        self.declare_parameter('connection_url', 'serial:///dev/ttyACM0:57600')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('connection_timeout', 10.0)
        self.declare_parameter('telemetry_topic', '/drone/telemetry')

        # Command bridge parameters
        self.declare_parameter('control_command_topic', '/drone/control/command')
        self.declare_parameter('offboard_enable_topic', '/drone/mavsdk/offboard_enable')
        self.declare_parameter('command_status_topic', '/drone/mavsdk/command_status')
        self.declare_parameter('mavsdk_offboard_enabled', False)
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('require_armed_for_offboard', True)
        self.declare_parameter('min_battery_percent', 20.0)
        self.declare_parameter('max_yaw_rate_rad_s', 1.0)
        self.declare_parameter('allow_translation_commands', False)
        self.declare_parameter('stop_offboard_on_disable', True)

        # Read parameters
        self._connection_url: str = str(self.get_parameter('connection_url').value)
        self._publish_rate: float = float(self.get_parameter('publish_rate').value)
        self._reconnect_interval: float = float(self.get_parameter('reconnect_interval').value)
        self._connection_timeout: float = float(self.get_parameter('connection_timeout').value)
        self._telemetry_topic: str = str(self.get_parameter('telemetry_topic').value)

        self._control_command_topic: str = str(self.get_parameter('control_command_topic').value)
        self._offboard_enable_topic: str = str(self.get_parameter('offboard_enable_topic').value)
        self._command_status_topic: str = str(self.get_parameter('command_status_topic').value)
        self._mavsdk_offboard_enabled: bool = bool(self.get_parameter('mavsdk_offboard_enabled').value)
        self._command_rate: float = float(self.get_parameter('command_rate').value)
        self._command_timeout: float = float(self.get_parameter('command_timeout').value)
        self._require_armed_for_offboard: bool = bool(self.get_parameter('require_armed_for_offboard').value)
        self._min_battery_percent: float = float(self.get_parameter('min_battery_percent').value)
        self._max_yaw_rate_rad_s: float = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self._allow_translation_commands: bool = bool(self.get_parameter('allow_translation_commands').value)
        self._stop_offboard_on_disable: bool = bool(self.get_parameter('stop_offboard_on_disable').value)

        self._validate_parameters()

        # MAVSDK state
        self._system = None
        self._connected: bool = False
        self._connection_status: str = "DISCONNECTED"
        self._offboard_active: bool = False
        self._telemetry_publish_count: int = 0
        self._control_command_count: int = 0
        self._offboard_enable_count: int = 0
        self._px4_send_count: int = 0
        self._px4_zero_count: int = 0
        self._last_bridge_status: str = BRIDGE_DISABLED
        self._last_status_publish_time: float = 0.0
        self._last_log_times: dict[str, float] = {}

        # Telemetry data protected by lock
        self._data_lock = threading.Lock()
        self._telemetry_data: dict = {
            'battery_voltage': 0.0,
            'battery_remaining': 0.0,
            'latitude': 0.0,
            'longitude': 0.0,
            'absolute_altitude': 0.0,
            'relative_altitude': 0.0,
            'gps_num_satellites': 0,
            'gps_fix_type': 0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'velocity_north': 0.0,
            'velocity_east': 0.0,
            'velocity_down': 0.0,
            'armed': False,
            'flight_mode': 'UNKNOWN',
            'landed_state': 'UNKNOWN',
            'health_all_ok': False,
            'health_accelerometer_ok': False,
            'health_gyroscope_ok': False,
            'health_magnetometer_ok': False,
            'health_gps_ok': False,
        }

        # Latest control command protected by lock
        self._command_lock = threading.Lock()
        self._latest_command: Optional[ControlCommand] = None
        self._latest_command_time: float = 0.0

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._telemetry_pub = self.create_publisher(
            DroneTelemetry,
            self._telemetry_topic,
            reliable_qos,
        )
        self._command_status_pub = self.create_publisher(
            String,
            self._command_status_topic,
            reliable_qos,
        )

        self._command_sub = self.create_subscription(
            ControlCommand,
            self._control_command_topic,
            self._control_command_callback,
            reliable_qos,
        )
        self._offboard_enable_sub = self.create_subscription(
            Bool,
            self._offboard_enable_topic,
            self._offboard_enable_callback,
            reliable_qos,
        )

        publish_period = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_telemetry)

        self._diagnostics = NodeDiagnostics(self, heartbeat_period=5.0, stale_seconds=2.0)
        self._diagnostics.add_output(self._telemetry_topic, "telemetry")
        self._diagnostics.add_input(self._control_command_topic, "control_command", stale_seconds=self._command_timeout)
        self._diagnostics.add_input(self._offboard_enable_topic, "offboard_enable", stale_seconds=60.0)
        self._diagnostics.add_output(self._command_status_topic, "mavsdk_command_status")

        self._status_timer = self.create_timer(5.0, self._report_status)

        self._async_thread: Optional[threading.Thread] = None
        self._running: bool = True
        self._start_mavsdk_connection()

        self.get_logger().warning(
            'PX4 MAVSDK bridge initialized | '
            f'telemetry_topic={self._telemetry_topic}, '
            f'control_command_topic={self._control_command_topic}, '
            f'offboard_enable_topic={self._offboard_enable_topic}, '
            f'command_status_topic={self._command_status_topic}, '
            f'url={self._connection_url}, telemetry_rate={self._publish_rate:.1f}Hz, '
            f'command_rate={self._command_rate:.1f}Hz, '
            f'mavsdk_offboard_enabled={self._mavsdk_offboard_enabled}, '
            f'mode=YAW_ONLY, max_yaw={self._max_yaw_rate_rad_s:.2f}rad/s, '
            'arming=manual_only, takeoff=not_implemented'
        )
        if not self._mavsdk_offboard_enabled:
            self.get_logger().warning(
                'MAVSDK OFFBOARD DISABLED - this node will publish telemetry but will not send movement '
                'setpoints until /drone/mavsdk/offboard_enable is true.'
            )

    def _validate_parameters(self) -> None:
        if self._publish_rate <= 0.0:
            raise ValueError(f'publish_rate must be > 0, got {self._publish_rate}')
        if self._command_rate <= 0.0:
            raise ValueError(f'command_rate must be > 0, got {self._command_rate}')
        if self._command_timeout <= 0.0:
            raise ValueError(f'command_timeout must be > 0, got {self._command_timeout}')
        if self._max_yaw_rate_rad_s < 0.0:
            raise ValueError(f'max_yaw_rate_rad_s must be >= 0, got {self._max_yaw_rate_rad_s}')
        if self._min_battery_percent < 0.0:
            raise ValueError(f'min_battery_percent must be >= 0, got {self._min_battery_percent}')

    def _control_command_callback(self, msg: ControlCommand) -> None:
        with self._command_lock:
            self._latest_command = msg
            self._latest_command_time = time.monotonic()
            self._control_command_count += 1

        self._diagnostics.mark_received(
            self._control_command_topic,
            summary=(
                f'messages={self._control_command_count}, type={msg.command_type}, '
                f'executed={msg.executed}, status={msg.execution_status}, yaw={msg.yaw_rate:+.3f}'
            ),
        )

    def _offboard_enable_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        old_enabled = self._mavsdk_offboard_enabled
        self._mavsdk_offboard_enabled = enabled
        self._offboard_enable_count += 1

        if enabled and not old_enabled:
            self.get_logger().warning('*** MAVSDK OFFBOARD EXECUTOR ENABLED by /drone/mavsdk/offboard_enable ***')
        elif not enabled and old_enabled:
            self.get_logger().warning('MAVSDK offboard executor disabled; sending/stopping zero setpoints.')

        self._diagnostics.mark_received(
            self._offboard_enable_topic,
            summary=f'messages={self._offboard_enable_count}, enabled={self._mavsdk_offboard_enabled}',
        )
        self._publish_command_status(
            f'{BRIDGE_READY if enabled else BRIDGE_DISABLED}: enabled={self._mavsdk_offboard_enabled}',
            force=True,
        )

    def _start_mavsdk_connection(self) -> None:
        self._async_thread = threading.Thread(
            target=self._run_async_loop,
            daemon=True,
            name='mavsdk_bridge_thread',
        )
        self._async_thread.start()

    def _run_async_loop(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            loop.run_until_complete(self._mavsdk_main())
        except Exception as exc:
            self.get_logger().error(f'MAVSDK async loop error: {exc}')
        finally:
            loop.close()

    async def _mavsdk_main(self) -> None:
        while self._running:
            try:
                await self._connect_and_stream()
            except Exception as exc:
                self.get_logger().error(f'MAVSDK connection/bridge error: {exc}')
                self._connected = False
                self._offboard_active = False
                self._connection_status = f"ERROR: {str(exc)[:50]}"

            if self._running:
                self.get_logger().info(f'Reconnecting in {self._reconnect_interval}s...')
                await asyncio.sleep(self._reconnect_interval)

    async def _connect_and_stream(self) -> None:
        try:
            from mavsdk import System
        except ImportError:
            self.get_logger().error('MAVSDK-Python not installed. Install with: pip install mavsdk')
            self._connection_status = "MAVSDK_NOT_INSTALLED"
            self._running = False
            return

        self.get_logger().info(f'Connecting to PX4 at: {self._connection_url}')
        self._connection_status = "CONNECTING"
        drone = System()
        self._system = drone
        await drone.connect(system_address=self._connection_url)

        self.get_logger().info('Waiting for drone connection...')
        start_time = time.monotonic()
        async for state in drone.core.connection_state():
            if state.is_connected:
                self._connected = True
                self._connection_status = "CONNECTED"
                self.get_logger().info('Drone connected! Telemetry active; command bridge waiting for gates.')
                break

            if time.monotonic() - start_time > self._connection_timeout:
                self._connected = False
                self._connection_status = "TIMEOUT"
                self.get_logger().warning('Connection timeout.')
                return

        await asyncio.gather(
            self._stream_battery(drone),
            self._stream_position(drone),
            self._stream_attitude(drone),
            self._stream_velocity(drone),
            self._stream_armed(drone),
            self._stream_flight_mode(drone),
            self._stream_landed_state(drone),
            self._stream_health(drone),
            self._stream_gps_info(drone),
            self._offboard_command_loop(drone),
        )

    async def _stream_battery(self, drone) -> None:
        async for battery in drone.telemetry.battery():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['battery_voltage'] = battery.voltage_v
                self._telemetry_data['battery_remaining'] = battery.remaining_percent * 100.0

    async def _stream_position(self, drone) -> None:
        async for position in drone.telemetry.position():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['latitude'] = position.latitude_deg
                self._telemetry_data['longitude'] = position.longitude_deg
                self._telemetry_data['absolute_altitude'] = position.absolute_altitude_m
                self._telemetry_data['relative_altitude'] = position.relative_altitude_m

    async def _stream_attitude(self, drone) -> None:
        async for attitude in drone.telemetry.attitude_euler():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['roll'] = math.radians(attitude.roll_deg)
                self._telemetry_data['pitch'] = math.radians(attitude.pitch_deg)
                self._telemetry_data['yaw'] = math.radians(attitude.yaw_deg)

    async def _stream_velocity(self, drone) -> None:
        async for velocity in drone.telemetry.velocity_ned():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['velocity_north'] = velocity.north_m_s
                self._telemetry_data['velocity_east'] = velocity.east_m_s
                self._telemetry_data['velocity_down'] = velocity.down_m_s

    async def _stream_armed(self, drone) -> None:
        async for is_armed in drone.telemetry.armed():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['armed'] = is_armed

    async def _stream_flight_mode(self, drone) -> None:
        async for flight_mode in drone.telemetry.flight_mode():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['flight_mode'] = str(flight_mode)

    async def _stream_landed_state(self, drone) -> None:
        async for landed_state in drone.telemetry.landed_state():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['landed_state'] = str(landed_state)

    async def _stream_health(self, drone) -> None:
        async for health in drone.telemetry.health():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['health_all_ok'] = health.is_global_position_ok and health.is_home_position_ok
                self._telemetry_data['health_accelerometer_ok'] = health.is_accelerometer_calibration_ok
                self._telemetry_data['health_gyroscope_ok'] = health.is_gyrometer_calibration_ok
                self._telemetry_data['health_magnetometer_ok'] = health.is_magnetometer_calibration_ok
                self._telemetry_data['health_gps_ok'] = health.is_global_position_ok

    async def _stream_gps_info(self, drone) -> None:
        async for gps_info in drone.telemetry.gps_info():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['gps_num_satellites'] = gps_info.num_satellites
                self._telemetry_data['gps_fix_type'] = gps_info.fix_type.value

    async def _offboard_command_loop(self, drone) -> None:
        try:
            from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
        except ImportError:
            self.get_logger().error('MAVSDK offboard plugin import failed. Check MAVSDK-Python install.')
            return

        period = 1.0 / self._command_rate
        zero_setpoint = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)

        while self._running:
            decision = self._get_command_decision()

            if not decision['executor_enabled']:
                if self._offboard_active and self._stop_offboard_on_disable:
                    await self._stop_offboard(drone)
                self._publish_command_status(BRIDGE_DISABLED)
                await asyncio.sleep(period)
                continue

            if not decision['valid']:
                if self._offboard_active:
                    await self._send_velocity_body(drone, zero_setpoint, BRIDGE_ZERO_SENT, is_zero=True)
                else:
                    self._publish_command_status(str(decision['reason']))
                    self._log_throttled('offboard_blocked', 'info', f'Offboard blocked: {decision["reason"]}', 2.0)
                await asyncio.sleep(period)
                continue

            velocity_setpoint = VelocityBodyYawspeed(
                float(decision['forward_m_s']),
                float(decision['right_m_s']),
                float(decision['down_m_s']),
                float(decision['yawspeed_deg_s']),
            )

            if not self._offboard_active:
                try:
                    # PX4 requires a valid setpoint before switching into Offboard.
                    await drone.offboard.set_velocity_body(zero_setpoint)
                    await drone.offboard.start()
                    self._offboard_active = True
                    self.get_logger().warning('*** PX4 OFFBOARD STARTED by MAVSDK command bridge ***')
                    self._publish_command_status(BRIDGE_OFFBOARD_STARTED, force=True)
                except OffboardError as exc:
                    result = getattr(getattr(exc, '_result', None), 'result', exc)
                    self._publish_command_status(f'{BRIDGE_OFFBOARD_START_FAILED}: {result}', force=True)
                    self._log_throttled(
                        'offboard_start_failed',
                        'warning',
                        f'Offboard start failed: {result}. Drone must already be safely armed/ready; this node will not arm or take off.',
                        2.0,
                    )
                    await asyncio.sleep(period)
                    continue

            await self._send_velocity_body(
                drone,
                velocity_setpoint,
                f'{BRIDGE_SENT}: yaw={decision["yaw_rate_rad_s"]:+.3f}rad/s ({decision["yawspeed_deg_s"]:+.1f}deg/s)',
                is_zero=False,
            )
            await asyncio.sleep(period)

    async def _send_velocity_body(self, drone, velocity_setpoint, status: str, is_zero: bool) -> None:
        try:
            await drone.offboard.set_velocity_body(velocity_setpoint)
            self._px4_send_count += 1
            if is_zero:
                self._px4_zero_count += 1
            self._last_bridge_status = status
            self._publish_command_status(status)
            if not is_zero:
                self._log_throttled('yaw_sent', 'info', f'MAVSDK yaw command sent | {status}', 0.5)
        except Exception as exc:
            self._publish_command_status(f'{BRIDGE_SEND_FAILED}: {str(exc)[:80]}', force=True)
            self._log_throttled('send_failed', 'warning', f'Offboard setpoint send failed: {exc}', 1.0)
            self._offboard_active = False

    async def _stop_offboard(self, drone) -> None:
        try:
            await drone.offboard.stop()
            self._offboard_active = False
            self.get_logger().warning('PX4 Offboard stopped by MAVSDK command bridge.')
            self._publish_command_status(BRIDGE_OFFBOARD_STOPPED, force=True)
        except Exception as exc:
            self._offboard_active = False
            self._publish_command_status(f'OFFBOARD_STOP_FAILED: {str(exc)[:80]}', force=True)
            self._log_throttled('stop_failed', 'warning', f'Offboard stop failed: {exc}', 1.0)

    def _get_command_decision(self) -> dict:
        now = time.monotonic()

        with self._command_lock:
            command = self._latest_command
            command_time = self._latest_command_time

        with self._data_lock:
            armed = bool(self._telemetry_data['armed'])
            battery_remaining = float(self._telemetry_data['battery_remaining'])
            flight_mode = str(self._telemetry_data['flight_mode'])

        decision = {
            'executor_enabled': self._mavsdk_offboard_enabled,
            'valid': False,
            'reason': BRIDGE_DISABLED,
            'forward_m_s': 0.0,
            'right_m_s': 0.0,
            'down_m_s': 0.0,
            'yaw_rate_rad_s': 0.0,
            'yawspeed_deg_s': 0.0,
            'flight_mode': flight_mode,
        }

        if not self._mavsdk_offboard_enabled:
            return decision

        if command is None:
            decision['reason'] = BRIDGE_NO_COMMAND
            return decision

        command_age = now - command_time
        if command_age > self._command_timeout:
            decision['reason'] = f'{BRIDGE_COMMAND_STALE} ({command_age:.2f}s)'
            return decision

        if command.command_type != CMD_VELOCITY:
            decision['reason'] = f'{BRIDGE_COMMAND_IDLE}: type={command.command_type}, status={command.execution_status}'
            return decision

        if not command.executed or command.execution_status != STATUS_SENT:
            decision['reason'] = (
                f'{BRIDGE_COMMAND_NOT_APPROVED}: executed={command.executed}, '
                f'status={command.execution_status}'
            )
            return decision

        if not self._connected:
            decision['reason'] = BRIDGE_NOT_CONNECTED
            return decision

        if self._require_armed_for_offboard and not armed:
            decision['reason'] = BRIDGE_NOT_ARMED
            return decision

        if battery_remaining > 0.0 and battery_remaining < self._min_battery_percent:
            decision['reason'] = f'{BRIDGE_LOW_BATTERY}: {battery_remaining:.1f}%'
            return decision

        yaw_rate_rad_s = self._clamp(
            float(command.yaw_rate),
            -self._max_yaw_rate_rad_s,
            self._max_yaw_rate_rad_s,
        )

        forward_m_s = 0.0
        right_m_s = 0.0
        down_m_s = 0.0
        if self._allow_translation_commands:
            # Disabled by default. Keep yaw-only for early testing.
            forward_m_s = float(command.velocity_forward)
            right_m_s = float(command.velocity_right)
            down_m_s = float(command.velocity_down)

        decision.update(
            {
                'valid': True,
                'reason': BRIDGE_READY,
                'forward_m_s': forward_m_s,
                'right_m_s': right_m_s,
                'down_m_s': down_m_s,
                'yaw_rate_rad_s': yaw_rate_rad_s,
                'yawspeed_deg_s': math.degrees(yaw_rate_rad_s),
            }
        )
        return decision

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def _publish_command_status(self, status: str, force: bool = False) -> None:
        now = time.monotonic()
        if not force and status == self._last_bridge_status and now - self._last_status_publish_time < 1.0:
            return

        self._last_bridge_status = status
        self._last_status_publish_time = now
        msg = String()
        msg.data = status
        self._command_status_pub.publish(msg)
        self._diagnostics.mark_published(
            self._command_status_topic,
            summary=(
                f'status={status}, offboard_enabled={self._mavsdk_offboard_enabled}, '
                f'offboard_active={self._offboard_active}, sent={self._px4_send_count}, zeros={self._px4_zero_count}'
            ),
        )

    def _log_throttled(self, key: str, level: str, message: str, interval: float) -> None:
        now = time.monotonic()
        last = self._last_log_times.get(key, 0.0)
        if now - last < interval:
            return
        self._last_log_times[key] = now
        logger = self.get_logger()
        if level == 'warning':
            logger.warning(message)
        elif level == 'error':
            logger.error(message)
        else:
            logger.info(message)

    def _publish_telemetry(self) -> None:
        msg = DroneTelemetry()
        msg.stamp = self.get_clock().now().to_msg()

        msg.connected = self._connected
        msg.connection_status = self._connection_status

        with self._data_lock:
            msg.battery_voltage = float(self._telemetry_data['battery_voltage'])
            msg.battery_remaining_percent = float(self._telemetry_data['battery_remaining'])
            msg.latitude = float(self._telemetry_data['latitude'])
            msg.longitude = float(self._telemetry_data['longitude'])
            msg.absolute_altitude = float(self._telemetry_data['absolute_altitude'])
            msg.relative_altitude = float(self._telemetry_data['relative_altitude'])
            msg.gps_num_satellites = int(self._telemetry_data['gps_num_satellites'])
            msg.gps_fix_type = int(self._telemetry_data['gps_fix_type'])
            msg.roll = float(self._telemetry_data['roll'])
            msg.pitch = float(self._telemetry_data['pitch'])
            msg.yaw = float(self._telemetry_data['yaw'])
            msg.velocity_north = float(self._telemetry_data['velocity_north'])
            msg.velocity_east = float(self._telemetry_data['velocity_east'])
            msg.velocity_down = float(self._telemetry_data['velocity_down'])
            msg.armed = bool(self._telemetry_data['armed'])
            msg.flight_mode = str(self._telemetry_data['flight_mode'])
            msg.landed_state = str(self._telemetry_data['landed_state'])
            msg.health_all_ok = bool(self._telemetry_data['health_all_ok'])
            msg.health_accelerometer_ok = bool(self._telemetry_data['health_accelerometer_ok'])
            msg.health_gyroscope_ok = bool(self._telemetry_data['health_gyroscope_ok'])
            msg.health_magnetometer_ok = bool(self._telemetry_data['health_magnetometer_ok'])
            msg.health_gps_ok = bool(self._telemetry_data['health_gps_ok'])

        self._telemetry_pub.publish(msg)
        self._telemetry_publish_count += 1
        self._diagnostics.mark_published(
            self._telemetry_topic,
            summary=(
                f"messages={self._telemetry_publish_count}, connected={msg.connected}, "
                f"status={msg.connection_status}, battery={msg.battery_remaining_percent:.1f}%"
            ),
        )

    def _report_status(self) -> None:
        with self._data_lock:
            armed = self._telemetry_data['armed']
            flight_mode = self._telemetry_data['flight_mode']
            battery = self._telemetry_data['battery_remaining']

        with self._command_lock:
            command_age = None if self._latest_command is None else time.monotonic() - self._latest_command_time
            latest_type = 'NONE' if self._latest_command is None else self._latest_command.command_type
            latest_status = 'NONE' if self._latest_command is None else self._latest_command.execution_status
            latest_yaw = 0.0 if self._latest_command is None else float(self._latest_command.yaw_rate)

        age_text = 'never' if command_age is None else f'{command_age:.2f}s'
        self.get_logger().info(
            'MAVSDK bridge status | '
            f'telemetry_published={self._telemetry_publish_count}, connected={self._connected}, '
            f'status={self._connection_status}, armed={armed}, mode={flight_mode}, battery={battery:.1f}%, '
            f'offboard_enabled={self._mavsdk_offboard_enabled}, offboard_active={self._offboard_active}, '
            f'control_msgs={self._control_command_count}, command_age={age_text}, '
            f'latest_type={latest_type}, latest_status={latest_status}, latest_yaw={latest_yaw:+.3f}, '
            f'px4_sends={self._px4_send_count}, zero_sends={self._px4_zero_count}, '
            f'bridge_status={self._last_bridge_status}'
        )

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down PX4 MAVSDK bridge node...')
        self._running = False
        if self._async_thread is not None:
            self._async_thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TelemetryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('telemetry_node').fatal(f'Fatal: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
