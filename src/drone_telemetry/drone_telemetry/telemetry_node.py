"""
PX4 Telemetry node for the drone vision system.

This node connects to PX4 through MAVSDK, reads telemetry data,
and publishes it as ROS 2 messages. It includes automatic reconnection
and connection health monitoring.
"""

import asyncio
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_interfaces.msg import DroneTelemetry


class TelemetryNode(Node):
    """ROS 2 node that bridges PX4/MAVSDK telemetry to ROS topics."""

    def __init__(self) -> None:
        """Initialize the telemetry node."""
        super().__init__('telemetry_node')

        # Declare parameters
        self.declare_parameter('connection_url', 'serial:///dev/ttyACM0:57600')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('connection_timeout', 10.0)

        # Read parameters
        self._connection_url: str = self.get_parameter('connection_url').value
        self._publish_rate: float = self.get_parameter('publish_rate').value
        self._reconnect_interval: float = self.get_parameter('reconnect_interval').value
        self._connection_timeout: float = self.get_parameter('connection_timeout').value

        # MAVSDK state
        self._system = None
        self._mavsdk = None
        self._connected: bool = False
        self._connection_status: str = "DISCONNECTED"

        # Telemetry data (protected by lock)
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

        # QoS
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publisher
        self._telemetry_pub = self.create_publisher(
            DroneTelemetry,
            '/drone/telemetry',
            telemetry_qos
        )

        # Publish timer
        publish_period = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_telemetry)

        # Status reporting timer
        self._status_timer = self.create_timer(10.0, self._report_status)

        # Start MAVSDK connection in background thread
        self._async_thread: Optional[threading.Thread] = None
        self._running: bool = True
        self._start_mavsdk_connection()

        self.get_logger().info(
            f'Telemetry node initialized: url={self._connection_url}, '
            f'rate={self._publish_rate}Hz'
        )

    def _start_mavsdk_connection(self) -> None:
        """Start the MAVSDK connection in a background asyncio thread."""
        self._async_thread = threading.Thread(
            target=self._run_async_loop,
            daemon=True,
            name='mavsdk_telemetry_thread'
        )
        self._async_thread.start()

    def _run_async_loop(self) -> None:
        """Run the asyncio event loop for MAVSDK in a separate thread."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            loop.run_until_complete(self._mavsdk_main())
        except Exception as e:
            self.get_logger().error(f'MAVSDK async loop error: {e}')
        finally:
            loop.close()

    async def _mavsdk_main(self) -> None:
        """Main MAVSDK coroutine that handles connection and telemetry streaming."""
        while self._running:
            try:
                await self._connect_and_stream()
            except Exception as e:
                self.get_logger().error(f'MAVSDK connection error: {e}')
                self._connected = False
                self._connection_status = f"ERROR: {str(e)[:50]}"

            if self._running:
                self.get_logger().info(
                    f'Reconnecting in {self._reconnect_interval}s...'
                )
                await asyncio.sleep(self._reconnect_interval)

    async def _connect_and_stream(self) -> None:
        """Connect to the drone and stream telemetry data."""
        try:
            from mavsdk import System

            self.get_logger().info(f'Connecting to PX4 at: {self._connection_url}')
            self._connection_status = "CONNECTING"

            drone = System()
            await drone.connect(system_address=self._connection_url)

            # Wait for connection
            self.get_logger().info('Waiting for drone connection...')
            start_time = time.time()

            async for state in drone.core.connection_state():
                if state.is_connected:
                    self._connected = True
                    self._connection_status = "CONNECTED"
                    self.get_logger().info('Drone connected!')
                    break

                if time.time() - start_time > self._connection_timeout:
                    self._connection_status = "TIMEOUT"
                    self.get_logger().warning('Connection timeout.')
                    return

            # Start streaming telemetry
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
            )

        except ImportError:
            self.get_logger().error(
                'MAVSDK-Python not installed. Install with: pip install mavsdk'
            )
            self._connection_status = "MAVSDK_NOT_INSTALLED"
            self._running = False

    async def _stream_battery(self, drone) -> None:
        """Stream battery telemetry."""
        async for battery in drone.telemetry.battery():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['battery_voltage'] = battery.voltage_v
                self._telemetry_data['battery_remaining'] = battery.remaining_percent * 100.0

    async def _stream_position(self, drone) -> None:
        """Stream position telemetry."""
        async for position in drone.telemetry.position():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['latitude'] = position.latitude_deg
                self._telemetry_data['longitude'] = position.longitude_deg
                self._telemetry_data['absolute_altitude'] = position.absolute_altitude_m
                self._telemetry_data['relative_altitude'] = position.relative_altitude_m

    async def _stream_attitude(self, drone) -> None:
        """Stream attitude telemetry."""
        async for attitude in drone.telemetry.attitude_euler():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['roll'] = attitude.roll_deg * 0.017453  # to radians
                self._telemetry_data['pitch'] = attitude.pitch_deg * 0.017453
                self._telemetry_data['yaw'] = attitude.yaw_deg * 0.017453

    async def _stream_velocity(self, drone) -> None:
        """Stream velocity telemetry."""
        async for velocity in drone.telemetry.velocity_ned():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['velocity_north'] = velocity.north_m_s
                self._telemetry_data['velocity_east'] = velocity.east_m_s
                self._telemetry_data['velocity_down'] = velocity.down_m_s

    async def _stream_armed(self, drone) -> None:
        """Stream armed state."""
        async for is_armed in drone.telemetry.armed():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['armed'] = is_armed

    async def _stream_flight_mode(self, drone) -> None:
        """Stream flight mode."""
        async for flight_mode in drone.telemetry.flight_mode():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['flight_mode'] = str(flight_mode)

    async def _stream_landed_state(self, drone) -> None:
        """Stream landed state."""
        async for landed_state in drone.telemetry.landed_state():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['landed_state'] = str(landed_state)

    async def _stream_health(self, drone) -> None:
        """Stream health information."""
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
        """Stream GPS info."""
        async for gps_info in drone.telemetry.gps_info():
            if not self._running:
                return
            with self._data_lock:
                self._telemetry_data['gps_num_satellites'] = gps_info.num_satellites
                self._telemetry_data['gps_fix_type'] = gps_info.fix_type.value

    def _publish_telemetry(self) -> None:
        """Publish the current telemetry state as a ROS message."""
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

    def _report_status(self) -> None:
        """Report telemetry node status."""
        with self._data_lock:
            self.get_logger().info(
                f'Telemetry status: connected={self._connected}, '
                f'status={self._connection_status}, '
                f'armed={self._telemetry_data["armed"]}, '
                f'mode={self._telemetry_data["flight_mode"]}, '
                f'battery={self._telemetry_data["battery_remaining"]:.1f}%'
            )

    def destroy_node(self) -> None:
        """Clean up resources."""
        self.get_logger().info('Shutting down telemetry node...')
        self._running = False
        if self._async_thread is not None:
            self._async_thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the telemetry node."""
    rclpy.init(args=args)
    node = TelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()