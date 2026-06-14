"""
Fake telemetry node that simulates PX4 drone telemetry.

This node publishes simulated telemetry data on the same topic as the real
telemetry node, enabling full system testing without a drone connection.
It simulates realistic battery drain, GPS coordinates, attitude changes,
and state transitions.
"""

import math
import time
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_interfaces.msg import DroneTelemetry


class FakeTelemetryNode(Node):
    """ROS 2 node that generates simulated drone telemetry."""

    def __init__(self) -> None:
        """Initialize the fake telemetry node."""
        super().__init__('fake_telemetry_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('simulate_armed', False)
        self.declare_parameter('simulate_flying', False)
        self.declare_parameter('initial_battery_percent', 85.0)
        self.declare_parameter('battery_drain_rate', 0.01)
        self.declare_parameter('base_latitude', 47.3769)
        self.declare_parameter('base_longitude', 8.5417)
        self.declare_parameter('base_altitude', 408.0)
        self.declare_parameter('flight_altitude', 10.0)
        self.declare_parameter('flight_mode', 'HOLD')
        self.declare_parameter('gps_satellites', 12)
        self.declare_parameter('simulate_gps_noise', True)
        self.declare_parameter('telemetry_topic', '/drone/telemetry')

        # Read parameters
        self._publish_rate: float = self.get_parameter('publish_rate').value
        self._simulate_armed: bool = self.get_parameter('simulate_armed').value
        self._simulate_flying: bool = self.get_parameter('simulate_flying').value
        self._initial_battery: float = self.get_parameter('initial_battery_percent').value
        self._battery_drain_rate: float = self.get_parameter('battery_drain_rate').value
        self._base_latitude: float = self.get_parameter('base_latitude').value
        self._base_longitude: float = self.get_parameter('base_longitude').value
        self._base_altitude: float = self.get_parameter('base_altitude').value
        self._flight_altitude: float = self.get_parameter('flight_altitude').value
        self._flight_mode: str = self.get_parameter('flight_mode').value
        self._gps_satellites: int = self.get_parameter('gps_satellites').value
        self._simulate_gps_noise: bool = self.get_parameter('simulate_gps_noise').value
        self._telemetry_topic: str = str(self.get_parameter('telemetry_topic').value)

        # State
        self._start_time: float = time.time()
        self._battery_percent: float = self._initial_battery
        self._current_altitude: float = 0.0

        # Simulated attitude
        self._roll: float = 0.0
        self._pitch: float = 0.0
        self._yaw: float = 0.0

        # QoS
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publisher
        self._telemetry_pub = self.create_publisher(
            DroneTelemetry,
            self._telemetry_topic,
            telemetry_qos
        )

        # Publish timer
        publish_period = 1.0 / self._publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_telemetry)

        # Status timer
        self._status_timer = self.create_timer(10.0, self._report_status)

        # Altitude ramp (if simulating flying)
        if self._simulate_flying:
            self._current_altitude = self._flight_altitude

        self.get_logger().info(
            f'Fake telemetry node initialized: topic={self._telemetry_topic}, '
            f'rate={self._publish_rate}Hz, armed={self._simulate_armed}, '
            f'flying={self._simulate_flying}, battery={self._initial_battery}%'
        )

    def _publish_telemetry(self) -> None:
        """Generate and publish simulated telemetry."""
        t = time.time() - self._start_time

        msg = DroneTelemetry()
        msg.stamp = self.get_clock().now().to_msg()

        # Connection - always "connected" in simulation
        msg.connected = True
        msg.connection_status = "CONNECTED (SIMULATED)"

        # Battery simulation
        self._battery_percent -= self._battery_drain_rate / self._publish_rate
        self._battery_percent = max(0.0, self._battery_percent)

        cell_count = 4
        cell_voltage_full = 4.2
        cell_voltage_empty = 3.3
        voltage_per_cell = cell_voltage_empty + (cell_voltage_full - cell_voltage_empty) * (self._battery_percent / 100.0)
        msg.battery_voltage = float(voltage_per_cell * cell_count)
        msg.battery_remaining_percent = float(self._battery_percent)

        # GPS simulation
        gps_noise_lat = 0.0
        gps_noise_lon = 0.0
        if self._simulate_gps_noise:
            gps_noise_lat = random.gauss(0, 0.000001)
            gps_noise_lon = random.gauss(0, 0.000001)

        msg.latitude = self._base_latitude + gps_noise_lat
        msg.longitude = self._base_longitude + gps_noise_lon
        msg.absolute_altitude = float(self._base_altitude + self._current_altitude)
        msg.relative_altitude = float(self._current_altitude)
        msg.gps_num_satellites = self._gps_satellites + random.randint(-1, 1)
        msg.gps_fix_type = 3  # 3D fix

        # Attitude simulation - slight oscillation when flying
        if self._simulate_flying:
            self._roll = math.sin(t * 0.5) * 0.03 + random.gauss(0, 0.005)
            self._pitch = math.cos(t * 0.3) * 0.02 + random.gauss(0, 0.005)
            self._yaw += random.gauss(0, 0.001)
        else:
            self._roll = random.gauss(0, 0.001)
            self._pitch = random.gauss(0, 0.001)
            self._yaw = 0.0

        msg.roll = float(self._roll)
        msg.pitch = float(self._pitch)
        msg.yaw = float(self._yaw)

        # Velocity simulation
        if self._simulate_flying:
            msg.velocity_north = float(random.gauss(0, 0.1))
            msg.velocity_east = float(random.gauss(0, 0.1))
            msg.velocity_down = float(random.gauss(0, 0.05))
        else:
            msg.velocity_north = 0.0
            msg.velocity_east = 0.0
            msg.velocity_down = 0.0

        # State
        msg.armed = self._simulate_armed
        msg.flight_mode = self._flight_mode

        if self._simulate_flying:
            msg.landed_state = "IN_AIR"
        elif self._simulate_armed:
            msg.landed_state = "ON_GROUND"
        else:
            msg.landed_state = "ON_GROUND"

        # Health - all OK in simulation
        msg.health_all_ok = True
        msg.health_accelerometer_ok = True
        msg.health_gyroscope_ok = True
        msg.health_magnetometer_ok = True
        msg.health_gps_ok = True

        self._telemetry_pub.publish(msg)

    def _report_status(self) -> None:
        """Report status."""
        self.get_logger().info(
            f'Fake telemetry: battery={self._battery_percent:.1f}%, '
            f'alt={self._current_altitude:.1f}m, '
            f'armed={self._simulate_armed}, mode={self._flight_mode}'
        )

    def destroy_node(self) -> None:
        """Clean up."""
        self.get_logger().info('Shutting down fake telemetry node...')
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the fake telemetry node."""
    rclpy.init(args=args)
    node = FakeTelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()