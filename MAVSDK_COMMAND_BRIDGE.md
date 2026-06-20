# MAVSDK Command Bridge

This update turns `drone_telemetry.telemetry_node` into a single PX4 MAVSDK bridge:

```text
Pixhawk <-> MAVSDK bridge -> /drone/telemetry
/drone/control/command -> MAVSDK bridge -> PX4 Offboard velocity-body setpoints
/drone/mavsdk/offboard_request -> autonomy manager operator request
/drone/mavsdk/offboard_enable -> MAVSDK bridge safety gate
```

The bridge owns the only MAVSDK connection to the Pixhawk. This avoids two ROS nodes fighting over the same serial device such as `/dev/ttyACM0`.

## Safety behavior

The first implementation is intentionally conservative:

- It does **not** arm the drone.
- It does **not** take off.
- It does **not** command forward, right, or down motion by default.
- It only sends yaw-rate setpoints through PX4 Offboard when both gates are enabled:
  - `/drone/autonomy/enabled == true`
  - `/drone/mavsdk/offboard_enable == true`
- It sends zero body velocity/yawspeed when the command is idle, stale, not approved, or target tracking is lost.
- It stops PX4 Offboard when `/drone/mavsdk/offboard_enable` is set to false.

## Topics

| Topic | Type | Purpose |
|---|---|---|
| `/drone/telemetry` | `drone_interfaces/msg/DroneTelemetry` | Telemetry from PX4/MAVSDK |
| `/drone/control/command` | `drone_interfaces/msg/ControlCommand` | Command produced by the control node |
| `/drone/mavsdk/offboard_request` | `std_msgs/msg/Bool` | Operator/dashboard request for the MAVSDK/PX4 executor gate |
| `/drone/mavsdk/offboard_enable` | `std_msgs/msg/Bool` | Second hard gate for actual MAVSDK/PX4 command sending, published by the autonomy manager |
| `/drone/mavsdk/command_status` | `std_msgs/msg/String` | Status/debug output from the MAVSDK command bridge |

## Parameters

Configured in `src/drone_bringup/config/full_system_params.yaml`. The operator request belongs to `autonomy_manager_node`; the actual executor gate belongs to `telemetry_node`:

```yaml
autonomy_manager_node:
  ros__parameters:
    offboard_request_topic: "/drone/mavsdk/offboard_request"
    offboard_enable_topic: "/drone/mavsdk/offboard_enable"

telemetry_node:
  ros__parameters:
    control_command_topic: "/drone/control/command"
    offboard_enable_topic: "/drone/mavsdk/offboard_enable"
    command_status_topic: "/drone/mavsdk/command_status"
    mavsdk_offboard_enabled: false
    command_rate: 20.0
    command_timeout: 0.5
    require_armed_for_offboard: true
    min_battery_percent: 20.0
    max_yaw_rate_rad_s: 1.0
    allow_translation_commands: false
    stop_offboard_on_disable: true
```

## Enable sequence for bench testing

Build and source:

```bash
cd ~/drone_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
```

Launch the full system:

```bash
ros2 launch drone_bringup full_system_launch.py connection_url:="serial:///dev/ttyACM0:57600"
```

Watch the generated command and MAVSDK bridge status:

```bash
ros2 topic echo /drone/control/command
ros2 topic echo /drone/mavsdk/command_status
```

Request autonomy through the manager:

```bash
ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool "{data: true}"
```

Request the MAVSDK/PX4 executor gate. The autonomy manager will only publish actual `/drone/mavsdk/offboard_enable == true` when the safety state is READY/TRACKING/TARGET_LOST:

```bash
ros2 topic pub --once /drone/mavsdk/offboard_request std_msgs/msg/Bool "{data: true}"
```

Disable the MAVSDK executor request:

```bash
ros2 topic pub --once /drone/mavsdk/offboard_request std_msgs/msg/Bool "{data: false}"
```

Disable autonomy through the manager:

```bash
ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool "{data: false}"
```

## Expected chain

With a locked target and passing telemetry safety gates:

```text
Target moves
-> /drone/tracking/target_error.error_x changes
-> control_node publishes /drone/control/command.yaw_rate in rad/s
-> telemetry_node converts yaw_rate to MAVSDK VelocityBodyYawspeed yawspeed_deg_s
-> PX4 receives yaw-only Offboard setpoints
```

If the target is lost/stale or either gate is disabled:

```text
/drone/control/command becomes IDLE or not approved
-> MAVSDK bridge sends zero body velocity/yawspeed if Offboard is active
-> MAVSDK bridge stops Offboard when /drone/mavsdk/offboard_enable is false
```

## Practical note

MAVSDK `VelocityBodyYawspeed` expects yaw speed in degrees/second. The ROS `ControlCommand.yaw_rate` field is radians/second, so the bridge converts radians/second to degrees/second before sending the setpoint.
